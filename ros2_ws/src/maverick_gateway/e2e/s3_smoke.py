"""End-to-end smoke test of the S3 host stack on synthetic camera topics (no Gazebo, no network).

    From the repo root:
    docker compose -f docker/compose.yaml --profile tools run --rm -T tools bash -lc \
        "cd /maverick && python3 ros2_ws/src/maverick_gateway/e2e/s3_smoke.py --seed 1000003"

A stand-in drone (SimDrone, in this process) renders the five /maverick/camera/* topics of the S3
integration contract from a worldgen apartment with maverick_mapping.synthetic (depth, 80x60 for
speed) and a class-id image (the segmentation camera), and flies the goto goals that the explore
skill sends to /maverick/flight/goal (kinematic model, <= 1 m/s). The real nodes run as separate
processes: mapping_node, memory_node, skill_node, gateway. A websocket client plays the console.

Checks: the map grows, rooms appear, objects and detections flow, every gateway frame validates
against the protocol schema, the state-on-connect order holds, text commands start skills.
Prints one JSON summary line and exits 0 when all checks pass.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import Counter
from pathlib import Path

import numpy as np

SRC = Path(__file__).resolve().parents[2]
for p in (SRC / "maverick_mapping", SRC / "maverick_memory", SRC / "maverick_skills",
          SRC / "maverick_gateway", SRC.parents[1] / "sim" / "worldgen"):  # fmt: skip
    sys.path.insert(0, str(p))

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from maverick_gateway.schema import validate  # noqa: E402
from maverick_mapping import topics as T  # noqa: E402
from maverick_mapping.frames import ned_to_enu, yaw_ned_to_enu  # noqa: E402
from maverick_mapping.ros_conv import matrix_to_quat  # noqa: E402
from maverick_mapping.synthetic import CameraSim, KinematicDrone, Scene  # noqa: E402
from maverick_memory.semantic_detector import CLASS_IDS  # noqa: E402
from maverick_memory.sim_detector import SimDetector  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)


# --------------------------------------------------------------------------------------------
class SimDrone(Node):
    """Publishes the camera topics and follows goto goals. Stands in for sim + bridge + flight."""

    def __init__(self, seed: int, speed: float) -> None:
        super().__init__("sim_drone")
        self.scene = Scene.from_seed(seed)
        sx, sy, syaw = self.scene.spawn
        self.alt = 1.2
        self.drone = KinematicDrone((sx, sy, self.alt), syaw, seed=seed)
        self.cam = CameraSim(80, 60, seed=seed + 2)
        self.det = SimDetector(self.scene.manifest, self.cam.intr, seed=seed)
        self.class_of = [CLASS_IDS.get(lab, 0) for lab in self.det.labels]
        self.target: tuple[float, float] | None = None
        self.goal_yaw: float | None = None
        self.tick_n = 0
        self.speed = speed
        self.goals_received = 0
        self.dt = 0.1
        q = qos_profile_sensor_data
        self.p_depth = self.create_publisher(Image, T.CAM_DEPTH, q)
        self.p_sem = self.create_publisher(Image, T.CAM_SEMANTIC, q)
        self.p_rgb = self.create_publisher(CompressedImage, T.CAM_RGB, q)
        self.p_pose = self.create_publisher(PoseStamped, T.CAM_POSE, q)
        self.p_info = self.create_publisher(CameraInfo, T.CAM_INFO, LATCHED)
        self.p_status = self.create_publisher(String, T.FLIGHT_STATUS, 10)
        self.create_subscription(String, T.FLIGHT_GOAL, self._on_goal, 10)
        self.create_timer(self.dt / speed, self._tick)
        self.create_timer(1.0, self._status)
        self._publish_info()
        self.create_timer(1.0, self._publish_info)
        from PIL import Image as PILImage

        buf = io.BytesIO()
        PILImage.new("RGB", (640, 480), (90, 110, 130)).save(buf, "JPEG", quality=60)
        self.jpeg = buf.getvalue()

    def _status(self) -> None:
        self.p_status.publish(String(data=json.dumps({"phase": "hover", "telemetry_age_s": 0.1})))

    def _publish_info(self) -> None:
        i = self.cam.intr
        m = CameraInfo()
        m.header.frame_id = "camera_optical"
        m.header.stamp = self.get_clock().now().to_msg()
        m.width, m.height = i.width, i.height
        m.k = [i.fx, 0.0, i.cx, 0.0, i.fy, i.cy, 0.0, 0.0, 1.0]
        self.p_info.publish(m)

    def _on_goal(self, msg: String) -> None:
        g = json.loads(msg.data)
        e = ned_to_enu(g["x"], g["y"], g["z"])
        self.target = (e[0], e[1])
        self.goal_yaw = yaw_ned_to_enu(g["yaw"]) if "yaw" in g else None
        self.goals_received += 1

    def _move(self) -> None:
        d = self.drone
        if self.target is None:
            d.hold(self.dt)
            return
        tgt = np.array(self.target)
        dsp = tgt - d.est[:2]
        if d.sp is None:
            d.sp = d.est[:2].copy()
        far = float(np.linalg.norm(tgt - d.sp)) > 0.3
        heading = math.atan2(dsp[1], dsp[0]) if far or self.goal_yaw is None else self.goal_yaw
        e = (heading - d.yaw + math.pi) % (2 * math.pi) - math.pi
        cap = d.vmax * max(0.0, 1.0 - abs(e) / math.radians(50.0)) if far else d.vmax
        d.turn(heading, self.dt)
        d.track(tgt, cap, self.dt)

    def _tick(self) -> None:
        self._move()
        self.tick_n += 1
        if self.tick_n % 3:  # one camera frame per 0.3 simulated seconds
            return
        t_true, r_true = self.drone.camera_pose(est=False)
        depth = self.cam.render(self.scene, t_true, r_true)
        inst = self.det.instance_image(depth, t_true, r_true)
        sem = np.zeros(depth.shape, np.uint8)
        for k, cid in enumerate(self.class_of):
            sem[inst == k] = cid
        t_est, r_est = self.drone.camera_pose(est=True)
        stamp = self.get_clock().now().to_msg()
        mm = np.where(np.isfinite(depth), np.clip(depth * 1000.0, 1, 65535), 0).astype(np.uint16)
        self.p_depth.publish(self._image(mm, "16UC1", stamp))
        self.p_sem.publish(self._image(sem, "mono8", stamp))
        rgb = CompressedImage()
        rgb.header.stamp, rgb.format, rgb.data = stamp, "jpeg", self.jpeg
        self.p_rgb.publish(rgb)
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = stamp, "map"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, t_est)
        o = pose.pose.orientation
        o.x, o.y, o.z, o.w = matrix_to_quat(r_est)
        self.p_pose.publish(pose)

    @staticmethod
    def _image(a: np.ndarray, enc: str, stamp) -> Image:
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, "camera_optical"
        m.height, m.width, m.encoding = a.shape[0], a.shape[1], enc
        m.step, m.is_bigendian, m.data = a.strides[0], 0, a.tobytes()
        return m


# --------------------------------------------------------------------------------------------
class WsClient(threading.Thread):
    """Minimal websocket client (the console's role): collects and validates every frame."""

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.frames: list[dict] = []
        self.errors: list[str] = []
        self.counts: Counter = Counter()
        self.lock = threading.Lock()
        self.sock: socket.socket | None = None
        self.occupied = 0
        self.connected = threading.Event()

    def run(self) -> None:
        s = socket.create_connection(("127.0.0.1", self.port), timeout=10)
        s.sendall(
            b"GET / HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            b"Sec-WebSocket-Key: " + base64.b64encode(os.urandom(16)) + b"\r\n"
            b"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += s.recv(4096)
        assert b" 101 " in buf.split(b"\r\n")[0]
        buf = buf.split(b"\r\n\r\n", 1)[1]
        self.sock = s
        self.connected.set()
        s.settimeout(None)
        while True:
            while len(buf) < 2:
                c = s.recv(65536)
                if not c:
                    return
                buf += c
            n = buf[1] & 0x7F
            off = 2
            if n == 126:
                need, off = 4, 4
            elif n == 127:
                need, off = 10, 10
            else:
                need = 2
            while len(buf) < need:
                buf += s.recv(65536)
            if n == 126:
                n = struct.unpack("!H", buf[2:4])[0]
            elif n == 127:
                n = struct.unpack("!Q", buf[2:10])[0]
            while len(buf) < off + n:
                c = s.recv(1 << 20)
                if not c:
                    return
                buf += c
            payload, buf = buf[off : off + n], buf[off + n :]
            if payload:
                self._frame(json.loads(payload))

    def _frame(self, f: dict) -> None:
        errs = validate(f)
        with self.lock:
            self.frames.append(f)
            self.counts[f.get("type")] += 1
            self.errors += errs
            if f.get("type") == "map_update":
                d = f["data"]
                self.occupied = (0 if d["reset"] else self.occupied) + (
                    len(d["add"]) - len(d["remove"])
                ) // 3

    def send_text(self, text: str) -> None:
        payload = json.dumps({"type": "text_command", "t": time.time(), "data": {"text": text}})
        raw = payload.encode()
        key = os.urandom(4)
        head = b"\x81" + (bytes([0x80 | len(raw)]) if len(raw) < 126 else
                          bytes([0x80 | 126]) + struct.pack("!H", len(raw)))  # fmt: skip
        self.sock.sendall(head + key + bytes(b ^ key[i & 3] for i, b in enumerate(raw)))

    def last(self, type_: str) -> dict | None:
        with self.lock:
            for f in reversed(self.frames):
                if f["type"] == type_:
                    return f["data"]
        return None

    def progress_states(self) -> list[tuple[str, str, str]]:
        with self.lock:
            return [
                (f["data"]["skill"], f["data"]["state"], f["data"]["message"])
                for f in self.frames
                if f["type"] == "skill_progress"
            ]


# --------------------------------------------------------------------------------------------
def spawn_nodes(port: int, seed: int, rate: float) -> list[subprocess.Popen]:
    env = dict(os.environ)
    pkgs = ("maverick_mapping", "maverick_memory", "maverick_skills", "maverick_gateway")
    env["PYTHONPATH"] = os.pathsep.join(
        [str(SRC / d) for d in pkgs]
        + [str(SRC.parents[1] / "sim" / "worldgen"), env.get("PYTHONPATH", "")]
    )
    env["CYCLONEDDS_URI"] = "file://" + str(
        SRC.parents[1] / "docker/zenoh/cyclonedds-localhost.xml"
    )
    ros = ["--ros-args", "-p"]
    nodes = [
        ("maverick_mapping.mapping_node", [f"max_rate_hz:={rate:.1f}"]),
        ("maverick_memory.memory_node", [f"max_rate_hz:={rate:.1f}", f"seed:={seed}"]),
        ("maverick_skills.skill_node", ["require_hover:=true"]),
        ("maverick_gateway.gateway_node", [f"port:={port}", "host:=127.0.0.1"]),
    ]
    procs = []
    for mod, params in nodes:
        args = [sys.executable, "-m", mod, *ros, params[0]]
        for extra in params[1:]:
            args += ["-p", extra]
        procs.append(subprocess.Popen(args, env=env))
    return procs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1000003)
    ap.add_argument("--speed", type=float, default=3.0, help="simulated seconds per wall second")
    ap.add_argument("--port", type=int, default=18765)
    ap.add_argument("--explore-wall-s", type=float, default=75.0)
    ap.add_argument("--min-voxels", type=int, default=2000)
    args = ap.parse_args()

    rclpy.init()
    drone = SimDrone(args.seed, args.speed)
    from rclpy.executors import SingleThreadedExecutor

    ex = SingleThreadedExecutor()
    ex.add_node(drone)
    procs = spawn_nodes(args.port, args.seed, rate=5.0 * args.speed)
    results: dict = {"seed": args.seed, "speed": args.speed}
    checks: dict[str, bool] = {}
    ws = WsClient(args.port)

    def spin(seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.02)

    try:
        for _ in range(100):  # wait for the gateway socket
            try:
                socket.create_connection(("127.0.0.1", args.port), timeout=0.2).close()
                break
            except OSError:
                spin(0.2)
        spin(1.0)
        ws.start()
        ws.connected.wait(10)
        spin(2.0)
        # the console connects before anything is known: first frames must be valid and ordered
        t_start = time.monotonic()
        ws.send_text("explore for 5 minutes")
        last_report = 0.0
        vox_early = None
        while time.monotonic() - t_start < args.explore_wall_s:
            spin(0.5)
            rooms = ws.last("rooms")
            if time.monotonic() - last_report > 15:
                last_report = time.monotonic()
                print(
                    f"[{time.monotonic() - t_start:5.0f}s] voxels={ws.occupied}"
                    f" frames={dict(ws.counts)}"
                    f" rooms={len((rooms or {}).get('rooms', []))} goals={drone.goals_received}",
                    flush=True,
                )
            if vox_early is None and time.monotonic() - t_start > 8:
                vox_early = ws.occupied
        vox_a = ws.occupied
        rooms = (ws.last("rooms") or {}).get("rooms", [])
        objs = (ws.last("objects") or {}).get("objects", [])
        results.update(
            explore_wall_s=round(time.monotonic() - t_start, 1),
            occupied_voxels=vox_a,
            occupied_voxels_at_8s=vox_early,
            rooms=[(r["id"], r["type"], round(r["confidence"], 2)) for r in rooms],
            objects=Counter(o["label"] for o in objs),
            sim_pos=[round(float(v), 2) for v in drone.drone.pos],
            goals_received=drone.goals_received,
        )
        # --- a console that connects late gets the state first
        late = WsClient(args.port)
        late.start()
        late.connected.wait(10)
        spin(1.0)
        order = [f["type"] for f in late.frames]
        results["late_connect_first"] = order[:8]
        checks["late_connect_map_first"] = bool(order) and order[0] == "map_update"
        checks["late_connect_reset"] = (
            bool(late.frames) and late.frames[0]["data"].get("reset") is True
        )
        checks["late_connect_voxels_match"] = abs(late.occupied - ws.occupied) <= max(
            50, 0.05 * ws.occupied
        )
        # --- go to a room that exists, by command
        typed = [r["type"] for r in rooms if r["type"] not in ("unknown", "other", "hallway")]
        room_word = {"living": "living room"}.get(typed[0], typed[0]) if typed else "kitchen"
        n_before = len(ws.progress_states())
        ws.send_text("stop")
        spin(2.0)
        ws.send_text(f"go to the {room_word}")
        t1 = time.monotonic()
        while time.monotonic() - t1 < 90:
            spin(0.5)
            st = ws.progress_states()[n_before:]
            if any(s[0] == "goto" and s[1] in ("succeeded", "failed") for s in st):
                break
        goto = [s for s in ws.progress_states()[n_before:] if s[0] == "goto"]
        results["goto"] = {"room": room_word, "last": goto[-1] if goto else None}
        spin(1.0)
        with ws.lock:
            counts = dict(ws.counts)
            transcripts = [f["data"]["text"] for f in ws.frames if f["type"] == "transcript"]
        results.update(frame_counts=counts, frame_errors=ws.errors[:10], transcript=transcripts[:6])
        checks.update(
            map_grows=vox_a >= args.min_voxels and vox_a > 1.25 * (vox_early or 0),
            drone_flew=drone.goals_received >= 5,
            rooms_appear=len(rooms) >= 1,
            objects_appear=len(objs) >= 1,
            all_frames_valid=not ws.errors and not late.errors,
            events_present=all(
                counts.get(k, 0) > 0
                for k in (
                    "pose",
                    "camera_frame",
                    "detections",
                    "map_update",
                    "frontiers",
                    "path",
                    "rooms",
                    "objects",
                    "skill_progress",
                    "status",
                    "transcript",
                    "agent_step",
                )  # fmt: skip
            ),
            explore_started=any(s[0] == "explore" for s in ws.progress_states()),
            goto_ran=bool(goto),
        )
        results["goto_succeeded"] = bool(goto and goto[-1][1] == "succeeded")
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
        drone.destroy_node()
        rclpy.try_shutdown()
    results["checks"] = checks
    results["pass"] = all(checks.values())
    print(json.dumps(results, default=str), flush=True)
    return 0 if results["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
