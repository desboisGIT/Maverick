"""ROS 2 node `gateway`: websocket bridge between the host stack and the Mission Console.

Implements docs/gateway-events.md (protocol v1). Display and natural-language input only: it
never publishes setpoints, and losing the websocket changes nothing about the flight. A text
command becomes a /maverick/skill/request (explore / go to a room / cancel) or the flight
mission "land"; the S4 agent will replace the keyword parser.

Topics in
  /maverick/camera/pose (pose), /maverick/camera/rgb/compressed (camera_frame), camera/info (hfov),
  /maverick/memory/detections, /maverick/map/voxels (map_update), /maverick/map/frontiers,
  /maverick/memory/rooms, /maverick/memory/objects, /maverick/skill/path, /maverick/skill/progress,
  /maverick/flight/status (status, with VRAM from nvidia-smi when available)
Topics out
  /maverick/skill/request, /maverick/flight/mission ("land"), /maverick/map/voxels_request

Parameters: host (0.0.0.0), port (8765), camera_max_hz (5.0), pose_max_hz (10.0), battery_pct
(100.0: placeholder, the simulation has no battery model), vram_total_gb (16.0), default_budget_s.
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from maverick_mapping import topics as T
from maverick_mapping.ros_conv import (
    RateLimiter,
    hfov_deg,
    info_to_intrinsics,
    pose_to_tr,
    stamp_key,
    stamp_to_sec,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Empty, String

from .protocol import (
    PATH_STATES,
    ChangeFilter,
    GatewayState,
    PoseTracker,
    agent_step_data,
    camera_frame_data,
    check_skill_progress,
    detections_data,
    envelope,
    parse_command,
    parse_nvidia_smi,
    status_data,
)
from .wsserver import WebSocketServer

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
RELIABLE = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)


class GatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("gateway")
        p = self.declare_parameter
        self.default_budget = float(p("default_budget_s", 360.0).value)
        self.battery_pct = float(p("battery_pct", 100.0).value)
        self.vram_total = float(p("vram_total_gb", 16.0).value)
        self.state = GatewayState()
        self.lock = threading.Lock()
        self.changes = ChangeFilter()
        self.pose = PoseTracker()
        self.cam_limit = RateLimiter(float(p("camera_max_hz", 5.0).value), time.monotonic)
        self.pose_limit = RateLimiter(float(p("pose_max_hz", 10.0).value), time.monotonic)
        self.hfov = 73.0
        self.seq = 0
        self.stamp_seq: dict[int, int] = {}
        self.flight: dict | None = None
        self.skill: dict | None = None
        self.plan_id, self.plan_goal, self.plan_t0 = "", "", 0.0
        self.vram = (0.0, self.vram_total)
        self.msg_n = 0
        self.got_voxels = False

        self.ws = WebSocketServer(
            str(p("host", "0.0.0.0").value),
            int(p("port", 8765).value),
            self._on_connect,
            self._on_ws_message,
        )

        q = qos_profile_sensor_data
        self.create_subscription(PoseStamped, T.CAM_POSE, self._on_pose, q)
        self.create_subscription(CompressedImage, T.CAM_RGB, self._on_rgb, q)
        for qos in (q,):  # sensor-data QoS only (see mapping_node)
            self.create_subscription(CameraInfo, T.CAM_INFO, self._on_info, qos)
        self.create_subscription(String, T.MEM_DETECTIONS, self._on_detections, RELIABLE)
        self.create_subscription(String, T.MAP_VOXELS, self._on_voxels, RELIABLE)
        self.create_subscription(
            String, T.MAP_FRONTIERS, self._wrap("frontiers", "frontiers"), LATCHED
        )
        self.create_subscription(String, T.MEM_ROOMS, self._wrap("rooms", "rooms"), LATCHED)
        self.create_subscription(String, T.MEM_OBJECTS, self._wrap("objects", "objects"), LATCHED)
        self.create_subscription(String, T.SKILL_PATH, self._on_path, LATCHED)
        self.create_subscription(String, T.SKILL_PROGRESS, self._on_progress, LATCHED)
        self.create_subscription(String, T.FLIGHT_STATUS, self._on_flight, 10)
        self.req_pub = self.create_publisher(String, T.SKILL_REQUEST, RELIABLE)
        self.mission_pub = self.create_publisher(String, T.FLIGHT_MISSION, 10)
        self.vox_req_pub = self.create_publisher(Empty, T.MAP_VOXELS_REQUEST, 10)
        self.create_timer(1.0, self._status_tick)
        self.create_timer(5.0, self._vram_tick)
        self.create_timer(2.0, self._voxel_request_tick)
        self.ws.start()
        self.get_logger().info(f"gateway listening on ws://{self.ws.host}:{self.ws.port}")

    # -- websocket side (loop thread) --------------------------------------------------------
    def _on_connect(self) -> list[str]:
        with self.lock:
            return self.state.connect_burst()

    def _on_ws_message(self, text: str) -> None:
        try:
            msg = json.loads(text)
            kind, data = msg["type"], msg.get("data") or {}
        except (ValueError, KeyError, TypeError):
            return
        if kind == "text_command":
            self._text_command(str(data.get("text", "")))
        # ptt_start / ptt_stop / director: stubs in v1, ignored

    def _text_command(self, text: str) -> None:
        if not text.strip():
            return
        cmd = parse_command(text, self.default_budget)
        self.msg_n += 1
        self._emit(
            "transcript", {"id": f"u{self.msg_n}", "role": "user", "text": text, "source": "text"}
        )
        self._emit(
            "transcript",
            {"id": f"m{self.msg_n}", "role": "maverick", "text": cmd.reply, "source": "text"},
        )
        if cmd.request is not None:
            self.req_pub.publish(String(data=json.dumps(cmd.request)))
            if cmd.kind != "cancel":
                self.plan_id, self.plan_goal, self.plan_t0 = f"plan-{self.msg_n}", text, time.time()
                self.skill = None
                self._emit_agent_step()
        elif cmd.kind == "land":
            self.mission_pub.publish(String(data="land"))

    # -- helpers -----------------------------------------------------------------------------
    def _emit(self, type_: str, data, t: float | None = None) -> None:
        """Record in the connect state and stream to all consoles."""
        with self.lock:
            frame = self.state.put(type_, data, t)
            self.ws.broadcast(frame)

    def _emit_agent_step(self) -> None:
        if self.plan_id:
            self._emit(
                "agent_step",
                agent_step_data(
                    self.plan_id, self.plan_goal, self.skill, time.time() - self.plan_t0
                ),
            )

    # -- ROS side ----------------------------------------------------------------------------
    def _on_pose(self, msg: PoseStamped) -> None:
        if not self.pose_limit.ready():
            return
        t, r = pose_to_tr(msg)
        self._emit("pose", self.pose.update(stamp_to_sec(msg.header.stamp), t, r))

    def _on_info(self, msg: CameraInfo) -> None:
        self.hfov = hfov_deg(info_to_intrinsics(msg))

    def _on_rgb(self, msg: CompressedImage) -> None:
        if not self.cam_limit.ready():
            return
        self.seq += 1
        self.stamp_seq[stamp_key(msg.header.stamp)] = self.seq
        if len(self.stamp_seq) > 64:
            self.stamp_seq.pop(next(iter(self.stamp_seq)))
        data = camera_frame_data(self.seq, bytes(msg.data), self.hfov)
        with self.lock:  # camera frames are not part of the connect state: stream only
            self.ws.broadcast(envelope("camera_frame", data, stamp_to_sec(msg.header.stamp)))

    def _on_detections(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        seq = self.stamp_seq.get(int(d.get("stamp_ns", -1)))
        if seq is None:
            return
        with self.lock:
            self.ws.broadcast(envelope("detections", detections_data(d, seq), d.get("stamp")))

    def _on_voxels(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.got_voxels = True
        with self.lock:
            self.ws.broadcast(self.state.map_delta(d))

    def _voxel_request_tick(self) -> None:
        if not self.got_voxels:
            self.vox_req_pub.publish(Empty())

    def _wrap(self, key: str, field: str):
        def cb(msg: String) -> None:
            try:
                items = json.loads(msg.data)
            except ValueError:
                return
            if self.changes.changed(key, items):
                self._emit(key, {field: items})

        return cb

    def _on_path(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if d.get("status") in PATH_STATES and self.changes.changed("path", d):
            self._emit("path", d)

    def _on_progress(self, msg: String) -> None:
        try:
            d = check_skill_progress(json.loads(msg.data))
        except ValueError:
            return
        if d is None:
            return
        changed = self.skill is None or any(
            self.skill.get(k) != d.get(k) for k in ("skill", "state", "message")
        )
        self.skill = d
        self._emit("skill_progress", d)
        if changed:
            self._emit_agent_step()

    def _on_flight(self, msg: String) -> None:
        try:
            self.flight = json.loads(msg.data)
        except ValueError:
            pass

    def _vram_tick(self) -> None:
        try:
            out = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=memory.used,memory.total",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=2.0,
            ).stdout
        except (OSError, subprocess.SubprocessError):
            return
        v = parse_nvidia_smi(out)
        if v:
            self.vram = v

    def _status_tick(self) -> None:
        self._emit("status", status_data(self.flight, self.skill, self.vram, self.battery_pct))
        if self.plan_id and self.skill is not None and self.skill.get("state") == "running":
            self._emit_agent_step()  # elapsed_s ticks


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = GatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ws.stop()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
