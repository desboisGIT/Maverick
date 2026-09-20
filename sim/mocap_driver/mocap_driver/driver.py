"""Simulated motion-capture system (runs in the sim container, "in the room").

Reads maverick_0's pose from Gazebo transport (world network), throttles to ~30 Hz, adds mocap
noise (2 mm, 0.5 deg, +-5 ms sample-time jitter) and sends it over UDP to the bridge on the drone
network (maverick_bridge.mocap_protocol). Positions are relative to the spawn point, which is the
PX4 local origin: x, y from the spawn pose (PX4_GZ_MODEL_POSE), z from the first sample.

  python3 -m mocap_driver.driver [--host 10.231.1.3] [--port 5601] [--world textured] [--hz 30]
"""

from __future__ import annotations

import argparse
import os
import random
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "ros2_ws/src/maverick_bridge"))

from maverick_bridge import mocap_protocol as proto  # noqa: E402

from mocap_driver.noise import noisy_pose, sample_time_error  # noqa: E402


def spawn_xy_from_env(text: str) -> tuple[float, float]:
    """PX4_GZ_MODEL_POSE "x,y,z,roll,pitch,yaw" -> (x, y); empty = world origin."""
    if not text.strip():
        return 0.0, 0.0
    v = [float(c) for c in text.split(",")]
    return v[0], v[1]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("CAMERA_BRIDGE_ADDR", "10.231.1.3"))
    ap.add_argument("--port", type=int, default=proto.DEFAULT_PORT)
    ap.add_argument("--model", default="maverick_0")
    ap.add_argument("--world", default=os.environ.get("PX4_GZ_WORLD", "textured"))
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args(argv)

    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    sx, sy = spawn_xy_from_env(os.environ.get("PX4_GZ_MODEL_POSE", ""))
    rng = random.Random(args.seed)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lock = threading.Lock()
    state = {"seq": 0, "last": 0.0, "z0": None, "sent": 0}

    def on_pose(msg: Pose_V) -> None:
        now = time.time()
        with lock:
            if now - state["last"] < 1.0 / args.hz:
                return
            state["last"] = now
        for p in msg.pose:
            if p.name != args.model:
                continue
            if state["z0"] is None:
                state["z0"] = p.position.z
            pos = (p.position.x - sx, p.position.y - sy, p.position.z - state["z0"])
            q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            pos, q = noisy_pose(rng, pos, q)
            state["seq"] += 1
            sample = proto.MocapSample(state["seq"], now + sample_time_error(rng), pos, q)
            try:
                sock.sendto(proto.encode(sample), (args.host, args.port))
                state["sent"] += 1
            except OSError:
                pass
            return

    node = Node()
    topic = f"/world/{args.world}/dynamic_pose/info"
    while not node.subscribe(Pose_V, topic, on_pose):
        time.sleep(1.0)
    print(
        f"[mocap] {topic} -> {args.host}:{args.port} at {args.hz} Hz, origin ({sx}, {sy})",
        flush=True,
    )
    while True:
        time.sleep(10.0)
        print(f"[mocap] sent {state['sent']}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
