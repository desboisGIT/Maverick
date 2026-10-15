"""Ground truth from Gazebo transport. Runs INSIDE the referee container.

  GZ_PARTITION=<sim hostname>:<user> python3 -m referee.gt [--model maverick_small_0] [--world X]

Prints one JSON line per pose update (throttled to ~20 Hz): wall-clock arrival time and the
model position in the Gazebo world frame (ENU, metres). The referee is the only component that
reads this; host and bridge are not on the `world` network.

Why GZ_PARTITION: gz-transport partitions discovery by "<hostname>:<user>" and containers have
different hostnames, so without a shared partition the referee sees no topics at all.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

MODEL_INSTANCE = (
    "maverick_0"
    if os.environ.get("MAVERICK_DRONE", "small").lower() == "x500"
    else "maverick_small_0"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default=MODEL_INSTANCE)
    parser.add_argument("--world", default="textured")
    parser.add_argument("--hz", type=float, default=20.0)
    args = parser.parse_args()

    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    last = [0.0]
    lock = threading.Lock()

    def on_pose(msg: Pose_V) -> None:
        now = time.time()
        with lock:
            if now - last[0] < 1.0 / args.hz:
                return
            last[0] = now
        for pose in msg.pose:
            if pose.name == args.model:
                p = pose.position
                print(
                    json.dumps(
                        {
                            "t": round(now, 3),
                            "x": round(p.x, 4),
                            "y": round(p.y, 4),
                            "z": round(p.z, 4),
                        }
                    ),
                    flush=True,
                )
                return

    node = Node()
    topic = f"/world/{args.world}/dynamic_pose/info"
    if not node.subscribe(Pose_V, topic, on_pose):
        print(f"cannot subscribe to {topic}", file=sys.stderr)
        return 1
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
