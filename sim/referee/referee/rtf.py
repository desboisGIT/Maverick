"""Gazebo real-time factor. Runs INSIDE the referee container.

  GZ_PARTITION=<sim hostname>:<user> python3 -m referee.rtf [--world textured]

Prints one JSON line per second: wall-clock time and the world's real_time_factor.
"""

from __future__ import annotations

import argparse
import json
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="textured")
    args = ap.parse_args()

    from gz.msgs10.world_stats_pb2 import WorldStatistics
    from gz.transport13 import Node

    last = [0.0]

    def on_stats(msg: WorldStatistics) -> None:
        now = time.time()
        if now - last[0] < 1.0:
            return
        last[0] = now
        print(json.dumps({"t": round(now, 3), "rtf": round(msg.real_time_factor, 4)}), flush=True)

    node = Node()
    topic = f"/world/{args.world}/stats"
    if not node.subscribe(WorldStatistics, topic, on_stats):
        print(f"cannot subscribe to {topic}", file=sys.stderr)
        return 1
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
