"""Publish a list of goals to /maverick/flight/goal (test tool for F6; run in the host container).

ros2 run maverick_flight inject_goals '[{"mode":"goto","x":9,"y":0,"z":-2}, ...]' --gap 1.0
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("goals", help="JSON list of goal objects")
    parser.add_argument("--gap", type=float, default=1.0, help="seconds between goals")
    args = parser.parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])
    goals = json.loads(args.goals)
    rclpy.init(args=argv)
    node = rclpy.create_node("goal_injector")
    pub = node.create_publisher(String, "/maverick/flight/goal", 10)
    deadline = time.monotonic() + 10.0
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    for g in goals:
        msg = String()
        msg.data = json.dumps(g)
        pub.publish(msg)
        print(json.dumps({"t": round(time.time(), 3), "event": "goal_sent", "goal": g}), flush=True)
        end = time.monotonic() + args.gap
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
