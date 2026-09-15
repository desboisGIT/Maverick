"""ROS 2 node `skill_node`: runs one skill at a time (explore, goto_room) on the flight client.

The S4 executive's precursor. Commands arrive on a topic as JSON; the node steps the active
skill at `tick_hz`, sends one goto goal at a time to the flight client, and reports progress.

Topics
  in  /maverick/skill/request   std_msgs/String JSON  {"skill": "explore", "budget_s": 360}
                                | {"skill": "goto_room", "room_type": "kitchen"}
                                | {"skill": "cancel"}
  in  /maverick/map/grid        nav_msgs/OccupancyGrid  (latched; the planner's map)
  in  /maverick/camera/pose     geometry_msgs/PoseStamped  (drone position = camera position)
  in  /maverick/flight/status   std_msgs/String JSON   (skills start once phase == "hover")
  in  /maverick/memory/answer   std_msgs/String JSON   (goal_for_type replies)
  out /maverick/flight/goal     std_msgs/String JSON   goto goals, PX4 NED (skills.goal_message)
  out /maverick/memory/query    std_msgs/String JSON
  out /maverick/skill/progress  std_msgs/String JSON   gateway `skill_progress` data, latched
  out /maverick/skill/path      std_msgs/String JSON   gateway `path` data, latched

Parameters: altitude (1.2), speed (0.8), tick_hz (2.0), require_hover (true),
default_budget_s (360).
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from maverick_mapping import topics as T
from maverick_mapping.ros_conv import camera_euler, occupancy_to_grid, pose_to_tr
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from .skills import Ctx, Out, SkillConfig, goal_message, make_skill

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
RELIABLE = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)


class SkillNode(Node):
    def __init__(self) -> None:
        super().__init__("skill_node")
        p = self.declare_parameter
        self.cfg = SkillConfig(
            altitude=float(p("altitude", 1.2).value),
            speed=float(p("speed", 0.8).value),
            default_budget_s=float(p("default_budget_s", 360.0).value),
        )
        self.require_hover = bool(p("require_hover", True).value)
        self.skill = None
        self.robot_xy: tuple[float, float] | None = None
        self.robot_yaw: float | None = None
        self.grid = None
        self.grid_time = -1e9
        self.phase = ""
        self.t0 = time.monotonic()

        self.create_subscription(String, T.SKILL_REQUEST, self._on_request, RELIABLE)
        self.create_subscription(OccupancyGrid, T.MAP_GRID, self._on_grid, LATCHED)
        self.create_subscription(PoseStamped, T.CAM_POSE, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(String, T.FLIGHT_STATUS, self._on_status, 10)
        self.create_subscription(String, T.MEM_ANSWER, self._on_answer, RELIABLE)
        self.goal_pub = self.create_publisher(String, T.FLIGHT_GOAL, 10)
        self.query_pub = self.create_publisher(String, T.MEM_QUERY, RELIABLE)
        self.progress_pub = self.create_publisher(String, T.SKILL_PROGRESS, LATCHED)
        self.path_pub = self.create_publisher(String, T.SKILL_PATH, LATCHED)
        self.create_timer(1.0 / float(p("tick_hz", 2.0).value), self._tick)
        self.get_logger().info("skill_node up")

    def _now(self) -> float:
        return time.monotonic() - self.t0

    def _ctx(self) -> Ctx:
        return Ctx(self._now(), self.robot_xy, self.grid, self.grid_time, self.robot_yaw)

    # -- inputs ------------------------------------------------------------------------------
    def _on_grid(self, msg: OccupancyGrid) -> None:
        g, res, org = occupancy_to_grid(msg)
        if g.size:
            self.grid, self.grid_time = (g, res, org), self._now()

    def _on_pose(self, msg: PoseStamped) -> None:
        t, r = pose_to_tr(msg)
        self.robot_xy = (float(t[0]), float(t[1]))
        self.robot_yaw = camera_euler(r)[0]

    def _on_status(self, msg: String) -> None:
        try:
            self.phase = str(json.loads(msg.data).get("phase", ""))
        except ValueError:
            pass

    def _on_answer(self, msg: String) -> None:
        if self.skill is not None:
            try:
                self.skill.on_answer(json.loads(msg.data))
            except ValueError:
                pass

    def _on_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
            ctx = self._ctx()
            old = self.skill
            new = make_skill(req, self.cfg, ctx.now)
        except (ValueError, TypeError, AttributeError) as exc:
            self.get_logger().warn(f"bad skill request {msg.data[:120]!r}: {exc}")
            self._publish_progress(
                {
                    "skill": "explore",
                    "state": "failed",
                    "progress": -1,
                    "message": f"bad request: {exc}",
                }
            )
            return
        if old is not None and not old.finished:
            self._emit(old.cancel(ctx), ctx)
        self.skill = new
        if new is None:
            self.get_logger().info("skill cancelled")
        else:
            self.get_logger().info(f"skill {req}")

    # -- loop --------------------------------------------------------------------------------
    def _tick(self) -> None:
        s = self.skill
        if s is None or s.finished:
            return
        ctx = self._ctx()
        if self.require_hover and self.phase != "hover":
            self._publish_progress(
                s.progress(
                    "running", -1.0, f"waiting for hover (flight phase: {self.phase or '?'})"
                )
            )
            return
        self._emit(s.step(ctx), ctx)

    def _emit(self, out: Out, ctx: Ctx) -> None:
        z = self.cfg.altitude
        if out.goal is not None:
            x, y, yaw = out.goal
            self.goal_pub.publish(
                String(data=json.dumps(goal_message(x, y, z, yaw, self.cfg.speed)))
            )
        elif out.hold and ctx.robot_xy is not None:
            x, y = ctx.robot_xy
            self.goal_pub.publish(
                String(data=json.dumps(goal_message(x, y, z, 0.0, self.cfg.speed)))
            )
        if out.query is not None:
            self.query_pub.publish(String(data=json.dumps(out.query)))
        if out.path is not None:
            self.path_pub.publish(String(data=json.dumps(out.path)))
        if out.progress is not None:
            self._publish_progress(out.progress)

    def _publish_progress(self, d: dict) -> None:
        self.progress_pub.publish(String(data=json.dumps(d)))


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = SkillNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
