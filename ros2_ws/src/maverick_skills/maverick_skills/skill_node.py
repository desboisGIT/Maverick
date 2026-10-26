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
  in  /maverick/memory/detections|rooms|objects  (search frames; known rooms and people)
  out /maverick/flight/mission  std_msgs/String        "land" (return with land)

Every request goes through the executive (executive.py) first: invalid, unsafe or concurrent
requests are rejected with a failed progress message ("rejected": true, "code", "message"). All
progress messages of an accepted request carry its "request_id" (the request's "id", else
"ex-<n>"); a skill's structured outcome is under "result".
  out /maverick/flight/goal     std_msgs/String JSON   goto goals, PX4 NED (skills.goal_message)
  out /maverick/memory/query    std_msgs/String JSON
  out /maverick/skill/progress  std_msgs/String JSON   gateway `skill_progress` data, latched
  out /maverick/skill/path      std_msgs/String JSON   gateway `path` data, latched

Parameters: altitude (1.2), speed (0.8), tick_hz (2.0), require_hover (true),
default_budget_s (360).
"""

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from maverick_mapping import topics as T
from maverick_mapping.drone import drone_config
from maverick_mapping.ros_conv import camera_euler, occupancy_to_grid, pose_to_tr
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String

from .executive import Executive, WorldView
from .guard import ClearanceGuard, GuardConfig, VelocityEstimate
from .skills import Ctx, Out, SkillConfig, goal_message, make_skill

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
RELIABLE = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)


class SkillNode(Node):
    def __init__(self) -> None:
        super().__init__("skill_node")
        p = self.declare_parameter
        drone = drone_config(str(p("drone", "").value) or None)  # "" -> MAVERICK_DRONE
        self.cfg = SkillConfig(
            altitude=float(p("altitude", 1.2).value),
            speed=float(p("speed", 0.8).value),
            default_budget_s=float(p("default_budget_s", 360.0).value),
            goal_tolerance=drone.goal_tolerance,
            arrive_radius=drone.arrive_radius,
            arrive_radius_slow=drone.arrive_radius_slow,
            planner=drone.planner(),
        )
        self.require_hover = bool(p("require_hover", True).value)
        # camera_mount.json x
        self.cam_fwd = float(p("camera_forward_m", drone.camera_forward_m).value)
        self.event_log = str(p("event_log", "/tmp/skill_events.jsonl").value)
        self.skill = None
        self.robot_xy: tuple[float, float] | None = None
        self.trail: list[tuple[float, float]] = []
        self.robot_yaw: float | None = None
        self.grid = None
        self.grid_time = -1e9
        self.phase = ""
        self.t0 = time.monotonic()
        self.executive = Executive()
        self.frames: list[dict] = []  # detection payloads since the last tick (active skill only)
        self.room_types: frozenset[str] = frozenset()
        self.people: list[tuple[float, float]] = []
        self.home_xy: tuple[float, float] | None = None
        self.active_rid: str | None = None
        # reactive clearance guard (guard.py): runs at 10 Hz on the mapping node's clearance report
        self.guard_on = bool(p("guard", True).value)
        self.guard = ClearanceGuard(GuardConfig())
        self.vel = VelocityEstimate()
        self.clear: dict | None = None
        self.clear_rx = -1e9
        self.guard_hold_until = -1e9
        self.guard_goal: tuple[float, float, float, float] | None = None  # x, y, yaw, speed
        self.guard_slow_sent = -1e9
        self.guard_slowed = False
        self.guard_nudges: list[float] = []
        self.keepouts: list[tuple[float, float, float]] = []  # obstacle x, y, monotonic time
        self.guard_pending: tuple[float, float, float, float] | None = None
        self.guard_stats = {"stop": 0, "nudge": 0, "slow": 0}

        self.create_subscription(String, T.SKILL_REQUEST, self._on_request, RELIABLE)
        self.create_subscription(OccupancyGrid, T.MAP_GRID, self._on_grid, LATCHED)
        self.create_subscription(PoseStamped, T.CAM_POSE, self._on_pose, qos_profile_sensor_data)
        self.create_subscription(String, T.FLIGHT_STATUS, self._on_status, 10)
        self.create_subscription(String, T.MEM_ANSWER, self._on_answer, RELIABLE)
        self.create_subscription(String, T.MEM_DETECTIONS, self._on_detections, RELIABLE)
        self.create_subscription(String, T.MEM_ROOMS, self._on_rooms, LATCHED)
        self.create_subscription(String, T.MEM_OBJECTS, self._on_objects, LATCHED)
        self.create_subscription(
            String, T.MAP_CLEARANCE, self._on_clearance, qos_profile_sensor_data
        )
        self.mission_pub = self.create_publisher(String, T.FLIGHT_MISSION, 10)
        self.goal_pub = self.create_publisher(String, T.FLIGHT_GOAL, 10)
        self.query_pub = self.create_publisher(String, T.MEM_QUERY, RELIABLE)
        self.progress_pub = self.create_publisher(String, T.SKILL_PROGRESS, LATCHED)
        self.path_pub = self.create_publisher(String, T.SKILL_PATH, LATCHED)
        self.create_timer(1.0 / float(p("tick_hz", 2.0).value), self._tick)
        self.create_timer(0.1, self._guard_tick)
        self.get_logger().info("skill_node up")

    def _now(self) -> float:
        return time.monotonic() - self.t0

    def _grid_with_keepouts(self):
        """The planner's grid with a disk around each recent guard stop's obstacle marked occupied,
        so that the replan after a stop does not lead back to the same spot."""
        now = time.monotonic()
        self.keepouts = [k for k in self.keepouts if now - k[2] < self.guard.cfg.keepout_s]
        if self.grid is None or not self.keepouts:
            return self.grid
        g, res, org = self.grid
        g = g.copy()
        r = self.guard.cfg.keepout_m
        n = int(math.ceil(r / res))
        for kx, ky, _t in self.keepouts:
            ci, cj = int((kx - org[0]) / res), int((ky - org[1]) / res)
            for di in range(-n, n + 1):
                for dj in range(-n, n + 1):
                    i, j = ci + di, cj + dj
                    if (
                        0 <= i < g.shape[0]
                        and 0 <= j < g.shape[1]
                        and math.hypot(di, dj) * res <= r
                    ):
                        g[i, j] = 100
        return g, res, org

    def _ctx(self) -> Ctx:
        return Ctx(
            self._now(),
            self.robot_xy,
            self._grid_with_keepouts(),
            self.grid_time,
            robot_yaw=self.robot_yaw,
            trail=self.trail,
        )

    # -- inputs ------------------------------------------------------------------------------
    def _on_grid(self, msg: OccupancyGrid) -> None:
        g, res, org = occupancy_to_grid(msg)
        if g.size:
            self.grid, self.grid_time = (g, res, org), self._now()

    def _on_pose(self, msg: PoseStamped) -> None:
        t, r = pose_to_tr(msg)
        yaw = camera_euler(r)[0]
        # the pose is the camera's; the controlled point is the body centre, behind the lens
        self.robot_xy = (
            float(t[0]) - self.cam_fwd * math.cos(yaw),
            float(t[1]) - self.cam_fwd * math.sin(yaw),
        )
        self.robot_yaw = yaw
        self.vel.add(time.monotonic(), self.robot_xy)
        if not self.trail or math.dist(self.trail[-1], self.robot_xy) > 0.1:
            self.trail.append(self.robot_xy)
            del self.trail[:-6000]

    def _on_clearance(self, msg: String) -> None:
        try:
            self.clear = json.loads(msg.data)
            self.clear_rx = time.monotonic()
        except ValueError:
            pass

    def _on_status(self, msg: String) -> None:
        try:
            self.phase = str(json.loads(msg.data).get("phase", ""))
        except ValueError:
            return
        if self.phase == "hover" and self.home_xy is None and self.robot_xy is not None:
            self.home_xy = self.robot_xy  # first hover = above the take-off point

    def _on_detections(self, msg: String) -> None:
        if self.skill is None or self.skill.finished:
            return
        try:
            self.frames.append(json.loads(msg.data))
        except ValueError:
            return
        del self.frames[:-200]

    def _on_rooms(self, msg: String) -> None:
        try:
            rooms = json.loads(msg.data)
            types = {r.get("type") for r in rooms} | {r.get("secondary") for r in rooms}
            self.room_types = frozenset(t for t in types if t)
        except (ValueError, AttributeError):
            pass

    def _on_objects(self, msg: String) -> None:
        try:
            objs = json.loads(msg.data)
            self.people = [
                (float(o["position"][0]), float(o["position"][1]))
                for o in objs
                if o.get("label") == "person" and float(o.get("confidence", 0)) >= 0.5
            ]
        except (ValueError, KeyError, TypeError):
            pass

    def _view(self) -> WorldView:
        return WorldView(self.room_types, list(self.people), self.robot_xy, self.home_xy)

    def _on_answer(self, msg: String) -> None:
        if self.skill is not None:
            try:
                self.skill.on_answer(json.loads(msg.data))
            except ValueError:
                pass

    def _on_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
        except ValueError:
            req = None
        self._sync_executive()
        dec = self.executive.submit(req, self._view())
        if not dec.accepted:
            self.get_logger().warn(f"rejected {msg.data[:120]!r}: {dec.code}: {dec.reason}")
            self._publish_progress(dec.progress())
            return
        req = dec.request
        if req["skill"] == "return":
            req = {**req, "home_xy": list(self.home_xy)}
        try:
            ctx = self._ctx()
            old = self.skill
            new = make_skill(req, self.cfg, ctx.now)
        except (ValueError, TypeError, AttributeError) as exc:
            self.get_logger().warn(f"bad skill request {msg.data[:120]!r}: {exc}")
            self._publish_progress(
                dec.progress() | {"message": f"bad request: {exc}", "code": "malformed"}
            )
            return
        if old is not None and not old.finished:
            self._emit(old.cancel(ctx), ctx)
            self._drain_events(old)
            self.executive.finished()
        self.frames.clear()
        self.skill = new
        if new is None:
            self.get_logger().info("skill cancelled")
        else:
            self.active_rid = dec.request_id
            self.executive.started(dec)
            self.get_logger().info(f"skill {req} ({dec.request_id})")

    def _sync_executive(self) -> None:
        """The executive is busy only while a skill really runs."""
        if self.skill is None or self.skill.finished:
            self.executive.finished()

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
        ctx.frames, self.frames = self.frames, []
        ctx.people = list(self.people)
        out = s.step(ctx)
        if out.goal is not None and s.skill in ("goto", "search", "come_to", "return"):
            why = self.executive.goal_violation(out.goal[:2], self._view())
            if why is not None:  # never fly closer than 1 m to a person
                self.get_logger().warn(f"{s.skill}: refused goal: {why}")
                out = s.finish("failed", f"refused: {why}", ctx)
        self._emit(out, ctx)
        self._drain_events(s)
        if s.finished:
            self.executive.finished()

    def _guard_tick(self) -> None:
        s = self.skill
        if not self.guard_on or s is None or s.finished or self.robot_xy is None:
            return
        if self.require_hover and self.phase != "hover":
            return
        now = time.monotonic()
        if now < self.guard_hold_until:
            return
        if self.guard_pending is not None:  # the skill's goal that arrived during a hold
            x, y, yaw, speed = self.guard_pending
            self.guard_pending = None
            self.goal_pub.publish(
                String(data=json.dumps(goal_message(x, y, self.cfg.altitude, yaw, speed)))
            )
            return
        if self.clear is None or now - self.clear_rx > self.guard.cfg.max_age_s:
            return
        gx, gy, gyaw, gspeed = self.guard_goal or (None, None, 0.0, self.cfg.speed)
        to_goal = None if gx is None else (gx - self.robot_xy[0], gy - self.robot_xy[1])
        vel = self.vel.velocity()
        heading = self.guard.heading(vel, to_goal)
        v_toward = 0.0
        if vel is not None and heading is not None:
            v_toward = vel[0] * math.cos(heading) + vel[1] * math.sin(heading)
        dec = self.guard.evaluate(list(self.clear["sectors"]), heading, v_toward)
        if dec.action == "ok":
            if self.guard_slowed and gx is not None:  # clear again: back to the skill's speed
                self.guard_slowed = False
                self.goal_pub.publish(
                    String(data=json.dumps(goal_message(gx, gy, self.cfg.altitude, gyaw, gspeed)))
                )
            return
        z = self.cfg.altitude
        rx, ry = self.robot_xy
        if dec.action == "stop":
            self.guard_stats["stop"] += 1
            self.guard_hold_until = now + self.guard.cfg.hold_s
            self.goal_pub.publish(String(data=json.dumps(goal_message(rx, ry, z, 0.0, 1.0))))
            info = {
                "d_fwd": round(dec.d_fwd, 3),
                "d_min": round(dec.d_min, 3),
                "heading": round(dec.heading, 2),
                "v": round(v_toward, 2),
            }
            if dec.bearing is not None:
                ox = rx + dec.d_fwd * math.cos(dec.bearing)
                oy = ry + dec.d_fwd * math.sin(dec.bearing)
                self.keepouts.append((ox, oy, now))
                info["obstacle"] = [round(ox, 2), round(oy, 2)]
            s.on_guard(self._ctx(), info)
            self._drain_events(s)
            self.get_logger().warn(f"guard stop {info}")
        elif dec.action == "nudge":
            # a nudge that changes nothing (the drone is wedged or down) is not repeated forever
            self.guard_nudges = [t for t in self.guard_nudges if now - t < 10.0]
            if len(self.guard_nudges) >= 4:
                return
            self.guard_nudges.append(now)
            self.guard_stats["nudge"] += 1
            self.guard_hold_until = now + 0.8
            ax, ay = dec.away
            self.goal_pub.publish(
                String(
                    data=json.dumps(
                        goal_message(rx + ax * dec.step, ry + ay * dec.step, z, 0.0, 0.3)
                    )
                )
            )
            s.ev(self._ctx(), "guard_nudge", d_min=round(dec.d_min, 3), step=round(dec.step, 3))
            self._drain_events(s)
        elif dec.action == "slow" and now - self.guard_slow_sent > 0.5 and gx is not None:
            self.guard_stats["slow"] += 1
            self.guard_slow_sent = now
            self.guard_slowed = True
            cap = min(dec.speed_cap, gspeed)
            self.goal_pub.publish(String(data=json.dumps(goal_message(gx, gy, z, gyaw, cap))))

    def _drain_events(self, s) -> None:
        if not s.events:
            return
        try:
            with open(self.event_log, "a") as f:
                for e in s.events:
                    f.write(json.dumps({"skill": s.skill, **e}) + "\n")
        except OSError:
            pass
        s.events.clear()

    def _emit(self, out: Out, ctx: Ctx) -> None:
        z = self.cfg.altitude
        if out.goal is not None:
            x, y, yaw = out.goal
            speed = out.speed or self.cfg.speed
            self.guard_goal = (x, y, yaw, speed)
            self.guard_slowed = False
            if time.monotonic() < self.guard_hold_until:
                self.guard_pending = (x, y, yaw, speed)  # sent when the hold ends
            else:
                self.goal_pub.publish(String(data=json.dumps(goal_message(x, y, z, yaw, speed))))
        elif out.hold and ctx.robot_xy is not None:
            self.guard_goal = None
            x, y = ctx.robot_xy
            self.goal_pub.publish(
                String(data=json.dumps(goal_message(x, y, z, 0.0, self.cfg.speed)))
            )
        if out.query is not None:
            self.query_pub.publish(String(data=json.dumps(out.query)))
        if out.mission is not None:
            self.mission_pub.publish(String(data=out.mission))
        if out.path is not None:
            self.path_pub.publish(String(data=json.dumps(out.path)))
        if out.progress is not None:
            self._publish_progress(out.progress)

    def _publish_progress(self, d: dict) -> None:
        if self.active_rid and "request_id" not in d:
            d = {**d, "request_id": self.active_rid}
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
