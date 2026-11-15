"""The lockstep loop: the real drone-stack pure modules on simulated time.

One `KinStack` owns a scene, a vehicle model and the drone stack as it runs in the host container:

    flight client   FlightMission + SafetyGate (real), 20 Hz, setpoint stream to the vehicle
    mapping node    VoxelMap.integrate_depth (real) at 5 Hz, clearance report after every frame,
                    2D grid at 1 Hz
    memory node     SemanticDetector + SceneGraph (real), 5 Hz frames, rooms/objects 1 Hz
                    (goto, search)
    skill node      Executive + skills (real) at 2 Hz; the reactive guard at 10 Hz. The node's own
                    glue (`_tick`, `_guard_tick`, `_emit`) is re-implemented in `SkillRunner`;
                    tests/test_parity.py pins the source of the node's methods so that a change
                    there cannot go unnoticed here.

Time is exact: `dt` = 25 ms steps; the clocks of the real system are multiples of it.
"""

from __future__ import annotations

import json
import math

import numpy as np
from maverick_flight.mission import (
    ARMING_STATE_ARMED,
    NAV_STATE_OFFBOARD,
    FlightMission,
    MissionConfig,
    Observation,
    Phase,
)
from maverick_flight.safety_gate import GateConfig, SafetyGate
from maverick_mapping.clearance import occupied_window, sector_clearance
from maverick_mapping.drone import drone_config
from maverick_mapping.voxel_map import VoxelMap
from maverick_skills.executive import Executive, WorldView
from maverick_skills.guard import ClearanceGuard, GuardConfig, VelocityEstimate
from maverick_skills.skills import Ctx, Out, SkillConfig, goal_message, make_skill

from .camera import DepthCamera, Mount, camera_pose
from .scene import KScene
from .vehicle import KinParams, Vehicle

DT = 0.025  # s, lockstep step: 40 Hz
MAP_BAND = (0.35, 1.6)  # mapping_node band
CLEAR_RMAX = 0.8
FLIGHT_ALT = 2.0  # flight client hover altitude in the S3 launch file
SKILL_ALT = 1.2  # skill node altitude parameter


class SkillRunner:
    """The skill node without ROS: `skill_node.SkillNode._tick/_guard_tick/_emit` and friends."""

    def __init__(self, drone, send_goal, send_mission, send_query, event_sink=None):
        self.cfg = SkillConfig(
            altitude=SKILL_ALT,
            speed=0.8,
            default_budget_s=360.0,
            goal_tolerance=drone.goal_tolerance,
            arrive_radius=drone.arrive_radius,
            arrive_radius_slow=drone.arrive_radius_slow,
            planner=drone.planner(),
        )
        self.cam_fwd = drone.camera_forward_m
        self.send_goal, self.send_mission, self.send_query = send_goal, send_mission, send_query
        self.events: list[dict] = event_sink if event_sink is not None else []
        self.skill = None
        self.robot_xy: tuple[float, float] | None = None
        self.trail: list[tuple[float, float]] = []
        self.robot_yaw: float | None = None
        self.grid = None
        self.grid_time = -1e9
        self.executive = Executive()
        self.frames: list[dict] = []
        self.room_types: frozenset[str] = frozenset()
        self.people: list[tuple[float, float]] = []
        self.home_xy = None
        self.active_rid = None
        self.progress: list[dict] = []
        self.guard = ClearanceGuard(GuardConfig())
        self.vel = VelocityEstimate()
        self.clear = None
        self.clear_rx = -1e9
        self.guard_hold_until = -1e9
        self.guard_goal = None
        self.guard_slow_sent = -1e9
        self.guard_slowed = False
        self.guard_nudges: list[float] = []
        self.guard_pending = None
        self.guard_stats = {"stop": 0, "nudge": 0, "slow": 0}
        self.now = 0.0

    # -- inputs ------------------------------------------------------------------------------
    def on_grid(self, now, grid) -> None:
        if grid[0].size:
            self.grid, self.grid_time = grid, now

    def on_pose(self, now, cam_t, cam_yaw) -> None:
        self.robot_xy = (
            float(cam_t[0]) - self.cam_fwd * math.cos(cam_yaw),
            float(cam_t[1]) - self.cam_fwd * math.sin(cam_yaw),
        )
        self.robot_yaw = cam_yaw
        self.vel.add(now, self.robot_xy)
        if not self.trail or math.dist(self.trail[-1], self.robot_xy) > 0.1:
            self.trail.append(self.robot_xy)
            del self.trail[:-6000]

    def on_clearance(self, now, msg) -> None:
        self.clear, self.clear_rx = msg, now

    def on_detections(self, payload) -> None:
        if self.skill is None or self.skill.finished:
            return
        self.frames.append(payload)
        del self.frames[:-200]

    def on_rooms(self, rooms) -> None:
        types = {r.get("type") for r in rooms} | {r.get("secondary") for r in rooms}
        self.room_types = frozenset(t for t in types if t)

    def on_objects(self, objs) -> None:
        self.people = [
            (float(o["position"][0]), float(o["position"][1]))
            for o in objs
            if o.get("label") == "person" and float(o.get("confidence", 0)) >= 0.5
        ]

    def on_answer(self, answer) -> None:
        if self.skill is not None:
            self.skill.on_answer(answer)

    def on_first_hover(self) -> None:
        if self.home_xy is None and self.robot_xy is not None:
            self.home_xy = self.robot_xy

    def _view(self) -> WorldView:
        return WorldView(self.room_types, list(self.people), self.robot_xy, self.home_xy)

    def _ctx(self, now) -> Ctx:
        return Ctx(
            now,
            self.robot_xy,
            self.grid,
            self.grid_time,
            robot_yaw=self.robot_yaw,
            trail=self.trail,
        )

    def request(self, now: float, req: dict) -> dict | None:
        """skill_node._on_request. Returns the rejection progress message, if any."""
        self.now = now
        if self.skill is None or self.skill.finished:
            self.executive.finished()
        dec = self.executive.submit(req, self._view())
        if not dec.accepted:
            p = dec.progress()
            self.progress.append(p)
            return p
        req = dec.request
        if req["skill"] == "return":
            req = {**req, "home_xy": list(self.home_xy)}
        ctx = self._ctx(now)
        old = self.skill
        new = make_skill(req, self.cfg, ctx.now)
        if old is not None and not old.finished:
            self._emit(old.cancel(ctx), ctx, now)
            self._drain(old)
            self.executive.finished()
        self.frames.clear()
        self.skill = new
        if new is not None:
            self.active_rid = dec.request_id
            self.executive.started(dec)
        return None

    # -- loops -------------------------------------------------------------------------------
    def tick(self, now: float) -> None:
        """skill_node._tick (2 Hz)."""
        self.now = now
        s = self.skill
        if s is None or s.finished:
            return
        ctx = self._ctx(now)
        ctx.frames, self.frames = self.frames, []
        ctx.people = list(self.people)
        out = s.step(ctx)
        if out.goal is not None and s.skill in ("goto", "search", "come_to", "return"):
            why = self.executive.goal_violation(out.goal[:2], self._view())
            if why is not None:
                out = s.finish("failed", f"refused: {why}", ctx)
        self._emit(out, ctx, now)
        self._drain(s)
        if s.finished:
            self.executive.finished()

    def guard_tick(self, now: float) -> None:
        """skill_node._guard_tick (10 Hz)."""
        self.now = now
        s = self.skill
        if s is None or s.finished or self.robot_xy is None:
            return
        if now < self.guard_hold_until:
            return
        if self.guard_pending is not None:
            x, y, yaw, speed = self.guard_pending
            self.guard_pending = None
            self.send_goal(goal_message(x, y, self.cfg.altitude, yaw, speed))
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
            if self.guard_slowed and gx is not None:
                self.guard_slowed = False
                self.send_goal(goal_message(gx, gy, self.cfg.altitude, gyaw, gspeed))
            return
        z = self.cfg.altitude
        rx, ry = self.robot_xy
        if dec.action == "stop":
            self.guard_stats["stop"] += 1
            self.guard_hold_until = now + self.guard.cfg.hold_s
            self.send_goal(goal_message(rx, ry, z, 0.0, 1.0))
            info = {
                "d_fwd": round(dec.d_fwd, 3),
                "d_min": round(dec.d_min, 3),
                "heading": round(dec.heading, 2),
                "v": round(v_toward, 2),
            }
            s.on_guard(self._ctx(now), info)
            self._drain(s)
        elif dec.action == "nudge":
            self.guard_nudges = [t for t in self.guard_nudges if now - t < 10.0]
            if len(self.guard_nudges) >= 4:
                return
            self.guard_nudges.append(now)
            self.guard_stats["nudge"] += 1
            self.guard_hold_until = now + 0.8
            ax, ay = dec.away
            self.send_goal(goal_message(rx + ax * dec.step, ry + ay * dec.step, z, 0.0, 0.3))
            s.ev(self._ctx(now), "guard_nudge", d_min=round(dec.d_min, 3), step=round(dec.step, 3))
            self._drain(s)
        elif dec.action == "slow" and now - self.guard_slow_sent > 0.5 and gx is not None:
            self.guard_stats["slow"] += 1
            self.guard_slow_sent = now
            self.guard_slowed = True
            cap = min(dec.speed_cap, gspeed)
            self.send_goal(goal_message(gx, gy, z, gyaw, cap))

    def _drain(self, s) -> None:
        if not s.events:
            return
        for e in s.events:
            self.events.append({"skill": s.skill, **e})
        s.events.clear()

    def _emit(self, out: Out, ctx: Ctx, now: float) -> None:
        z = self.cfg.altitude
        if out.goal is not None:
            x, y, yaw = out.goal
            speed = out.speed or self.cfg.speed
            self.guard_goal = (x, y, yaw, speed)
            self.guard_slowed = False
            if now < self.guard_hold_until:
                self.guard_pending = (x, y, yaw, speed)
            else:
                self.send_goal(goal_message(x, y, z, yaw, speed))
        elif out.hold and ctx.robot_xy is not None:
            self.guard_goal = None
            x, y = ctx.robot_xy
            self.send_goal(goal_message(x, y, z, 0.0, self.cfg.speed))
        if out.query is not None:
            self.send_query(out.query)
        if out.mission is not None:
            self.send_mission(out.mission)
        if out.progress is not None:
            d = out.progress
            if self.active_rid and "request_id" not in d:
                d = {**d, "request_id": self.active_rid}
            self.progress.append(d)


class KinStack:
    """Scene + vehicle + the drone stack, advanced in lockstep."""

    def __init__(
        self,
        scene: KScene,
        params: KinParams | None = None,
        seed: int = 0,
        run: int = 0,
        with_memory: bool = False,
        drone: str = "small",
    ) -> None:
        self.scene, self.params = scene, params or KinParams()
        self.seed, self.run = seed, run
        rng = np.random.default_rng(np.random.SeedSequence([seed, run, 7]))
        self.rng = rng
        self.drone = drone_config(drone)
        self.mount = Mount.load(self.drone.mount_dir)
        self.cam = DepthCamera(scene, self.mount, self.params.map_stride)
        self.vehicle = Vehicle(
            scene,
            self.params.vehicle,
            (0.0, 0.0, self.params.vehicle.start_z),
            scene.spawn[2],
            rng,
        )
        self.vmap = VoxelMap(0.1, max_range=5.0)
        self.t = 0.0
        self.n = 0
        self.events: list[dict] = []
        self.runner = SkillRunner(
            self.drone, self._on_goal, self._on_mission, self._on_query, self.events
        )
        # flight client, started in HOVER at the flight altitude (the take-off is not simulated)
        gate = SafetyGate(GateConfig((-15.0, -15.0, -2.4), (15.0, 15.0, 0.3), 1.0, 1.0))
        self.mission = FlightMission(MissionConfig(altitude=FLIGHT_ALT, hover_seconds=None), gate)
        m = self.mission
        m.gate.origin = (0.0, 0.0, 0.0)
        m.setpoint = (0.0, 0.0, -FLIGHT_ALT)
        m.hold = m.goal = m.setpoint
        m.phase = Phase.HOVER
        m._t0 = m._phase_t0 = 0.0
        m._last_t = None
        m.armed_ever = True
        self.sp_ned = m.setpoint
        self.yaw_ned = math.nan
        # memory (goto / search)
        self.memory = None
        if with_memory:
            from .memory import MemoryShim

            self.memory = MemoryShim(self.drone, seed, run, self.runner)
            self.memory.set_intrinsics(self.cam.intr)
        # phase offsets of the periodic nodes (deterministic per run)
        ph = rng.integers(0, 20, size=3)
        self.skill_phase, self.guard_phase, self.grid_phase = int(ph[0]), int(ph[1]) % 4, int(ph[2])
        self._frame_q: list[tuple] = []  # (deliver_t, frame dict)
        self._frame_i = 0
        self._last_taken = -1e9
        self.grid_pub = None
        self.track: list[tuple[float, float, float, float]] = []  # t, x, y, z (truth, map frame)
        self.est_track: list[tuple[float, float, float, float]] = []
        self.snapshots: list[tuple[float, tuple]] = []
        self.integrations = 0
        self.stats = {"render_s": 0.0, "integrate_s": 0.0, "plan_s": 0.0, "memory_s": 0.0}

    # -- flight client ------------------------------------------------------------------------
    def _on_goal(self, d: dict) -> None:
        self.mission.on_goal(
            str(d.get("mode", "goto")),
            (float(d["x"]), float(d["y"]), float(d["z"])),
            float(d["speed"]) if d.get("speed") is not None else None,
            float(d["yaw"]) if d.get("yaw") is not None else None,
        )

    def _on_mission(self, text: str) -> None:
        if text.strip().lower() == "land":
            self.mission.request_land()

    def _on_query(self, q: dict) -> None:
        if self.memory is not None:
            self.runner.on_answer(self.memory.answer(q))

    def _flight_step(self) -> None:
        v = self.vehicle
        e = v.est_pos
        obs = Observation(
            t=self.t,
            position=(float(e[1]), float(e[0]), float(-e[2])),
            arming_state=ARMING_STATE_ARMED,
            nav_state=NAV_STATE_OFFBOARD,
            landed=False,
            telemetry_age_s=0.0,
            ev_fused=True,
        )
        act = self.mission.step(obs)
        if act.setpoint is not None:
            self.sp_ned = act.setpoint
            self.yaw_ned = self.mission.yaw

    # -- sensors ------------------------------------------------------------------------------
    def _camera_frame(self) -> None:
        """Every 0.1 s: pose of the frame; frames the nodes will use are rendered and queued."""
        v, p = self.vehicle, self.params.vehicle
        est_p, est_yaw = v.estimate_at(self.t)
        cam_est_t, cam_est_r = camera_pose(est_p, est_yaw, self.mount)
        self._frame_i += 1
        # the 5 Hz rate limiters of the nodes (wall clock) against 10 Hz frames with jitter: some
        # frames arrive just under 0.2 s after the last one taken and are skipped (real: ~4.6 / s)
        arrive = self.t + p.map_latency_s + float(self.rng.normal(0.0, p.frame_jitter_s))
        take = arrive - self._last_taken >= 0.2
        if take:
            self._last_taken = arrive
        frame = {
            "t": self.t,
            "cam_est_t": cam_est_t,
            "cam_est_r": cam_est_r,
            "est_yaw": est_yaw,
            "depth": None,
            "sem": None,
        }
        if take:
            import time

            r0 = time.perf_counter()
            # the image shows the world as it was `pose_lead_s` before its paired pose
            tp, _, ty = v._at(self.t - p.pose_lead_s)
            cam_t, cam_r = camera_pose(tp, ty, self.mount)  # TRUE pose renders the image
            frame["depth"] = self.cam.depth_for_mapping(cam_t, cam_r)
            if self.memory is not None:
                frame["sem"] = self.cam.semantic_and_depth(cam_t, cam_r)
            self.stats["render_s"] += time.perf_counter() - r0
        self._frame_q.append((self.t + p.map_latency_s, frame))

    def _deliver_frames(self) -> None:
        import time

        while self._frame_q and self._frame_q[0][0] <= self.t + 1e-9:
            _, f = self._frame_q.pop(0)
            ct, cr = f["cam_est_t"], f["cam_est_r"]
            self.runner.on_pose(self.t, ct, f["est_yaw"])
            if f["depth"] is None:
                continue
            r0 = time.perf_counter()
            self.vmap.integrate_depth(
                f["depth"], self.cam.intr, ct, cr, stride=self.params.map_stride
            )
            self.integrations += 1
            self.stats["integrate_s"] += time.perf_counter() - r0
            self._publish_clearance(ct, f["est_yaw"])
            if self.memory is not None and f["sem"] is not None:
                r0 = time.perf_counter()
                self.memory.process(f["sem"], ct, cr, self.t)
                self.stats["memory_s"] += time.perf_counter() - r0

    def _publish_clearance(self, cam_t, yaw) -> None:
        c = (
            float(cam_t[0]) - self.drone.camera_forward_m * math.cos(yaw),
            float(cam_t[1]) - self.drone.camera_forward_m * math.sin(yaw),
        )
        occ, org = occupied_window(self.vmap, c, CLEAR_RMAX, MAP_BAND)
        sectors = sector_clearance(occ, self.vmap.res, org, c, CLEAR_RMAX)
        self.runner.on_clearance(self.t, {"xy": c, "sectors": sectors})

    def _publish_grid(self) -> None:
        if self.vmap.frames == 0:
            return
        self.grid_pub = self.vmap.project_2d(*MAP_BAND)
        self.runner.on_grid(self.t, self.grid_pub)
        if self.memory is not None:
            self.memory.on_grid(self.grid_pub)

    # -- loop ---------------------------------------------------------------------------------
    def step(self) -> None:
        import time

        self.n += 1
        t = self.n * DT
        v = self.vehicle
        if self.n % 2 == 0:
            self._flight_step()
        sp = np.array([self.sp_ned[1], self.sp_ned[0], -self.sp_ned[2]])
        yaw_sp = None if not math.isfinite(self.yaw_ned) else math.pi / 2 - self.yaw_ned
        v.step(DT, sp, yaw_sp)
        self.t = t
        if self.n % 2 == 0:
            self.track.append((t, float(v.pos[0]), float(v.pos[1]), float(v.pos[2])))
            self.est_track.append(
                (t, float(v.est_pos[0]), float(v.est_pos[1]), float(v.est_pos[2]))
            )
        if self.n % 4 == 0:
            self._camera_frame()
        self._deliver_frames()
        if (self.n + self.guard_phase) % 4 == 0:
            self.runner.guard_tick(t)
        if (self.n + self.skill_phase) % 20 == 0:
            r0 = time.perf_counter()
            self.runner.tick(t)
            self.stats["plan_s"] += time.perf_counter() - r0
        if (self.n + self.grid_phase) % 40 == 0:
            self._publish_grid()
            if self.memory is not None:
                r0 = time.perf_counter()
                self.memory.update_rooms(t)
                self.stats["memory_s"] += time.perf_counter() - r0

    def run_for(self, seconds: float, until=None) -> None:
        end = self.t + seconds
        while self.t < end - 1e-9:
            self.step()
            if until is not None and until():
                break

    def settle(self, seconds: float = 8.0) -> None:
        """Hover before the first skill (the first map frames arrive; home is the hover point)."""
        self.run_for(seconds)
        self.runner.on_first_hover()

    def skill_done(self) -> bool:
        s = self.runner.skill
        return s is None or s.finished

    def json_events(self) -> str:
        return "\n".join(json.dumps(e) for e in self.events)
