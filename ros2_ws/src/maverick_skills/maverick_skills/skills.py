"""Skill state machines (explore, goto_room): pure Python + maverick_mapping, no ROS imports.

A skill is stepped at ~2 Hz with a context (clock, robot xy, latest 2D grid) and returns what to
do: a goto goal for the flight client, a path / progress event for the console, or a memory
query. Skills never send setpoints; goals are one waypoint at a time and the flight client's
safety gate does the rest. This is the S4 executive's precursor: keep it simple and replaceable.

Frames: everything here is the map frame (ENU, x east, y north, z up). `goal_message` converts to
the flight client's NED goal at the very end.
"""

from __future__ import annotations

import copy
import dataclasses
import math
from dataclasses import dataclass, field

import numpy as np
from maverick_mapping.frames import enu_to_ned, yaw_enu_to_ned
from maverick_mapping.frontiers import detect_frontiers
from maverick_mapping.planner import (
    ExplorationPlanner,
    PlannerConfig,
    PlanningGrid,
    astar,
    shorten_ex,
)

XY = tuple[float, float]


@dataclass
class SkillConfig:
    altitude: float = 1.2
    speed_slow: float = 0.4  # speed on a waypoint that crosses a squeeze (door)
    arrive_radius_slow: float = 0.08  # tighter arrival there: no corner cutting at a door frame
    speed: float = 0.8  # goto speed handed to the flight client (it caps at its own limit)
    # waypoint counts as reached; corners are cut by up to this much, while the path clears
    # obstacles by only robot_radius + margin (0.2 clipped door frames)
    arrive_radius: float = 0.1
    stuck_s: float = 12.0  # no progress of `min_progress` towards the waypoint for this long
    min_progress: float = 0.15
    replan_s: float = 3.0
    look_s: float = 1.5  # pause at a viewpoint, facing the frontier, before judging it
    goal_tolerance: float = 0.25  # goto_room: done when this close to the goal
    goto_timeout_s: float = 180.0
    query_timeout_s: float = 3.0
    path_fail_s: float = 10.0  # goto_room: give up when no path exists for this long
    default_budget_s: float = 360.0
    scan_every_m: float = 4.0  # 360 degree yaw scan at the start and after this much travel
    scan_step_rad: float = math.radians(60.0)
    scan_step_timeout_s: float = 3.0  # a yaw step that never completes (no yaw control) is skipped
    # when nothing reachable is left but frontiers remain (blacklisted viewpoints, frontiers that
    # a noisy first look could not resolve), clear the blacklists and sweep again: S3 tuning seeds
    # 20-59 ended with 0.90 coverage and 4 frontiers left at 86 s of a 360 s budget
    retry_passes: int = 2
    retry_min_budget_s: float = 40.0
    planner: PlannerConfig = field(default_factory=PlannerConfig)


@dataclass
class Ctx:
    now: float
    robot_xy: XY | None
    grid: tuple[np.ndarray, float, XY] | None  # (int8 grid[ix, iy], resolution, origin)
    grid_time: float = -1e9  # `now` at which that grid arrived
    robot_yaw: float | None = (
        None  # camera heading, rad ccw from +x (None: no yaw feedback, no scans)
    )
    trail: list | None = None  # own flown positions (map frame), for the last-resort path


@dataclass
class Out:
    goal: tuple[float, float, float] | None = None  # (x, y, yaw) map frame; altitude = cfg
    speed: float | None = None  # goto speed for `goal` (None: the node's default)
    hold: bool = False  # hold at the current robot position
    path: dict | None = None  # gateway `path` data
    progress: dict | None = None  # gateway `skill_progress` data
    query: dict | None = None  # memory query to publish
    finished: bool = False


def goal_message(x: float, y: float, z_up: float, yaw_enu: float, speed: float) -> dict:
    """Flight-client goal JSON (PX4 local NED) for a map-frame (ENU) point."""
    n, e, d = enu_to_ned(x, y, z_up)
    return {
        "mode": "goto",
        "x": round(n, 3),
        "y": round(e, 3),
        "z": round(d, 3),
        "speed": speed,
        "yaw": round(yaw_enu_to_ned(yaw_enu), 3),  # ignored until the flight client supports it
    }


class WaypointFollower:
    """Walk a waypoint list: one goto goal at a time, advance on arrival, detect a stall."""

    def __init__(
        self,
        arrive_radius: float = 0.2,
        stuck_s: float = 12.0,
        min_progress: float = 0.15,
        arrive_radius_slow: float | None = None,
    ):
        self.arrive_radius, self.stuck_s, self.min_progress = arrive_radius, stuck_s, min_progress
        self.arrive_radius_slow = arrive_radius_slow or arrive_radius
        self.slow: list[bool] = []
        self.wps: list[XY] = []
        self.i = 0
        self._new = False
        self._best = math.inf
        self._t_best = 0.0

    @property
    def target(self) -> XY | None:
        return self.wps[self.i] if self.i < len(self.wps) else None

    @property
    def target_slow(self) -> bool:
        return self.i < len(self.slow) and self.slow[self.i]

    def set(self, wps: list[XY], now: float, slow: list[bool] | None = None) -> None:
        prev = self.target
        self.wps, self.i = [tuple(w) for w in wps], 0
        self.slow = list(slow) if slow else [False] * len(self.wps)
        t = self.target
        same = t is not None and prev is not None and math.dist(t, prev) <= 0.05
        if t is None:
            self._new = False
        elif not same:
            self._new = True
        # else: a goal still waiting to be sent (update() advanced to this target) stays pending;
        # clearing it here left the drone waiting at the previous waypoint (seeds 10-13 goto_room)
        if not same:  # a replan that keeps the target must not hide a stall from the stuck timer
            self._best, self._t_best = math.inf, now

    def take_new_target(self) -> XY | None:
        if self._new:
            self._new = False
            return self.target
        return None

    def update(self, robot: XY, now: float) -> str:
        """'moving', 'arrived' (all waypoints reached) or 'stuck'."""
        while self.i < len(self.wps) and math.dist(robot, self.wps[self.i]) < (
            self.arrive_radius_slow if self.target_slow else self.arrive_radius
        ):
            self.i += 1
            self._new = True
            self._best, self._t_best = math.inf, now
        if self.i >= len(self.wps):
            self._new = False
            return "arrived"
        d = math.dist(robot, self.wps[self.i])
        if d < self._best - self.min_progress:
            self._best, self._t_best = d, now
        elif now - self._t_best > self.stuck_s:
            return "stuck"
        return "moving"


def _path_event(pid: str, status: str, robot: XY, wps: list[XY], goal: XY | None, z: float) -> dict:
    pts = [[round(robot[0], 3), round(robot[1], 3), z]] if wps else []
    pts += [[round(x, 3), round(y, 3), z] for x, y in wps]
    d: dict = {"path_id": pid, "status": status, "waypoints": pts}
    if goal is not None:
        d["goal"] = [round(goal[0], 3), round(goal[1], 3), z]
    return d


class _Base:
    skill = ""  # gateway skill name

    def __init__(self, cfg: SkillConfig, now: float) -> None:
        self.cfg = cfg
        self.t0 = now
        self.follow = WaypointFollower(
            cfg.arrive_radius, cfg.stuck_s, cfg.min_progress, cfg.arrive_radius_slow
        )
        self.finished = False
        self.result = ""
        self.n_paths = 0
        self.last_path: dict | None = None
        self.events: list[dict] = []  # decision log, drained by the node (debugging, referee)

    def ev(self, ctx: Ctx, kind: str, **kw) -> None:
        r = ctx.robot_xy or (0.0, 0.0)
        xy = [round(r[0], 2), round(r[1], 2)]
        self.events.append({"t": round(ctx.now - self.t0, 2), "ev": kind, "xy": xy, **kw})

    def progress(self, state: str, progress: float, message: str, eta: float | None = None) -> dict:
        d = {
            "skill": self.skill,
            "state": state,
            "progress": round(progress, 3),
            "message": message,
        }
        if eta is not None:
            d["eta_s"] = round(max(eta, 0.0), 1)
        return d

    def _new_path_id(self) -> str:
        self.n_paths += 1
        return f"{self.skill}-{self.n_paths}"

    def finish(self, state: str, message: str, ctx: Ctx, hold: bool = True) -> Out:
        self.finished, self.result = True, state
        self.ev(ctx, "finish", state=state, message=message)
        robot = ctx.robot_xy or (0.0, 0.0)
        if state == "succeeded":
            path = _path_event(f"{self.skill}-done", "done", robot, [], None, self.cfg.altitude)
        elif self.last_path is not None:
            path = dict(self.last_path, status="failed" if state == "failed" else "done")
            if state == "cancelled":
                path = _path_event(f"{self.skill}-x", "done", robot, [], None, self.cfg.altitude)
        else:
            path = None
        prog = self.progress(state, 1.0 if state == "succeeded" else -1.0, message)
        return Out(hold=hold, path=path, progress=prog, finished=True)

    def cancel(self, ctx: Ctx) -> Out:
        return self.finish("cancelled", "cancelled", ctx)

    def on_answer(self, answer: dict) -> None:
        pass


class ExploreSkill(_Base):
    """Frontier exploration: plan, fly the waypoints, look, repeat, until nothing is left."""

    skill = "explore"

    def __init__(self, cfg: SkillConfig, now: float, budget_s: float | None = None) -> None:
        super().__init__(cfg, now)
        self.budget = float(budget_s if budget_s else cfg.default_budget_s)
        pc = copy.copy(cfg.planner)
        pc.altitude = cfg.altitude
        self.planner = ExplorationPlanner(pc)
        self.state = "plan"
        self.goal = None  # current Frontier
        self.last_plan = -1e9
        self.t_look = 0.0
        self.n_goals = 0
        self.n_fronts = 0
        self.odo = 0.0
        self.last_xy: XY | None = None
        self.last_scan_odo = -1e9
        self.scan_i = 0
        self.scan_yaw0 = 0.0
        self.scan_xy: XY = (0.0, 0.0)
        self.scan_t = 0.0
        self.retries = 0
        self.looked_vp: list[XY] = []  # viewpoints already looked at from where the robot stood

    def _scan(self, ctx: Ctx, out: Out) -> bool:
        """Spin in place in `scan_step_rad` steps with camera yaw feedback. True when done."""
        c = self.cfg
        n = int(round(2 * math.pi / c.scan_step_rad))
        target = self.scan_yaw0 + (self.scan_i + 1) * c.scan_step_rad
        err = math.atan2(math.sin(ctx.robot_yaw - target), math.cos(ctx.robot_yaw - target))
        if abs(err) < 0.25 or ctx.now - self.scan_t > c.scan_step_timeout_s:
            self.scan_i += 1
            self.scan_t = ctx.now
            if self.scan_i >= n:
                self.last_scan_odo = self.odo
                return True
            target += c.scan_step_rad
        out.goal = (self.scan_xy[0], self.scan_xy[1], target)
        return False

    def _start_scan(self, ctx: Ctx) -> None:
        self.state = "scan"
        self.scan_i, self.scan_t = 0, ctx.now
        self.scan_yaw0, self.scan_xy = ctx.robot_yaw, ctx.robot_xy

    def _run(self, ctx: Ctx):
        grid, res, org = ctx.grid
        return self.planner.plan(grid, res, org, ctx.robot_xy)

    def _adopt(self, plan, ctx: Ctx, out: Out) -> None:
        self.goal = plan.goal
        self.last_plan = ctx.now
        wps = [(x, y) for x, y, _ in plan.waypoints]
        self.follow.set(wps, ctx.now, plan.slow)
        self.state = "fly"
        self.n_fronts = len(plan.frontiers)
        self.n_goals += 1
        self.ev(
            ctx,
            "plan",
            centroid=[round(v, 2) for v in plan.goal.centroid],
            vp=[round(v, 2) for v in plan.goal.viewpoint_xy],
            size=plan.goal.size,
            n_fronts=len(plan.frontiers),
            wps=[[round(x, 2), round(y, 2)] for x, y, _ in plan.waypoints],
        )
        self.last_path = _path_event(
            self._new_path_id(),
            "executing",
            ctx.robot_xy,
            wps,
            plan.goal.viewpoint_xy,
            self.cfg.altitude,
        )
        out.path = self.last_path

    def step(self, ctx: Ctx) -> Out:
        el = ctx.now - self.t0
        if el >= self.budget:
            return self.finish("succeeded", "time budget used up", ctx)
        out = Out()
        out.progress = self.progress(
            "running",
            min(el / self.budget, 0.99),
            f"exploring: {self.n_goals} goals, {self.n_fronts} frontiers",
            self.budget - el,
        )
        if ctx.grid is None or ctx.robot_xy is None:
            out.progress["message"] = "waiting for the map"
            return out
        robot = ctx.robot_xy
        if self.last_xy is not None:
            self.odo += math.dist(robot, self.last_xy)
        self.last_xy = robot
        can_scan = ctx.robot_yaw is not None and self.cfg.scan_every_m > 0
        if self.state == "scan":
            if self._scan(ctx, out):
                self.state = "plan"
            else:
                out.progress["message"] = "scanning around"
                return out
        for _ in range(4):  # a few immediate replans (viewpoint already reached, ...)
            if self.state == "fly":
                st = self.follow.update(robot, ctx.now)
                if st == "stuck":
                    self.ev(ctx, "stuck", target=self.follow.target)
                    self.planner.mark_viewpoint_failed(self.goal.viewpoint_xy)
                    self.state = "plan"
                elif st == "arrived":
                    self.state, self.t_look = "look", ctx.now
                    cx, cy = self.goal.centroid
                    out.goal = (robot[0], robot[1], math.atan2(cy - robot[1], cx - robot[0]))
                    break
                elif ctx.now - self.last_plan >= self.cfg.replan_s:
                    self.state = "replan"
                else:
                    break
            if self.state == "look":
                if ctx.now - self.t_look < self.cfg.look_s or ctx.grid_time < self.t_look + 0.5:
                    break
                g, res, org = ctx.grid
                for f in detect_frontiers(g, res, org, self.cfg.planner.min_cluster):
                    x = org[0] + (f.cells[:, 0] + 0.5) * res
                    y = org[1] + (f.cells[:, 1] + 0.5) * res
                    if float(np.min(np.hypot(x - robot[0], y - robot[1]))) < 0.9:
                        self.ev(ctx, "look_failed")
                        self.planner.mark_failed(robot)  # looked, the frontier did not resolve
                        break
                self.state = "plan"
            if (
                self.state == "plan"
                and can_scan
                and (self.odo - self.last_scan_odo >= self.cfg.scan_every_m)
            ):
                self._start_scan(ctx)
                self._scan(ctx, out)
                out.progress["message"] = "scanning around"
                return out
            if self.state in ("plan", "replan"):
                plan = self._run(ctx)
                if (
                    plan.status == "done"
                    and plan.reason == "no reachable frontier"
                    and self.retries < self.cfg.retry_passes
                    and self.budget - el > self.cfg.retry_min_budget_s
                ):
                    self.retries += 1
                    self.ev(ctx, "retry", n=self.retries, n_fronts=len(plan.frontiers))
                    self.planner.failed.clear()
                    self.planner.failed_vp.clear()
                    self.planner.goal_centroid = None
                    self.last_scan_odo = -1e9  # look around first: the map may have changed
                    self.state = "plan"
                    continue
                if plan.status == "done":
                    self.ev(
                        ctx,
                        "plan_done",
                        reason=plan.reason,
                        n_fronts=len(plan.frontiers),
                        failed=len(self.planner.failed),
                        failed_vp=len(self.planner.failed_vp),
                    )
                    why = {
                        "no frontier left": "explored everything",
                        "no reachable frontier": "explored everything reachable",
                    }.get(plan.reason, plan.reason)
                    return self.finish("succeeded", why, ctx, hold=False)
                if self.state == "replan":
                    same = self.goal is not None and (
                        math.dist(plan.goal.centroid, self.goal.centroid) < 1.2
                    )
                    self.last_plan = ctx.now
                    self.n_fronts = len(plan.frontiers)
                    if same:  # keep flying the current path
                        self.state = "fly"
                        break
                if math.dist(plan.goal.viewpoint_xy, robot) < 0.35:
                    self.ev(ctx, "vp_here", vp=list(plan.goal.viewpoint_xy))
                    vp = plan.goal.viewpoint_xy
                    if all(math.dist(vp, v) > 0.5 for v in self.looked_vp):
                        # the frontier is still there although the robot stands at its viewpoint:
                        # it was flying past, camera not facing it. Face it first, then judge
                        # (before, the viewpoint was blacklisted unseen: tuning seeds 41, 45)
                        self.looked_vp.append(vp)
                        self.goal = plan.goal
                        self.state, self.t_look = "look", ctx.now
                        cx, cy = plan.goal.centroid
                        out.goal = (robot[0], robot[1], math.atan2(cy - robot[1], cx - robot[0]))
                        break
                    self.planner.mark_viewpoint_failed(vp)
                    self.state = "plan"
                    continue
                self._adopt(plan, ctx, out)
                break
        t = self.follow.take_new_target()
        if t is not None:
            out.goal = (t[0], t[1], math.atan2(t[1] - robot[1], t[0] - robot[0]))
            out.speed = self.cfg.speed_slow if self.follow.target_slow else None
        return out


def snap_free(pg: PlanningGrid, xy: XY, radius_m: float) -> tuple[int, int] | None:
    """The cell nearest to xy (within radius_m) that is not blocked in the planning grid."""
    c = pg.to_cell(xy)
    r = int(math.ceil(radius_m / pg.res))
    best, best_d = None, math.inf
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            d = math.hypot(di, dj)
            if d <= r and d < best_d and pg.free_at(c[0] + di, c[1] + dj):
                best, best_d = (c[0] + di, c[1] + dj), d
    return best


class GotoRoomSkill(_Base):
    """'Go to the kitchen': ask memory for a goal in the room, A* over the known free space."""

    skill = "goto"

    def __init__(self, cfg: SkillConfig, now: float, room_type: str) -> None:
        super().__init__(cfg, now)
        self.room_type = room_type
        self.state = "query"
        self.qid = f"q{int(now * 1000) % 10**9}"
        self.t_query = now
        self.answer: dict | None = None
        self.goal_xy: XY | None = None
        self.t_nopath: float | None = None
        self.d0 = 0.0
        self.last_plan = -1e9
        self.n_stuck = 0

    def on_answer(self, answer: dict) -> None:
        if answer.get("id") == self.qid:
            self.answer = answer

    def _plan_path(self, ctx: Ctx) -> tuple[list[XY], list[bool]] | None:
        grid, res, org = ctx.grid
        cells = pg = b = None
        tiers = [
            (tr, None) for tr in (self.cfg.planner.tight_radius, *self.cfg.planner.tight_fallbacks)
        ]
        tiers.append((self.cfg.planner.tight_fallbacks[-1], ctx.trail))  # + the flown track
        for tr, trail in tiers:
            pc = dataclasses.replace(self.cfg.planner, tight_radius=tr)
            pg = PlanningGrid.build(grid, res, org, pc, ctx.robot_xy, trail)
            a = pg.to_cell(ctx.robot_xy)
            b = snap_free(pg, self.goal_xy, 0.6)
            if b is None or not pg.free_at(*a):
                continue
            cells = astar(pg, a, b)
            if cells is not None:
                if tr != self.cfg.planner.tight_radius or trail:
                    self.ev(ctx, "tight_fallback", tight=tr, trail=trail is not None)
                break
        if cells is None:
            return None
        pts, slow = shorten_ex(pg, cells, ctx.robot_xy, self.cfg.planner.squeeze_pad_m)
        if len(pts) < 2:
            return [pg.to_xy(b)], [False]
        return pts[1:], slow[1:]

    def step(self, ctx: Ctx) -> Out:
        el = ctx.now - self.t0
        label = self.room_type
        if el > self.cfg.goto_timeout_s:
            return self.finish("failed", f"could not reach the {label} in time", ctx)
        out = Out(progress=self.progress("running", -1.0, f"going to the {label}"))
        if ctx.grid is None or ctx.robot_xy is None:
            out.progress["message"] = "waiting for the map"
            return out
        robot = ctx.robot_xy
        if self.state == "query":
            out.query = {
                "id": self.qid,
                "query": "goal_for_type",
                "room_type": self.room_type,
                "start": [robot[0], robot[1]],
            }
            self.state, self.t_query = "wait", ctx.now
            return out
        if self.state == "wait":
            if self.answer is None:
                if ctx.now - self.t_query > self.cfg.query_timeout_s:
                    return self.finish("failed", "memory did not answer", ctx)
                return out
            if not self.answer.get("ok"):
                why = self.answer.get("reason") or f"no {label} known yet"
                return self.finish("failed", f"I do not know where the {label} is: {why}", ctx)
            self.goal_xy = (float(self.answer["goal"][0]), float(self.answer["goal"][1]))
            self.ev(ctx, "goal", goal=list(self.goal_xy), answer=self.answer)
            self.d0 = max(math.dist(robot, self.goal_xy), 0.5)
            self.state = "plan"
        dist = math.dist(robot, self.goal_xy)
        out.progress["progress"] = round(max(0.0, min(1.0 - dist / self.d0, 0.99)), 3)
        if dist < self.cfg.goal_tolerance:
            return self.finish("succeeded", f"arrived in the {label}", ctx, hold=False)
        if self.state == "fly":
            st = self.follow.update(robot, ctx.now)
            if st == "arrived":
                if dist < self.cfg.goal_tolerance * 2:
                    return self.finish("succeeded", f"arrived in the {label}", ctx, hold=False)
                self.state = "plan"
            elif st == "stuck":
                self.ev(ctx, "stuck", target=self.follow.target, n_stuck=self.n_stuck + 1)
                self.n_stuck += 1
                if self.n_stuck >= 3:
                    return self.finish("failed", f"stuck on the way to the {label}", ctx)
                self.state = "plan"
            elif ctx.now - self.last_plan >= self.cfg.replan_s:
                self.state = "plan"
        if self.state == "plan":
            self.last_plan = ctx.now
            res_p = self._plan_path(ctx)
            wps, slow = res_p if res_p is not None else (None, None)
            if wps is None:
                if self.t_nopath is None:
                    self.t_nopath = ctx.now
                if ctx.now - self.t_nopath > self.cfg.path_fail_s:
                    return self.finish("failed", f"no path to the {label}", ctx)
                out.progress["message"] = f"looking for a path to the {label}"
                return out
            self.t_nopath = None
            first = self.n_paths == 0 or self.follow.target is None
            moved = first or math.dist(wps[0], self.follow.target) > 0.05
            self.follow.set(wps, ctx.now, slow)
            self.state = "fly"
            if moved:
                self.ev(ctx, "path", wps=[[round(x, 2), round(y, 2)] for x, y in wps])
                self.last_path = _path_event(
                    self._new_path_id(), "executing", robot, wps, self.goal_xy, self.cfg.altitude
                )
                out.path = self.last_path
        t = self.follow.take_new_target()
        if t is not None:
            out.goal = (t[0], t[1], math.atan2(t[1] - robot[1], t[0] - robot[0]))
            out.speed = self.cfg.speed_slow if self.follow.target_slow else None
            self.ev(
                ctx,
                "send",
                wp=[round(t[0], 2), round(t[1], 2)],
                i=self.follow.i,
                n=len(self.follow.wps),
                slow=self.follow.target_slow,
            )
        return out


def make_skill(req: dict, cfg: SkillConfig, now: float) -> _Base | None:
    """Build a skill from a /maverick/skill/request dict; None for cancel; ValueError if invalid."""
    kind = req.get("skill")
    if kind == "cancel":
        return None
    if kind == "explore":
        b = req.get("budget_s")
        return ExploreSkill(cfg, now, float(b) if b is not None else None)
    if kind == "goto_room":
        rt = str(req.get("room_type", "")).strip().lower()
        if not rt:
            raise ValueError("goto_room needs room_type")
        return GotoRoomSkill(cfg, now, rt)
    raise ValueError(f"unknown skill {kind!r}")
