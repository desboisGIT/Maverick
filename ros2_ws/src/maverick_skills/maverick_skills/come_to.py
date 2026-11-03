"""come_to(person) and return: fly to a person's last known position (stopping short), or home.

come_to asks memory for the person's last known position, picks a free point `standoff_m` from
the person (1.5 m by default, never under `min_person_m` = 1.0 m) on the side the drone comes
from, flies there and turns to face them. In S4 people are static and have the appearance class
"person" (re-identification is S6), so "the person" is the most recently seen one.

return flies back to the take-off point (given by the node as `home_xy`) and optionally lands.
Pure Python (see navigation.py).
"""

from __future__ import annotations

import math

import numpy as np
from maverick_mapping.voxel_map import OCCUPIED

from .navigation import CompoundSkill, free_checker, wrap, yaw_to
from .skills import XY, Ctx, Out, SkillConfig

FACE_TOL_RAD = 0.2
FACE_TIMEOUT_S = 6.0
FIX_TOL_M = 0.1  # an arrival further than this from the ring radius is corrected


def clearance_at(grid_t, xy: XY, cap: float = 1.0) -> float:
    """Distance from xy to the nearest occupied cell of the map (capped at `cap`)."""
    grid, res, org = grid_t
    r = int(math.ceil(cap / res))
    ci, cj = int((xy[0] - org[0]) // res), int((xy[1] - org[1]) // res)
    i0, j0 = max(ci - r, 0), max(cj - r, 0)
    sub = grid[i0 : ci + r + 1, j0 : cj + r + 1] == OCCUPIED
    if not sub.any():
        return cap
    ii, jj = np.nonzero(sub)
    x = org[0] + (ii + i0 + 0.5) * res
    y = org[1] + (jj + j0 + 0.5) * res
    return float(min(cap, np.hypot(x - xy[0], y - xy[1]).min()))


def line_of_sight(grid_t, a: XY, b: XY, skip_end_m: float = 0.4) -> bool:
    """No occupied cell on the segment a-b (the last `skip_end_m` before b is ignored: the
    person is in the map as an obstacle themselves)."""
    grid, res, org = grid_t
    n = max(2, int(math.dist(a, b) / (res * 0.5)))
    for k in range(n + 1):
        t = k / n
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if math.dist(p, b) < skip_end_m:
            break
        i, j = int((p[0] - org[0]) // res), int((p[1] - org[1]) // res)
        if 0 <= i < grid.shape[0] and 0 <= j < grid.shape[1] and grid[i, j] == OCCUPIED:
            return False
    return True


def standoff_points(
    person: XY,
    robot: XY,
    free,
    standoff: float,
    min_dist: float = 1.0,
    grid_t=None,
    step_deg: float = 10.0,
) -> list[XY]:
    """Candidate stopping points on a ring around a person, best first. The ring is
    `standoff` +- 0.05 m (never under `min_dist` + 0.1), every point must be free in the planner's
    view (which keeps `min_dist` from people). With a map, points are ranked by clearance from
    obstacles (capped at 0.8 m), line of sight to the person, how close the radius is to
    `standoff` and how little of a detour from the robot's side it is."""
    base = math.atan2(robot[1] - person[1], robot[0] - person[0])
    n = int(round(360.0 / step_deg))
    radii = sorted({max(standoff + d, min_dist + 0.1) for d in (-0.05, 0.0, 0.05)})
    scored: list[tuple[float, XY]] = []
    for r in radii:
        for k in range(n):
            a = wrap(base + k * math.radians(step_deg))
            p = (person[0] + r * math.cos(a), person[1] + r * math.sin(a))
            if math.dist(p, person) < min_dist or not free(p):
                continue
            sc = -abs(r - standoff) * 2.0 - 0.8 * abs(wrap(a - base)) / math.pi
            if grid_t is not None:
                sc += 2.0 * min(clearance_at(grid_t, p), 0.8)
                sc += 1.5 if line_of_sight(grid_t, p, person) else 0.0
            scored.append((sc, p))
    scored.sort(key=lambda t: -t[0])
    return [p for _, p in scored]


class ComeToSkill(CompoundSkill):
    skill = "come_to"

    def __init__(self, cfg: SkillConfig, now: float, person: str, standoff_m: float = 1.25) -> None:
        super().__init__(cfg, now)
        self.person = person
        self.standoff = max(float(standoff_m), cfg.min_person_m)
        self.state = "ask"
        self.qid = f"c{int(now * 1000) % 10**9}"
        self.answer: dict | None = None
        self.t_query = now
        self.target: XY | None = None
        self.cands: list[XY] = []
        self.tries = 0
        self.pick: XY | None = None  # the stand-off point being flown to
        self.n_fix = 0
        self.t_face = 0.0
        self.xy_hold: XY = (0.0, 0.0)

    def on_answer(self, answer: dict) -> None:
        if answer.get("id") == self.qid:
            self.answer = answer

    def _next_candidate(self, ctx: Ctx) -> bool:
        if not self.cands or self.tries >= 5:
            return False
        p = self.cands.pop(0)
        # skip candidates right next to the one that just failed
        self.cands = [q for q in self.cands if math.dist(q, p) > 0.3]
        self.pick = p
        self.tries += 1
        self.start_leg(ctx, p, "the person", self.cfg.come_to_tolerance)
        self.ev(ctx, "standoff", p=[round(p[0], 2), round(p[1], 2)])
        return True

    def step(self, ctx: Ctx) -> Out:
        el = ctx.now - self.t0
        out = Out(progress=self.progress("running", -1.0, f"coming to the {self.person}"))
        if ctx.grid is None or ctx.robot_xy is None:
            out.progress["message"] = "waiting for the map"
            return out
        robot = ctx.robot_xy
        if self.state == "ask":
            out.query = {"id": self.qid, "query": "find", "label": "person", "confirmed": False}
            self.state, self.t_query = "wait", ctx.now
            return out
        if self.state == "wait":
            if self.answer is None:
                if ctx.now - self.t_query > self.cfg.query_timeout_s:
                    return self.finish_with("failed", "memory did not answer", ctx)
                return out
            people = [o for o in self.answer.get("objects", []) if o.get("confidence", 0) >= 0.5]
            if not people:
                return self.finish_with("failed", "I do not know where the person is", ctx)
            p = max(people, key=lambda o: (o.get("last_seen", 0.0), o.get("confidence", 0.0)))
            self.target = (float(p["position"][0]), float(p["position"][1]))
            self.cands = []
            # the ring first; when it is all blocked, a wider ring (up to +0.1 m, the referee's
            # 1.5 m bound is the limit), then the same with the relaxed squeeze radius
            relaxed = self.cfg.planner.tight_fallbacks[-1]
            # reachable points first; when the map cuts the drone off (it ends an explore wedged
            # beside furniture), any free point: the leg planner can use the drone's own track
            for tight, reach in ((None, True), (relaxed, True), (None, False), (relaxed, False)):
                free = free_checker(ctx, self.cfg, tight, reachable=reach)
                for extra in (0.0, 0.1):
                    pts = standoff_points(
                        self.target, robot, free, self.standoff + extra,
                        self.cfg.min_person_m, ctx.grid,
                    )  # fmt: skip
                    self.cands += [q for q in pts if q not in self.cands]
                if self.cands:
                    break
            self.pick = self.cands[0] if self.cands else None
            self.ev(ctx, "person", at=[round(v, 2) for v in self.target], n_cands=len(self.cands))
            if not self._next_candidate(ctx):
                return self.finish_with("failed", "there is no free place near the person", ctx)
            self.state = "fly"
        if self.state == "fly":
            if el > self.cfg.goto_timeout_s:
                return self.finish_with("failed", "could not reach the person in time", ctx)
            st = self.step_leg(ctx, out)
            if out.goal is not None and self.pick is not None:
                # the last metres to the stand-off point are flown slowly (PX4 overshoots)
                if math.dist(out.goal[:2], self.pick) < 1.2:
                    out.speed = self.cfg.speed_slow
            if st is None:
                return out
            if st == "failed":
                if not self._next_candidate(ctx):
                    return self.finish_with("failed", "I could not get near the person", ctx)
                return out
            d = math.dist(robot, self.target)
            want_d = math.dist(self.pick, self.target) if self.pick else self.standoff
            if abs(d - want_d) > FIX_TOL_M and self.n_fix < 3 and self.pick is not None:
                self.n_fix += 1  # closed loop: the arrival was off the ring, go there again
                self.start_leg(ctx, self.pick, "the person", self.cfg.come_to_tolerance / 2)
                self.ev(ctx, "fix", dist=round(d, 2), n=self.n_fix)
                return out
            self.state, self.t_face = "face", ctx.now
            self.xy_hold = robot
        if self.state == "face":
            want = yaw_to(robot, self.target)
            out.goal = (self.xy_hold[0], self.xy_hold[1], want)
            out.progress["message"] = "turning to the person"
            err = abs(wrap(ctx.robot_yaw - want)) if ctx.robot_yaw is not None else 0.0
            if err < FACE_TOL_RAD or ctx.now - self.t_face > FACE_TIMEOUT_S:
                d = math.dist(robot, self.target)
                if self.pick is not None and self.n_fix < 3:
                    if abs(d - math.dist(self.pick, self.target)) > FIX_TOL_M + 0.05:
                        self.n_fix += 1  # drifted while turning: go back to the ring point
                        self.start_leg(ctx, self.pick, "the person", self.cfg.come_to_tolerance / 2)
                        self.ev(ctx, "fix", dist=round(d, 2), n=self.n_fix, after="face")
                        self.state = "fly"
                        return out
                res = {"person_xy": [round(v, 3) for v in self.target], "distance_m": round(d, 2)}
                return self.finish_with("succeeded", f"I am {d:.1f} m from the person", ctx, res)
        return out


class ReturnSkill(CompoundSkill):
    skill = "return"

    def __init__(self, cfg: SkillConfig, now: float, home_xy: XY, land: bool = False) -> None:
        super().__init__(cfg, now)
        self.home = (float(home_xy[0]), float(home_xy[1]))
        self.land = land
        self.state = "start"

    def step(self, ctx: Ctx) -> Out:
        out = Out(progress=self.progress("running", -1.0, "flying back to the take-off point"))
        if ctx.grid is None or ctx.robot_xy is None:
            out.progress["message"] = "waiting for the map"
            return out
        if self.state == "start":
            self.start_leg(ctx, self.home, "the take-off point", self.cfg.return_tolerance)
            self.state = "fly"
        if ctx.now - self.t0 > self.cfg.goto_timeout_s:
            return self.finish_with("failed", "could not get back in time", ctx)
        st = self.step_leg(ctx, out)
        if st is None:
            return out
        if st == "failed":
            return self.finish_with("failed", "I could not get back to the take-off point", ctx)
        res = {"home_xy": [round(self.home[0], 3), round(self.home[1], 3)]}
        done = self.finish_with("succeeded", "back at the take-off point", ctx, res)
        if self.land:
            done.mission = "land"
        return done
