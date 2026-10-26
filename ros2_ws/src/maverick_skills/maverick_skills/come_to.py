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

from .navigation import CompoundSkill, free_checker, wrap, yaw_to
from .skills import XY, Ctx, Out, SkillConfig

FACE_TOL_RAD = 0.2
FACE_TIMEOUT_S = 6.0


def standoff_points(
    person: XY,
    robot: XY,
    free,
    standoff: float,
    min_dist: float = 1.0,
    max_extra: float = 0.1,
    step_deg: float = 10.0,
) -> list[XY]:
    """Candidate stopping points around a person, best first: free, at least `standoff` away
    (the ring grows when the first is blocked), preferring the side the robot is on.
    Never closer than `min_dist`."""
    base = math.atan2(robot[1] - person[1], robot[0] - person[0])
    n = int(round(360.0 / step_deg))
    angles = sorted(
        (wrap(base + k * math.radians(step_deg)) for k in range(n)),
        key=lambda a: abs(wrap(a - base)),
    )
    out: list[XY] = []
    radius = max(standoff, min_dist)
    while radius <= max(standoff, min_dist) + max_extra + 1e-9:
        for a in angles:
            p = (person[0] + radius * math.cos(a), person[1] + radius * math.sin(a))
            if math.dist(p, person) >= min_dist and free(p):
                out.append(p)
        radius += 0.05
    return out


class ComeToSkill(CompoundSkill):
    skill = "come_to"

    def __init__(self, cfg: SkillConfig, now: float, person: str, standoff_m: float = 1.5) -> None:
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
        self.t_face = 0.0
        self.xy_hold: XY = (0.0, 0.0)

    def on_answer(self, answer: dict) -> None:
        if answer.get("id") == self.qid:
            self.answer = answer

    def _next_candidate(self, ctx: Ctx) -> bool:
        if not self.cands or self.tries >= 4:
            return False
        p = self.cands.pop(0)
        # skip candidates right next to the one that just failed
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
            free = free_checker(ctx, self.cfg)
            self.cands = standoff_points(
                self.target, robot, free, self.standoff, self.cfg.min_person_m
            )
            self.ev(ctx, "person", at=[round(v, 2) for v in self.target], n_cands=len(self.cands))
            if not self._next_candidate(ctx):
                return self.finish_with("failed", "there is no free place near the person", ctx)
            self.state = "fly"
        if self.state == "fly":
            if el > self.cfg.goto_timeout_s:
                return self.finish_with("failed", "could not reach the person in time", ctx)
            st = self.step_leg(ctx, out)
            if st is None:
                return out
            if st == "failed":
                if not self._next_candidate(ctx):
                    return self.finish_with("failed", "I could not get near the person", ctx)
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
