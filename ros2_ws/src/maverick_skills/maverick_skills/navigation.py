"""Building blocks for compound skills (search, come_to, return): fly to a point, face a point.

`GotoXY` is the goto_room skill with the goal given instead of asked from memory: same A*, squeeze
handling, stuck detection and replanning (no new planner). `CompoundSkill` steps one such leg at a
time and forwards its goal / path to the node. Pure Python, no ROS imports.
"""

from __future__ import annotations

import dataclasses
import math

from maverick_mapping.planner import PlanningGrid

from .skills import XY, Ctx, GotoRoomSkill, Out, SkillConfig, _Base


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to(src: XY, dst: XY) -> float:
    return math.atan2(dst[1] - src[1], dst[0] - src[0])


class GotoXY(GotoRoomSkill):
    """Fly to a map-frame point along a planned path; succeeded within `tolerance` of it."""

    def __init__(
        self, cfg: SkillConfig, now: float, goal_xy: XY, label: str = "the goal", tolerance=None
    ) -> None:
        if tolerance is not None:
            cfg = dataclasses.replace(cfg, goal_tolerance=tolerance)
        super().__init__(cfg, now, label)
        self.goal_xy = (float(goal_xy[0]), float(goal_xy[1]))
        self.state = "plan"
        self.d0 = 1.0


class CompoundSkill(_Base):
    """A skill made of legs (GotoXY) and in-place actions; keeps one leg at a time."""

    def __init__(self, cfg: SkillConfig, now: float) -> None:
        super().__init__(cfg, now)
        self.leg: GotoXY | None = None
        self.last_message = ""
        self.result_data: dict | None = None

    def start_leg(self, ctx: Ctx, goal: XY, label: str, tolerance: float | None = None) -> None:
        self.leg = GotoXY(self.cfg, ctx.now, goal, label, tolerance)
        self.ev(ctx, "leg", goal=[round(goal[0], 2), round(goal[1], 2)], label=label)

    def step_leg(self, ctx: Ctx, out: Out) -> str | None:
        """Advance the current leg. None while it runs, else 'succeeded' or 'failed'."""
        leg = self.leg
        o = leg.step(ctx)
        out.goal, out.speed, out.hold = o.goal, o.speed, o.hold
        if o.path is not None:
            out.path = o.path
            self.last_path = o.path
        self.events.extend(leg.events)
        leg.events.clear()
        if leg.finished:
            self.ev(ctx, "leg_end", state=leg.result, message=leg.result and o.progress["message"])
            self.leg = None
            return leg.result
        return None

    def on_guard(self, ctx: Ctx, info: dict) -> None:
        if self.leg is not None and not self.leg.finished:
            self.leg.on_guard(ctx, info)

    def progress(self, state, progress, message, eta=None) -> dict:
        d = super().progress(state, progress, message, eta)
        if self.result_data is not None:
            d["result"] = self.result_data
        return d

    def finish_with(self, state: str, message: str, ctx: Ctx, result: dict | None = None) -> Out:
        self.result_data = result
        return self.finish(state, message, ctx, hold=True)


def free_checker(ctx: Ctx, cfg: SkillConfig):
    """`free(xy) -> bool`: the point is free in the planner's inflated view of the map."""
    grid, res, org = ctx.grid
    pg = PlanningGrid.build(grid, res, org, cfg.planner, ctx.robot_xy, None)

    def free(xy: XY) -> bool:
        i, j = pg.to_cell(xy)
        return bool(pg.free_at(i, j))

    return free
