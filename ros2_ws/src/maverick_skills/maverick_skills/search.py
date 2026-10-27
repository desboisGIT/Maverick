"""search(object, region?, budget_s): look for an object by flying through known rooms.

For each candidate room (the requested region, else every known room, most promising first) the
drone flies to the room's goal, sweeps a full yaw circle, then visits viewpoints that face each
surface (counter, table, desk, ...) memory holds for that room. It stops as soon as the detector
reports the object with confidence >= `search_conf` in at least two frames at positions within
`agree_m` of each other, confirms with memory (which keeps the position and a photo crop), and
reports it. When the budget or the rooms run out it reports "not found, searched <rooms>".

Perception: the simulated detector from the segmentation camera (oracle-quality with noise);
detections arrive as memory's /maverick/memory/detections payloads, with a lifted map-frame
position per box. No ground truth is read here. Pure Python (see navigation.py).
"""

from __future__ import annotations

import math
from collections.abc import Callable

from .navigation import CompoundSkill, free_checker, wrap, yaw_to
from .skills import XY, Ctx, Out, SkillConfig

try:
    from maverick_memory.store import canonical_label
except ImportError:  # pragma: no cover

    def canonical_label(label: str) -> str:
        return label.strip().lower().replace(" ", "_")


# Where an object is likely to be, most likely first (rooms not listed follow by distance).
PRIOR: dict[str, tuple[str, ...]] = {
    "banana": ("kitchen", "living"),
    "bread": ("kitchen", "living"),
    "person": ("living", "bedroom", "office"),
}
ROOM_NAMES = {"living": "living room"}


def room_name(rtype: str) -> str:
    return ROOM_NAMES.get(rtype, rtype)


def surface_viewpoints(
    lo: XY,
    hi: XY,
    free: Callable[[XY], bool],
    standoff: float = 1.3,
    long_side: float = 1.8,
) -> list[tuple[XY, float]]:
    """Viewpoints (xy, facing yaw) around a surface box, on the sides where the point is free.

    A box with a side longer than `long_side` gets two viewpoints along that side (a camera at
    1.3 m sees about 1.7 m of it), others one at the middle.
    """
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    hx, hy = (hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2
    out: list[tuple[XY, float]] = []
    for nx, ny in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        along_x = nx == 0  # the side faces +-y, so it runs along x
        length = 2 * (hx if along_x else hy)
        offs = (-length / 4, length / 4) if length > long_side else (0.0,)
        for off in offs:
            tx, ty = (off, 0.0) if along_x else (0.0, off)
            px = cx + nx * (hx + standoff) + tx
            py = cy + ny * (hy + standoff) + ty
            tgt = (cx + tx, cy + ty)
            if free((px, py)):
                out.append(((px, py), yaw_to((px, py), tgt)))
    return out


def order_nearest(
    points: list[tuple[XY, float]], start: XY, avoid: list[XY] | None = None, min_dist=1.2
) -> list[tuple[XY, float]]:
    """Greedy nearest-neighbour tour; points closer than `min_dist` to a person are dropped."""
    left = [p for p in points if not any(math.dist(p[0], a) < min_dist for a in avoid or [])]
    tour, cur = [], start
    while left:
        k = min(range(len(left)), key=lambda i: math.dist(left[i][0], cur))
        tour.append(left.pop(k))
        cur = tour[-1][0]
    return tour


def order_rooms(rooms: list[dict], label: str, start: XY | None, region: str | None) -> list[dict]:
    """Rooms most promising first: those where memory already saw the object, then the prior
    for the class, then nearest."""
    prior = PRIOR.get(label, ())

    def key(r: dict):
        d = math.dist(r["goal"], start) if start is not None else 0.0
        t = r.get("type")
        rank = prior.index(t) if t in prior else len(prior)
        return (0 if r.get("known") else 1, rank if region is None else 0, d)

    return sorted(rooms, key=key)


def clear_goal(goal: XY, people: list, free: Callable[[XY], bool], min_d: float) -> XY:
    """`goal`, or the nearest free point (within 2 m, 0.25 m steps, 16 directions) that is at
    least `min_d` from every person, so a room goal next to the operator is not refused."""
    if all(math.dist(goal, p) >= min_d for p in people):
        return goal
    for k in range(1, 9):
        for n in range(16):
            a = 2 * math.pi * n / 16
            c = (goal[0] + 0.25 * k * math.cos(a), goal[1] + 0.25 * k * math.sin(a))
            if all(math.dist(c, p) >= min_d for p in people) and free(c):
                return c
    return goal


class SearchSkill(CompoundSkill):
    skill = "search"

    def __init__(
        self,
        cfg: SkillConfig,
        now: float,
        label: str,
        region: str | None = None,
        budget_s: float = 120.0,
    ) -> None:
        super().__init__(cfg, now)
        self.label = canonical_label(label)
        self.region = region
        self.budget = float(budget_s)
        self.state = "plan"
        self.qid = f"s{int(now * 1000) % 10**9}"
        self.answer: dict | None = None
        self.t_query = now
        self.rooms: list[dict] = []
        self.room_i = -1
        self.room: dict | None = None
        self.searched: list[str] = []
        self.views: list[tuple[XY, float]] = []
        self.hits: list[tuple[int, float, float, float, float]] = []  # frame, x, y, z, conf
        self.n_frames = 0
        self.found: tuple[float, float, float] | None = None
        self.sweep_i = 0
        self.sweep_yaw0 = 0.0
        self.sweep_t = 0.0
        self.sweep_arrived: float | None = None
        self.dwell_t = 0.0
        self.view_yaw = 0.0
        self.view_phase = 0
        self.xy_hold: XY = (0.0, 0.0)

    # -- memory ------------------------------------------------------------------------------
    def on_answer(self, answer: dict) -> None:
        if answer.get("id") == self.qid:
            self.answer = answer

    def _ask(self, ctx: Ctx, out: Out, query: dict) -> None:
        self.qid = f"s{int(ctx.now * 1000) % 10**9}-{self.n_frames}"
        out.query = {"id": self.qid, **query}
        self.answer, self.t_query = None, ctx.now

    # -- detections --------------------------------------------------------------------------
    def _ingest(self, ctx: Ctx) -> bool:
        """Add this tick's frames; True when two frames agree on a position."""
        for f in ctx.frames:
            self.n_frames += 1
            for b in f.get("boxes", []):
                pos = b.get("position")
                if (
                    canonical_label(str(b.get("label", ""))) == self.label
                    and pos is not None
                    and float(b.get("confidence", 0.0)) >= self.cfg.search_conf
                ):
                    self.hits.append(
                        (self.n_frames, pos[0], pos[1], pos[2], float(b["confidence"]))
                    )
        return self._agreeing() is not None

    def _agreeing(self) -> tuple[float, float, float] | None:
        h = self.hits
        for a in h:
            grp = [b for b in h if math.hypot(a[1] - b[1], a[2] - b[2]) <= self.cfg.search_agree_m]
            if len({g[0] for g in grp}) >= 2:
                xs = sorted(g[1] for g in grp)
                ys = sorted(g[2] for g in grp)
                zs = sorted(g[3] for g in grp)
                m = len(grp) // 2
                return xs[m], ys[m], zs[m]
        return None

    # -- states ------------------------------------------------------------------------------
    def _msg(self, text: str) -> str:
        self.last_message = text
        return text

    def _not_found(self, ctx: Ctx, why: str) -> Out:
        seen = ", ".join(room_name(t) for t in self.searched) or "nowhere"
        res = {
            "found": False,
            "object": self.label,
            "rooms_searched": list(self.searched),
            "reason": why,
        }
        return self.finish_with("succeeded", f"not found, searched {seen}", ctx, res)

    def _next_room(self, ctx: Ctx) -> bool:
        self.room_i += 1
        if self.room_i >= len(self.rooms):
            return False
        self.room = self.rooms[self.room_i]
        self.views, self.view_phase = [], 0
        self.state = "go"
        goal = tuple(self.room["goal"])
        if ctx.people and ctx.grid is not None:
            goal = clear_goal(
                goal, ctx.people, free_checker(ctx, self.cfg), self.cfg.search_person_avoid_m
            )
        self.start_leg(ctx, goal, room_name(self.room["type"]), 0.3)
        self.ev(ctx, "room", id=self.room["id"], type=self.room["type"], goal=self.room["goal"])
        return True

    def _room_done(self) -> None:
        t = self.room["type"] if self.room else None
        if t and t not in self.searched:
            self.searched.append(t)

    def _sweep(self, ctx: Ctx, out: Out) -> bool:
        """Turn on the spot in yaw steps, pausing at each; True when the circle is done."""
        c = self.cfg
        n = int(round(2 * math.pi / c.scan_step_rad))
        target = self.sweep_yaw0 + (self.sweep_i + 1) * c.scan_step_rad
        if abs(wrap(ctx.robot_yaw - target)) < 0.25 and self.sweep_arrived is None:
            self.sweep_arrived = ctx.now
        dwelled = (
            self.sweep_arrived is not None and ctx.now - self.sweep_arrived >= c.search_dwell_s
        )
        if dwelled or ctx.now - self.sweep_t > c.scan_step_timeout_s + c.search_dwell_s:
            self.sweep_i += 1
            self.sweep_t, self.sweep_arrived = ctx.now, None
            if self.sweep_i >= n:
                return True
            target += c.scan_step_rad
        out.goal = (self.xy_hold[0], self.xy_hold[1], target)
        return False

    def _start_sweep(self, ctx: Ctx) -> None:
        self.state = "sweep"
        self.sweep_i, self.sweep_t, self.sweep_arrived = 0, ctx.now, None
        self.sweep_yaw0 = ctx.robot_yaw if ctx.robot_yaw is not None else 0.0
        self.xy_hold = ctx.robot_xy

    def _plan_views(self, ctx: Ctx) -> None:
        free = free_checker(ctx, self.cfg)
        pts: list[tuple[XY, float]] = []
        for s in self.room.get("surfaces", []):
            pts += surface_viewpoints(
                (s["lo"][0], s["lo"][1]), (s["hi"][0], s["hi"][1]), free, self.cfg.search_standoff
            )
        self.views = order_nearest(pts, ctx.robot_xy, ctx.people, self.cfg.search_person_avoid_m)
        pts_log = [[round(p[0][0], 2), round(p[0][1], 2)] for p in self.views]
        self.ev(ctx, "views", n=len(self.views), pts=pts_log)
        self.state = "views"

    def _next_view(self, ctx: Ctx) -> bool:
        if not self.views:
            self._room_done()
            return self._next_room(ctx)
        (xy, yaw), *self.views = self.views
        self.view_yaw = yaw
        self.start_leg(ctx, xy, "a viewpoint", 0.2)
        self.state = "view_go"
        return True

    def step(self, ctx: Ctx) -> Out:
        el = ctx.now - self.t0
        out = Out()
        out.progress = self.progress(
            "running",
            min(el / self.budget, 0.99),
            f"searching for the {self.label}",
            self.budget - el,
        )
        if ctx.grid is None or ctx.robot_xy is None:
            out.progress["message"] = "waiting for the map"
            return out
        searching = self.state not in ("confirm_wait", "done")
        if searching and self._ingest(ctx):
            self.found = self._agreeing()
            self.xy_hold, self.leg = ctx.robot_xy, None
            self.ev(ctx, "found", pos=[round(v, 3) for v in self.found], hits=len(self.hits))
            self._ask(ctx, out, {"query": "find", "label": self.label, "confirmed": False})
            self.state = "confirm_wait"
            return out
        if searching and el >= self.budget:
            self._room_done()
            return self._not_found(ctx, "time budget used up")

        if self.state == "plan":
            self._ask(
                ctx,
                out,
                {
                    "query": "search_plan",
                    "label": self.label,
                    "region": self.region,
                    "start": list(ctx.robot_xy),
                },
            )
            self.state = "plan_wait"
            return out

        if self.state == "plan_wait":
            if self.answer is None:
                if ctx.now - self.t_query > self.cfg.query_timeout_s:
                    return self.finish_with("failed", "memory did not answer", ctx)
                return out
            if not self.answer.get("ok") or not self.answer.get("rooms"):
                why = self.answer.get("reason") or "no room to search is known yet"
                return self.finish_with("failed", f"I cannot search: {why}", ctx)
            self.rooms = order_rooms(self.answer["rooms"], self.label, ctx.robot_xy, self.region)
            self.ev(ctx, "plan", rooms=[(r["id"], r["type"]) for r in self.rooms])
            if not self._next_room(ctx):
                return self._not_found(ctx, "no room")

        if self.state == "confirm_wait":
            fx, fy, _ = self.found
            out.goal = (self.xy_hold[0], self.xy_hold[1], yaw_to(self.xy_hold, (fx, fy)))
            out.progress["message"] = f"found the {self.label}, checking"
            if self.answer is None and ctx.now - self.t_query < self.cfg.query_timeout_s:
                return out
            return self._finish_found(ctx)

        name = room_name(self.room["type"]) if self.room else "room"
        if self.state == "go":
            out.progress["message"] = f"going to the {name} to look"
            st = self.step_leg(ctx, out)
            if st is None:
                return out
            if st == "failed":
                self.ev(ctx, "room_unreachable", id=self.room["id"])
                self._room_done()
                if not self._next_room(ctx):
                    return self._not_found(ctx, "rooms exhausted")
                return out
            if ctx.robot_yaw is None:  # no yaw feedback: no sweeps
                self._plan_views(ctx)
            else:
                self._start_sweep(ctx)

        if self.state == "sweep":
            out.progress["message"] = f"looking around the {name}"
            if not self._sweep(ctx, out):
                return out
            self._plan_views(ctx)

        if self.state == "views":
            if not self._next_view(ctx):
                return self._not_found(ctx, "rooms exhausted")
            if self.state != "view_go":  # moved on to the next room
                return out

        if self.state == "view_go":
            out.progress["message"] = f"checking the surfaces in the {name}"
            st = self.step_leg(ctx, out)
            if st is None:
                return out
            if st == "failed":
                self.state = "views"
                return out
            self.state, self.view_phase = "view_look", 0
            self.dwell_t, self.xy_hold = ctx.now, ctx.robot_xy

        if self.state == "view_look":
            c = self.cfg
            yaws = (self.view_yaw, self.view_yaw - 0.45, self.view_yaw + 0.45)
            out.goal = (self.xy_hold[0], self.xy_hold[1], yaws[self.view_phase])
            out.progress["message"] = f"checking the surfaces in the {name}"
            if ctx.now - self.dwell_t >= c.search_look_s:
                self.view_phase += 1
                self.dwell_t = ctx.now
                if self.view_phase >= len(yaws):
                    self.state = "views"
        return out

    def _finish_found(self, ctx: Ctx) -> Out:
        fx, fy, fz = self.found
        obj = None
        for o in (self.answer or {}).get("objects", []):
            d = math.hypot(o["position"][0] - fx, o["position"][1] - fy)
            if d < 0.6 and (obj is None or d < obj[0]):
                obj = (d, o)
        o = obj[1] if obj else {}
        rtype = self.room["type"] if self.room else None
        res = {
            "found": True,
            "object": self.label,
            "position": [round(fx, 3), round(fy, 3), round(fz, 3)],
            "room_type": rtype,
            "object_id": o.get("id"),
            "photo": o.get("photo"),
            "rooms_searched": self.searched + ([rtype] if rtype not in self.searched else []),
        }
        where = f" in the {room_name(rtype)}" if rtype else ""
        return self.finish_with("succeeded", f"found the {self.label}{where}", ctx, res)
