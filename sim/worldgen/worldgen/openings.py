"""Openings: doors between rooms and open-plan gaps, chosen so the flat is connected."""

from __future__ import annotations

import random
from dataclasses import dataclass

from .floorplan import Room
from .geometry import Rect, dm
from .profile import DroneProfile
from .programme import BEDROOMS, Programme

MIN_MARGIN_DM = 2  # wall left on each side of an opening
DOOR_HEIGHT = 2.05


@dataclass(frozen=True)
class Opening:
    id: str
    rooms: tuple[str, str]
    kind: str  # "door" or "open"
    orient: str  # "v": wall at x = pos running along y; "h": wall at y = pos running along x
    pos: float
    start: float  # extent along the wall
    end: float

    @property
    def width(self) -> float:
        return round(self.end - self.start, 3)

    @property
    def center(self) -> tuple[float, float]:
        mid = (self.start + self.end) / 2
        return (self.pos, mid) if self.orient == "v" else (mid, self.pos)


def shared_wall(a: Rect, b: Rect) -> tuple[str, float, int, int] | None:
    """Shared boundary of two rooms as (orient, pos, start_dm, end_dm), or None."""
    for left, right in ((a, b), (b, a)):
        if abs(left.x1 - right.x0) < 1e-6:
            lo, hi = max(dm(left.y0), dm(right.y0)), min(dm(left.y1), dm(right.y1))
            if hi > lo:
                return "v", left.x1, lo, hi
        if abs(left.y1 - right.y0) < 1e-6:
            lo, hi = max(dm(left.x0), dm(right.x0)), min(dm(left.x1), dm(right.x1))
            if hi > lo:
                return "h", left.y1, lo, hi
    return None


def edge_allowed(a: str, b: str, has_hall: bool, has_bed: bool) -> bool:
    """Which room types may share a door."""
    pair = {a, b}
    if a == b:
        return False  # also keeps bedrooms from being reachable only through another bedroom
    if "bathroom" in pair:
        other = (pair - {"bathroom"}).pop()
        return other in ("hallway", "bedroom") or (
            other == "living" and not has_hall and not has_bed
        )
    if "bedroom" in pair or "office" in pair:
        other = pair - {"bedroom", "office"}
        return bool(other) and other <= {"hallway", "living"}
    return True


def make_openings(
    rng: random.Random, rooms: list[Room], prog: Programme, prof: DroneProfile
) -> list[Opening] | None:
    has_hall = any(r.type == "hallway" for r in rooms)
    has_bed = BEDROOMS[prog.flat_type] > 0
    candidates = []  # (priority, tiebreak, i, j, wall)
    for i, a in enumerate(rooms):
        for j in range(i + 1, len(rooms)):
            b = rooms[j]
            wall = shared_wall(a.rect, b.rect)
            if wall is None or wall[3] - wall[2] < prof.min_shared_dm:
                continue
            if not edge_allowed(a.type, b.type, has_hall, has_bed):
                continue
            types = {a.type, b.type}
            if types == {"kitchen", "living"}:
                prio = -1
            elif "hallway" in types:
                prio = 0
            elif "living" in types:
                prio = 1
            else:
                prio = 2
            candidates.append((prio, rng.random(), i, j, wall))
    candidates.sort(key=lambda c: c[:2])

    parent = list(range(len(rooms)))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    chosen = []
    rest = []
    for cand in candidates:
        ri, rj = find(cand[2]), find(cand[3])
        if ri != rj:
            parent[ri] = rj
            chosen.append(cand)
        else:
            rest.append(cand)
    if len({find(i) for i in range(len(rooms))}) != 1:
        return None
    if not any({rooms[c[2]].type, rooms[c[3]].type} == {"kitchen", "living"} for c in chosen):
        return None  # kitchen must be next to the living room
    public = {"living", "kitchen", "hallway", "office"}
    extras = 0
    for cand in rest:
        if (
            extras < 2
            and {rooms[cand[2]].type, rooms[cand[3]].type} <= public
            and rng.random() < 0.3
        ):
            chosen.append(cand)
            extras += 1

    openings = []
    for n, (_, _, i, j, (orient, pos, lo, hi)) in enumerate(chosen):
        a, b = rooms[i], rooms[j]
        kitchen_living = {a.type, b.type} == {"kitchen", "living"}
        if kitchen_living and prog.open_kitchen and hi - lo >= prof.open_min_dm + 2 * MIN_MARGIN_DM:
            width = min(hi - lo - 2 * MIN_MARGIN_DM, 24)
            kind = "open"
        else:
            width = rng.choice([w for w in prof.door_widths_dm if w + 2 * MIN_MARGIN_DM <= hi - lo])
            kind = "door"
        start = rng.randint(lo + MIN_MARGIN_DM, hi - MIN_MARGIN_DM - width)
        openings.append(
            Opening(f"d{n}", (a.id, b.id), kind, orient, pos, start / 10, (start + width) / 10)
        )
    return openings
