"""Walls as merged segments with door and gap openings cut out."""

from __future__ import annotations

from dataclasses import dataclass

from .floorplan import Room
from .geometry import dm
from .openings import Opening

THICKNESS = 0.1


@dataclass(frozen=True)
class WallSegment:
    orient: str  # "v": runs along y at x = pos; "h": runs along x at y = pos
    pos: float
    start: float  # centre-line extent, already extended into corner junctions
    end: float
    low: str | None  # room on the -x (v) or -y (h) side, None = outside
    high: str | None  # room on the +x (v) or +y (h) side

    @property
    def length(self) -> float:
        return self.end - self.start


def build_walls(rooms: list[Room], openings: list[Opening]) -> list[WallSegment]:
    owner: dict[tuple[int, int], str] = {}
    for r in rooms:
        for i in range(dm(r.rect.x0), dm(r.rect.x1)):
            for j in range(dm(r.rect.y0), dm(r.rect.y1)):
                owner[(i, j)] = r.id

    # unit wall edges: key (orient, pos_dm, index_dm) -> (low, high)
    edges: dict[tuple[str, int, int], tuple[str | None, str | None]] = {}
    for (i, j), rid in owner.items():
        west, south = owner.get((i - 1, j)), owner.get((i, j - 1))
        if west != rid:
            edges[("v", i, j)] = (west, rid)
        if south != rid:
            edges[("h", j, i)] = (south, rid)
        if (i + 1, j) not in owner:
            edges[("v", i + 1, j)] = (rid, None)
        if (i, j + 1) not in owner:
            edges[("h", j + 1, i)] = (rid, None)

    # a wall end extends by half a thickness when another wall meets it there
    def junction(orient: str, pos: int, along: int) -> bool:
        if orient == "v":  # vertex (pos, along): look for horizontal edges touching it
            return ("h", along, pos - 1) in edges or ("h", along, pos) in edges
        return ("v", along, pos - 1) in edges or ("v", along, pos) in edges

    cut = set()
    for o in openings:
        for k in range(dm(o.start), dm(o.end)):
            cut.add((o.orient, dm(o.pos), k))

    segments: list[WallSegment] = []
    runs: dict[tuple[str, int, tuple[str | None, str | None]], list[int]] = {}
    for key in sorted(edges, key=lambda k: (k[0], k[1], k[2])):
        if key in cut:
            continue
        runs.setdefault((key[0], key[1], edges[key]), []).append(key[2])
    half = THICKNESS / 2
    for (orient, pos, (low, high)), idx in runs.items():
        idx.sort()
        begin = prev = idx[0]
        for k in idx[1:] + [None]:
            if k is not None and k == prev + 1:
                prev = k
                continue
            a, b = begin / 10, (prev + 1) / 10
            if junction(orient, pos, begin):
                a -= half
            if junction(orient, pos, prev + 1):
                b += half
            segments.append(WallSegment(orient, pos / 10, a, b, low, high))
            if k is not None:
                begin = prev = k
    segments.sort(key=lambda s: (s.orient, s.pos, s.start, s.end))
    return segments
