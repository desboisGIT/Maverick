"""Floor plan: a rectangular or L-shaped footprint split into axis-aligned rooms."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, replace

from .geometry import Rect
from .programme import Programme, RoomSpec


@dataclass(frozen=True)
class Room:
    id: str
    type: str
    rect: Rect


@dataclass(frozen=True)
class FloorPlan:
    footprint: list[list[float]]  # outline polygon, counter-clockwise
    wings: list[Rect]  # rectangles whose union is the footprint
    rooms: list[Room]


def _r1(v: float) -> float:
    return round(v, 1)


def _fits(spec: RoomSpec, r: Rect) -> bool:
    short, long = min(r.w, r.h), max(r.w, r.h)
    return (
        short >= spec.min_side - 1e-6
        and r.area >= spec.min_area
        and long / short <= spec.max_aspect
    )


def _bsp(
    rng: random.Random, rect: Rect, specs: list[RoomSpec]
) -> list[tuple[RoomSpec, Rect]] | None:
    """Recursively cut `rect` so that consecutive specs end up in consecutive sub-rectangles."""
    if len(specs) == 1:
        return [(specs[0], rect)] if _fits(specs[0], rect) else None
    total = sum(s.area for s in specs)
    splits = []
    for k in range(1, len(specs)):
        left = sum(s.area for s in specs[:k])
        splits.append((abs(left / total - 0.5) + rng.uniform(0, 0.15), k, left / total))
    splits.sort()
    for _, k, frac in splits:
        frac = min(0.8, max(0.2, frac + rng.uniform(-0.05, 0.05)))
        axes = ["x", "y"] if rect.w >= rect.h else ["y", "x"]
        if rng.random() < 0.2:
            axes.reverse()
        lo_a = min(s.min_side for s in specs[:k])
        lo_b = min(s.min_side for s in specs[k:])
        for axis in axes:
            start, length = (rect.x0, rect.w) if axis == "x" else (rect.y0, rect.h)
            # keep both parts at least as thick as their thinnest room needs
            low = math.ceil((start + lo_a) * 10 - 1e-6) / 10
            high = math.floor((start + length - lo_b) * 10 + 1e-6) / 10
            if low > high:
                continue
            cut = min(high, max(low, _r1(start + frac * length)))
            if axis == "x":
                a, b = Rect(rect.x0, rect.y0, cut, rect.y1), Rect(cut, rect.y0, rect.x1, rect.y1)
            else:
                a, b = Rect(rect.x0, rect.y0, rect.x1, cut), Rect(rect.x0, cut, rect.x1, rect.y1)
            if min(a.w, a.h) < lo_a - 1e-6 or min(b.w, b.h) < lo_b - 1e-6:
                continue
            left = _bsp(rng, a, specs[:k])
            if left is None:
                continue
            right = _bsp(rng, b, specs[k:])
            if right is not None:
                return left + right
    return None


def _shape(
    rng: random.Random, total: float, allow_l: bool
) -> tuple[list[Rect], list[list[float]]] | None:
    """Pick the footprint: returns its wings and outline polygon."""
    if allow_l and total >= 50 and rng.random() < 0.4:
        fx, fy = rng.uniform(0.35, 0.5), rng.uniform(0.35, 0.5)
        bound = total / (1 - fx * fy)
        w = math.sqrt(bound * rng.uniform(1.0, 1.6))
        w, d = _r1(w), _r1(bound / w)
        cw, cd = _r1(w * fx), _r1(d * fy)
        if min(w - cw, cd, d - cd) < 3.0:
            return None
        main = Rect(0.0, 0.0, w, _r1(d - cd))
        if rng.random() < 0.5:  # upper wing on the left
            top = Rect(0.0, main.y1, _r1(w - cw), d)
            outline = [[0, 0], [w, 0], [w, main.y1], [top.x1, main.y1], [top.x1, d], [0, d]]
        else:
            top = Rect(cw, main.y1, w, d)
            outline = [[0, 0], [w, 0], [w, d], [cw, d], [cw, main.y1], [0, main.y1]]
        return [main, top], [[float(a), float(b)] for a, b in outline]
    a = rng.uniform(1.0, 1.7)
    w = _r1(math.sqrt(total * a))
    d = _r1(total / w)
    if rng.random() < 0.5:
        w, d = d, w
    return [Rect(0.0, 0.0, w, d)], [[0.0, 0.0], [w, 0.0], [w, d], [0.0, d]]


def _split_for_wings(specs: tuple[RoomSpec, ...], area_a: float, scale: float) -> int:
    best, best_err = 1, math.inf
    for k in range(1, len(specs)):
        err = abs(sum(s.area for s in specs[:k]) * scale - area_a)
        if err < best_err:
            best, best_err = k, err
    return best


def make_floorplan(rng: random.Random, prog: Programme, attempts: int = 60) -> FloorPlan | None:
    for _ in range(attempts):
        shape = _shape(rng, prog.total_area, allow_l=len(prog.rooms) >= 4)
        if shape is None:
            continue
        wings, outline = shape
        scale = sum(w.area for w in wings) / prog.total_area
        specs = [replace(s, area=s.area * scale) for s in prog.rooms]
        if len(wings) == 1:
            groups = [specs]
        else:
            k = _split_for_wings(prog.rooms, wings[0].area, scale)
            groups = [specs[:k], specs[k:]]
        leaves: list[tuple[RoomSpec, Rect]] = []
        for wing, group in zip(wings, groups, strict=True):
            part = _bsp(rng, wing, group)
            if part is None:
                break
            leaves += part
        else:
            counts: dict[str, int] = {}
            rooms = []
            for spec, rect in leaves:
                n = counts.get(spec.type, 0)
                counts[spec.type] = n + 1
                rooms.append(Room(f"{spec.type}_{n}", spec.type, rect))
            return FloorPlan(outline, wings, rooms)
    return None
