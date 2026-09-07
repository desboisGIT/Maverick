"""Floor texture without image files: many slightly different coloured tiles or planks."""

from __future__ import annotations

import random
from dataclasses import dataclass

from .floorplan import Room
from .geometry import Rect
from .materials import RGB, Palette, jitter, shade

GAP = 0.006


@dataclass(frozen=True)
class Tile:
    cx: float
    cy: float
    sx: float
    sy: float
    color: RGB


@dataclass(frozen=True)
class RoomFloor:
    room: str
    kind: str  # "planks" | "tiles" | "carpet" | "plain"
    base: RGB
    grout: RGB
    rect: Rect
    tiles: tuple[Tile, ...]


def floor_kinds(rng: random.Random, rooms: list[Room]) -> dict[str, str]:
    main = rng.choice(("planks", "planks", "tiles"))
    kitchen_tiles = rng.random() < 0.6
    out = {}
    for r in rooms:
        if r.type == "bathroom":
            out[r.id] = "tiles"
        elif r.type == "kitchen":
            out[r.id] = "tiles" if kitchen_tiles else main
        elif r.type in ("bedroom", "office") and rng.random() < 0.35:
            out[r.id] = "carpet"
        else:
            out[r.id] = main
    return out


def _grid(rng: random.Random, r: Rect, w: float, h: float, base: RGB, amount: float) -> list[Tile]:
    tiles = []
    nx, ny = int(r.w / w) + 1, int(r.h / h) + 1
    for i in range(nx):
        for j in range(ny):
            x0, y0 = r.x0 + i * w, r.y0 + j * h
            x1, y1 = min(x0 + w, r.x1), min(y0 + h, r.y1)
            if x1 - x0 < 0.02 or y1 - y0 < 0.02:
                continue
            c = jitter(rng, base, amount * (2.0 if rng.random() < 0.06 else 1.0))
            tiles.append(Tile((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0 - GAP, y1 - y0 - GAP, c))
    return tiles


def _planks(rng: random.Random, r: Rect, base: RGB, amount: float) -> list[Tile]:
    along_x = rng.random() < 0.5
    pw = round(rng.uniform(0.12, 0.2), 3)
    length_total = r.w if along_x else r.h
    rows = int((r.h if along_x else r.w) / pw) + 1
    tiles = []
    for k in range(rows):
        lo = (r.y0 if along_x else r.x0) + k * pw
        hi = min(lo + pw, r.y1 if along_x else r.x1)
        if hi - lo < 0.02:
            continue
        pos = -rng.uniform(0, 1.0)
        while pos < length_total:
            ln = rng.uniform(0.8, 1.3)
            a, b = max(pos, 0.0), min(pos + ln, length_total)
            pos += ln
            if b - a < 0.02:
                continue
            c = jitter(rng, base, amount)
            s0 = r.x0 if along_x else r.y0
            mid_a, mid_b = s0 + (a + b) / 2, (lo + hi) / 2
            along, across = b - a - GAP, hi - lo - GAP
            if along_x:
                tiles.append(Tile(mid_a, mid_b, along, across, c))
            else:
                tiles.append(Tile(mid_b, mid_a, across, along, c))
    return tiles


def make_floors(
    rng: random.Random,
    rooms: list[Room],
    pal: Palette,
    low_texture: bool = False,
) -> list[RoomFloor]:
    kinds = floor_kinds(rng, rooms)
    contrast = rng.uniform(0.10, 0.18)
    floors = []
    for room in rooms:
        kind = kinds[room.id]
        source = {"planks": pal.wood, "tiles": pal.tile, "carpet": pal.carpet}[kind]
        base = jitter(rng, rng.choice(source), 0.05)
        r = room.rect
        if low_texture:
            floors.append(RoomFloor(room.id, "plain", base, base, r, ()))
            continue
        if kind == "planks":
            tiles = _planks(rng, r, base, contrast)
        elif kind == "tiles":
            size = rng.choice((0.3, 0.4, 0.5, 0.6))
            tiles = _grid(
                rng,
                r,
                size,
                size * rng.choice((1, 1, 2)) if size <= 0.3 else size,
                base,
                contrast * 0.8,
            )
        else:
            tiles = _grid(rng, r, 0.5, 0.5, base, contrast * 0.5)
        floors.append(RoomFloor(room.id, kind, base, shade(base, 0.45), r, tuple(tiles)))
    return floors
