"""Ground-truth manifest: what the referee reads to score a run."""

from __future__ import annotations

from typing import Any

from .decor import Panel
from .floorplan import FloorPlan
from .furniture import Placed, Spawn
from .openings import Opening

GENERATOR_VERSION = "1"
TUNING_SEEDS = (0, 99_999)
HELD_OUT_SEEDS = (1_000_000, 1_099_999)


def seed_set(seed: int) -> str:
    if TUNING_SEEDS[0] <= seed <= TUNING_SEEDS[1]:
        return "tuning"
    if HELD_OUT_SEEDS[0] <= seed <= HELD_OUT_SEEDS[1]:
        return "held_out"
    return "other"


def world_name(seed: int) -> str:
    return f"apt_{seed}"


def _r(v: float) -> float:
    return round(v, 3)


def build_manifest(
    seed: int,
    flat_type: str,
    plan: FloorPlan,
    openings: list[Opening],
    furniture: list[Placed],
    spawn: Spawn,
    panels: list[Panel],
    ceiling_height: float,
    lighting: str,
    palette: str,
    drone_profile: str,
    options: dict[str, Any],
) -> dict[str, Any]:
    return {
        "seed": seed,
        "seed_set": seed_set(seed),
        "generator_version": GENERATOR_VERSION,
        "world": world_name(seed),
        "flat_type": flat_type,
        "palette": palette,
        "drone_profile": drone_profile,
        "options": options,
        "footprint": [[_r(x), _r(y)] for x, y in plan.footprint],
        "ceiling_height": ceiling_height,
        "rooms": [
            {
                "id": r.id,
                "type": r.type,
                "polygon": [[_r(x), _r(y)] for x, y in r.rect.corners()],
                "area_m2": round(r.rect.area, 2),
            }
            for r in plan.rooms
        ],
        "doors": [
            {
                "id": o.id,
                "rooms": list(o.rooms),
                "kind": o.kind,
                "center": [_r(o.center[0]), _r(o.center[1])],
                "width": o.width,
            }
            for o in openings
        ],
        "furniture": [
            {
                "id": p.id,
                "class": p.cls,
                "room": p.room,
                "pose": [p.x, p.y, 0.0, p.yaw],
                "bbox": [
                    [_r(p.rect.x0), _r(p.rect.y0), 0.0],
                    [_r(p.rect.x1), _r(p.rect.y1), _r(p.inst.height)],
                ],
                "surfaces": p.inst.surface_polygons(p.x, p.y, p.yaw),
            }
            for p in furniture
        ],
        "windows": [
            {
                "room": w.room,
                "center": [
                    _r(w.face) if w.orient == "v" else _r(w.u),
                    _r(w.u) if w.orient == "v" else _r(w.face),
                    _r(w.z),
                ],
                "width": w.w,
                "height": w.h,
            }
            for w in panels
            if w.kind == "window"
        ],
        "spawn": {"room": spawn.room, "pose": [spawn.x, spawn.y, 0.0, spawn.yaw]},
        "lighting": lighting,
    }
