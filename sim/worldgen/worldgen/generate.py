"""Pipeline: seed -> programme -> floor plan -> openings -> furniture -> SDF + manifest."""

from __future__ import annotations

import json
import random
from dataclasses import dataclass
from typing import Any

from .decor import make_lamps, make_panels
from .floorplan import make_floorplan
from .floors import make_floors
from .furniture import furnish
from .manifest import build_manifest, world_name
from .materials import PALETTES, jitter
from .openings import make_openings
from .programme import make_programme
from .sdf import LIGHTING, write_sdf
from .walls import build_walls

MAX_ATTEMPTS = 300
REDRAW_EVERY = 30


class GenerationError(RuntimeError):
    pass


@dataclass(frozen=True)
class World:
    name: str
    sdf: str
    manifest: dict[str, Any]

    def manifest_json(self) -> str:
        return json.dumps(self.manifest, indent=1) + "\n"


def sub_rng(seed: int, stage: str) -> random.Random:
    """Independent stream per stage, so output options never shift the layout draws."""
    return random.Random(f"{seed}/{stage}")


def generate(
    seed: int,
    *,
    ceiling: bool = True,
    low_texture: bool = False,
    lighting: str | None = None,
) -> World:
    if lighting is not None and lighting not in LIGHTING:
        raise ValueError(f"unknown lighting preset {lighting!r}")
    rng = random.Random(seed)
    for attempt in range(MAX_ATTEMPTS):
        if attempt % REDRAW_EVERY == 0:  # keep the programme for a few tries to avoid bias
            prog = make_programme(rng)
        plan = make_floorplan(rng, prog)
        if plan is None:
            continue
        openings = make_openings(rng, plan.rooms, prog)
        if openings is None:
            continue
        palette = rng.choice(PALETTES)
        result = furnish(rng, plan, openings, palette)
        if result is not None:
            furniture, spawn = result
            break
    else:
        raise GenerationError(f"no valid flat for seed {seed}")

    ceiling_height = round(rng.uniform(2.4, 2.7) * 20) / 20
    walls = build_walls(plan.rooms, openings)
    deco = sub_rng(seed, "decor")
    wall_main = deco.choice(palette.walls)
    wall_colors = {}
    for r in plan.rooms:
        base = wall_main if deco.random() < 0.65 else deco.choice(palette.walls)
        wall_colors[r.id] = jitter(deco, base, 0.03)
    panels = make_panels(deco, plan.rooms, walls, furniture, palette, ceiling_height)
    lamps = make_lamps(plan.rooms)
    floors = make_floors(sub_rng(seed, "floor"), plan.rooms, palette, low_texture)
    preset = (
        lighting
        or sub_rng(seed, "lighting").choices(("day", "evening", "lamp_only"), (0.4, 0.35, 0.25))[0]
    )

    name = world_name(seed)
    sdf = write_sdf(
        name,
        plan,
        openings,
        floors,
        walls,
        wall_colors,
        panels,
        lamps,
        furniture,
        palette,
        ceiling_height,
        preset,
        ceiling,
    )
    manifest = build_manifest(
        seed,
        prog.flat_type,
        plan,
        openings,
        furniture,
        spawn,
        panels,
        ceiling_height,
        preset,
        palette.name,
        {"ceiling": ceiling, "low_texture": low_texture},
    )
    return World(name, sdf, manifest)
