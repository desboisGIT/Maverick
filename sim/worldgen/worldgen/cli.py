"""Command line: python -m worldgen --seed 7 [--count N] [--out DIR]."""

from __future__ import annotations

import argparse
from pathlib import Path

from .generate import generate
from .profile import DEFAULT_PROFILE, PROFILES
from .sdf import LIGHTING
from .validate import validate


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="worldgen", description=__doc__)
    ap.add_argument("--seed", type=int, required=True, help="first seed")
    ap.add_argument("--count", type=int, default=1, help="number of consecutive seeds")
    ap.add_argument("--out", type=Path, default=Path("sim/worlds/generated"))
    ap.add_argument("--no-ceiling", action="store_true", help="omit the ceiling (top-down view)")
    ap.add_argument("--low-texture", action="store_true", help="plain single-colour floors")
    ap.add_argument("--lighting", choices=sorted(LIGHTING), help="override the seeded preset")
    ap.add_argument(
        "--drone", choices=sorted(PROFILES), default=DEFAULT_PROFILE, help="drone size profile"
    )
    args = ap.parse_args(argv)

    args.out.mkdir(parents=True, exist_ok=True)
    status = 0
    for seed in range(args.seed, args.seed + args.count):
        world = generate(
            seed,
            ceiling=not args.no_ceiling,
            low_texture=args.low_texture,
            lighting=args.lighting,
            drone=args.drone,
        )
        problems = validate(world.manifest, world.sdf)
        (args.out / f"{world.name}.sdf").write_text(world.sdf)
        (args.out / f"{world.name}.json").write_text(world.manifest_json())
        rooms = len(world.manifest["rooms"])
        print(
            f"{world.name}: {world.manifest['flat_type']}, {rooms} rooms, "
            f"{len(world.manifest['furniture'])} furniture, spawn {world.manifest['spawn']['pose']}"
            + (f"  INVALID: {problems}" if problems else "")
        )
        status |= bool(problems)
    return status
