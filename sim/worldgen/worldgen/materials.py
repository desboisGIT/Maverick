"""Colour palettes and per-object colour roles. All colours are linear-ish RGB in 0..1."""

from __future__ import annotations

import random
from dataclasses import dataclass

RGB = tuple[float, float, float]


def clamp(v: float) -> float:
    return min(1.0, max(0.0, v))


def shade(c: RGB, f: float) -> RGB:
    return (clamp(c[0] * f), clamp(c[1] * f), clamp(c[2] * f))


def jitter(rng: random.Random, c: RGB, amount: float) -> RGB:
    """Brightness jitter plus a small per-channel tint, so neighbours differ visibly."""
    f = 1 + rng.uniform(-amount, amount)
    return (
        clamp(c[0] * f + rng.uniform(-amount, amount) * 0.25),
        clamp(c[1] * f + rng.uniform(-amount, amount) * 0.25),
        clamp(c[2] * f + rng.uniform(-amount, amount) * 0.25),
    )


@dataclass(frozen=True)
class Palette:
    name: str
    walls: tuple[RGB, ...]
    ceiling: RGB
    trim: RGB
    wood: tuple[RGB, ...]  # floors and furniture wood
    tile: tuple[RGB, ...]  # kitchen / bathroom floors
    carpet: tuple[RGB, ...]
    fabric: tuple[RGB, ...]
    accent: tuple[RGB, ...]
    cabinet: tuple[RGB, ...]
    stone: tuple[RGB, ...]  # worktops


PALETTES: tuple[Palette, ...] = (
    Palette(
        "scandinavian",
        walls=((0.93, 0.93, 0.91), (0.86, 0.9, 0.92), (0.95, 0.9, 0.85)),
        ceiling=(0.96, 0.96, 0.95),
        trim=(0.97, 0.97, 0.96),
        wood=((0.78, 0.62, 0.42), (0.72, 0.56, 0.38)),
        tile=((0.82, 0.83, 0.84), (0.7, 0.74, 0.76)),
        carpet=((0.74, 0.74, 0.72), (0.6, 0.66, 0.7)),
        fabric=((0.55, 0.62, 0.68), (0.82, 0.78, 0.72), (0.4, 0.45, 0.5)),
        accent=((0.85, 0.55, 0.35), (0.35, 0.55, 0.5), (0.9, 0.8, 0.45)),
        cabinet=((0.92, 0.92, 0.9), (0.6, 0.72, 0.7)),
        stone=((0.85, 0.85, 0.83), (0.3, 0.3, 0.32)),
    ),
    Palette(
        "warm",
        walls=((0.93, 0.85, 0.7), (0.88, 0.72, 0.58), (0.94, 0.9, 0.78)),
        ceiling=(0.95, 0.93, 0.88),
        trim=(0.95, 0.92, 0.86),
        wood=((0.55, 0.35, 0.2), (0.62, 0.4, 0.24)),
        tile=((0.75, 0.55, 0.4), (0.8, 0.7, 0.55)),
        carpet=((0.6, 0.35, 0.3), (0.7, 0.55, 0.4)),
        fabric=((0.7, 0.35, 0.25), (0.55, 0.45, 0.3), (0.35, 0.28, 0.22)),
        accent=((0.2, 0.45, 0.4), (0.85, 0.65, 0.2), (0.6, 0.2, 0.2)),
        cabinet=((0.5, 0.33, 0.2), (0.85, 0.8, 0.7)),
        stone=((0.7, 0.62, 0.5), (0.25, 0.22, 0.2)),
    ),
    Palette(
        "modern_grey",
        walls=((0.8, 0.8, 0.82), (0.9, 0.9, 0.9), (0.62, 0.65, 0.7)),
        ceiling=(0.94, 0.94, 0.95),
        trim=(0.95, 0.95, 0.95),
        wood=((0.45, 0.4, 0.36), (0.55, 0.5, 0.45)),
        tile=((0.55, 0.57, 0.6), (0.4, 0.42, 0.45)),
        carpet=((0.45, 0.47, 0.5), (0.65, 0.65, 0.68)),
        fabric=((0.3, 0.32, 0.36), (0.6, 0.6, 0.62), (0.25, 0.3, 0.4)),
        accent=((0.85, 0.3, 0.25), (0.95, 0.75, 0.2), (0.2, 0.5, 0.75)),
        cabinet=((0.22, 0.23, 0.25), (0.88, 0.88, 0.88)),
        stone=((0.18, 0.18, 0.2), (0.8, 0.8, 0.8)),
    ),
    Palette(
        "rustic",
        walls=((0.85, 0.8, 0.7), (0.72, 0.75, 0.65), (0.9, 0.86, 0.78)),
        ceiling=(0.93, 0.91, 0.85),
        trim=(0.45, 0.32, 0.2),
        wood=((0.45, 0.3, 0.18), (0.38, 0.26, 0.16)),
        tile=((0.62, 0.45, 0.35), (0.55, 0.5, 0.45)),
        carpet=((0.55, 0.45, 0.35), (0.5, 0.55, 0.45)),
        fabric=((0.5, 0.55, 0.4), (0.65, 0.5, 0.35), (0.45, 0.3, 0.3)),
        accent=((0.75, 0.35, 0.2), (0.3, 0.45, 0.3), (0.85, 0.75, 0.5)),
        cabinet=((0.4, 0.28, 0.18), (0.55, 0.6, 0.5)),
        stone=((0.5, 0.48, 0.45), (0.25, 0.25, 0.25)),
    ),
    Palette(
        "pastel",
        walls=((0.95, 0.85, 0.87), (0.85, 0.92, 0.9), (0.88, 0.88, 0.96)),
        ceiling=(0.97, 0.97, 0.97),
        trim=(0.98, 0.98, 0.98),
        wood=((0.85, 0.72, 0.55), (0.8, 0.68, 0.55)),
        tile=((0.78, 0.88, 0.88), (0.9, 0.8, 0.82)),
        carpet=((0.85, 0.75, 0.8), (0.75, 0.82, 0.9)),
        fabric=((0.9, 0.7, 0.75), (0.7, 0.82, 0.85), (0.85, 0.85, 0.65)),
        accent=((0.55, 0.7, 0.9), (0.95, 0.65, 0.55), (0.6, 0.8, 0.6)),
        cabinet=((0.95, 0.88, 0.85), (0.8, 0.9, 0.88)),
        stone=((0.9, 0.9, 0.9), (0.6, 0.6, 0.65)),
    ),
    Palette(
        "dark_moody",
        walls=((0.25, 0.3, 0.33), (0.3, 0.25, 0.3), (0.5, 0.52, 0.5)),
        ceiling=(0.8, 0.8, 0.8),
        trim=(0.12, 0.12, 0.13),
        wood=((0.3, 0.2, 0.14), (0.25, 0.18, 0.13)),
        tile=((0.2, 0.22, 0.24), (0.35, 0.37, 0.38)),
        carpet=((0.28, 0.28, 0.32), (0.35, 0.22, 0.22)),
        fabric=((0.2, 0.3, 0.35), (0.45, 0.2, 0.22), (0.15, 0.15, 0.17)),
        accent=((0.85, 0.65, 0.25), (0.7, 0.3, 0.2), (0.3, 0.6, 0.55)),
        cabinet=((0.14, 0.15, 0.17), (0.25, 0.32, 0.3)),
        stone=((0.12, 0.12, 0.13), (0.7, 0.7, 0.7)),
    ),
    Palette(
        "coastal",
        walls=((0.88, 0.93, 0.95), (0.75, 0.86, 0.9), (0.96, 0.95, 0.9)),
        ceiling=(0.97, 0.97, 0.96),
        trim=(0.98, 0.98, 0.98),
        wood=((0.72, 0.62, 0.5), (0.6, 0.52, 0.45)),
        tile=((0.55, 0.72, 0.78), (0.85, 0.88, 0.88)),
        carpet=((0.8, 0.78, 0.7), (0.6, 0.72, 0.78)),
        fabric=((0.25, 0.4, 0.6), (0.9, 0.9, 0.88), (0.5, 0.65, 0.75)),
        accent=((0.9, 0.5, 0.35), (0.2, 0.5, 0.65), (0.95, 0.85, 0.5)),
        cabinet=((0.9, 0.92, 0.92), (0.3, 0.45, 0.6)),
        stone=((0.88, 0.88, 0.86), (0.4, 0.42, 0.45)),
    ),
    Palette(
        "retro",
        walls=((0.9, 0.78, 0.4), (0.5, 0.7, 0.65), (0.92, 0.85, 0.75)),
        ceiling=(0.95, 0.93, 0.88),
        trim=(0.95, 0.9, 0.8),
        wood=((0.6, 0.38, 0.2), (0.5, 0.33, 0.2)),
        tile=((0.3, 0.55, 0.5), (0.85, 0.5, 0.3)),
        carpet=((0.75, 0.4, 0.25), (0.35, 0.5, 0.55)),
        fabric=((0.85, 0.5, 0.2), (0.3, 0.55, 0.5), (0.7, 0.25, 0.25)),
        accent=((0.95, 0.75, 0.2), (0.25, 0.5, 0.6), (0.8, 0.3, 0.3)),
        cabinet=((0.3, 0.55, 0.5), (0.9, 0.82, 0.6)),
        stone=((0.9, 0.85, 0.75), (0.35, 0.3, 0.3)),
    ),
)

FIXED_ROLES: dict[str, RGB] = {
    "white": (0.92, 0.92, 0.92),
    "metal": (0.62, 0.63, 0.66),
    "dark": (0.1, 0.1, 0.11),
    "screen": (0.03, 0.04, 0.06),
    "glass": (0.7, 0.85, 0.92),
    "mirror": (0.75, 0.82, 0.86),
    "water": (0.55, 0.78, 0.9),
    "pot": (0.7, 0.42, 0.3),
}


def furniture_colors(rng: random.Random, pal: Palette) -> dict[str, RGB]:
    """Resolve the colour roles used by asset parts for one furniture instance."""
    wood = jitter(rng, rng.choice(pal.wood), 0.1)
    fabrics = rng.sample(pal.fabric, 2)
    accents = rng.sample(pal.accent, 3)
    roles = dict(FIXED_ROLES)
    roles.update(
        wood=wood,
        wood_dark=shade(wood, 0.7),
        fabric=jitter(rng, fabrics[0], 0.06),
        fabric2=jitter(rng, fabrics[1], 0.06),
        accent=accents[0],
        accent2=accents[1],
        accent3=accents[2],
        cabinet=jitter(rng, rng.choice(pal.cabinet), 0.05),
        stone=rng.choice(pal.stone),
        tile=rng.choice(pal.tile),
        green=(rng.uniform(0.1, 0.25), rng.uniform(0.4, 0.55), rng.uniform(0.12, 0.25)),
    )
    return roles
