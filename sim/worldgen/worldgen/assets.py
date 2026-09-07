"""Furniture library as data: nominal dimensions plus primitive parts.

Conventions for every asset: local origin at the centre of the footprint on the floor,
x = depth with the front facing +x (the back is -x), y = width, z up. Parts are boxes
(size = x, y, z), vertical cylinders (size = radius, length) or spheres (size = radius).
Instances are scaled per axis from the nominal size, so legs and handles scale slightly too.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .geometry import Point, Rect, rotate


@dataclass(frozen=True)
class Part:
    shape: str  # "box" | "cyl" | "sphere"
    pos: tuple[float, float, float]
    size: tuple[float, ...]
    role: str
    collide: bool = True
    alpha: float = 1.0


@dataclass(frozen=True)
class Surface:
    name: str
    height: float
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass(frozen=True)
class Asset:
    cls: str
    depth: float
    width: float
    height: float
    parts: tuple[Part, ...]
    surfaces: tuple[Surface, ...] = ()
    width_range: tuple[float, float] | None = None
    depth_range: tuple[float, float] | None = None
    walkable: bool = False  # rugs: no collision, may overlap other furniture


def box(
    x: float, y: float, z: float, sx: float, sy: float, sz: float, role: str, **kw: object
) -> Part:
    return Part("box", (x, y, z), (sx, sy, sz), role, **kw)  # type: ignore[arg-type]


def cyl(x: float, y: float, z: float, r: float, length: float, role: str) -> Part:
    return Part("cyl", (x, y, z), (r, length), role)


def sphere(x: float, y: float, z: float, r: float, role: str) -> Part:
    return Part("sphere", (x, y, z), (r,), role)


def _legs(dx: float, dy: float, h: float, t: float, role: str) -> tuple[Part, ...]:
    return tuple(box(sx * dx, sy * dy, h / 2, t, t, h, role) for sx in (-1, 1) for sy in (-1, 1))


def _books() -> tuple[Part, ...]:
    out = []
    spec = [(0.03, "accent"), (0.515, "accent2"), (0.965, "accent3"), (1.415, "accent")]
    for tier, (z0, role) in enumerate(spec):
        y = -0.4
        for k in range(4):
            bw = 0.04 + 0.01 * ((tier + k) % 3)
            bh = 0.2 + 0.03 * ((tier * 2 + k) % 3)
            if (k + tier) % 4 != 3:
                out.append(box(0.0, y + bw / 2, z0 + bh / 2 + 0.015, 0.22, bw, bh, role))
            y += bw + 0.05 + 0.04 * (k % 2)
    return tuple(out)


ASSETS: dict[str, Asset] = {
    a.cls: a
    for a in (
        Asset(
            "kitchen_counter",
            0.6,
            2.0,
            0.94,
            parts=(
                box(0, 0, 0.43, 0.56, 2.0, 0.86, "cabinet"),
                box(0, 0, 0.9, 0.6, 2.0, 0.04, "stone"),
                box(0.29, -0.6, 0.7, 0.02, 0.25, 0.02, "metal"),
                box(0.29, 0.0, 0.7, 0.02, 0.25, 0.02, "metal"),
                box(0.29, 0.6, 0.7, 0.02, 0.25, 0.02, "metal"),
                box(0.0, 0.4, 0.925, 0.45, 0.5, 0.01, "dark", collide=False),
            ),
            surfaces=(Surface("top", 0.92, -0.3, -1.0, 0.3, 1.0),),
            width_range=(1.2, 2.6),
        ),
        Asset(
            "fridge",
            0.65,
            0.65,
            1.8,
            parts=(
                box(0, 0, 0.9, 0.65, 0.65, 1.8, "white"),
                box(0.31, 0.22, 1.25, 0.02, 0.03, 0.4, "metal"),
                box(0.31, 0.22, 0.55, 0.02, 0.03, 0.4, "metal"),
            ),
            width_range=(0.6, 0.7),
        ),
        Asset(
            "oven",
            0.6,
            0.6,
            0.92,
            parts=(
                box(0, 0, 0.43, 0.6, 0.6, 0.86, "metal"),
                box(0, 0, 0.9, 0.6, 0.6, 0.04, "dark"),
                box(0.295, 0, 0.4, 0.01, 0.45, 0.3, "dark", collide=False),
            ),
            surfaces=(Surface("top", 0.92, -0.3, -0.3, 0.3, 0.3),),
        ),
        Asset(
            "dining_table",
            0.9,
            1.6,
            0.76,
            parts=(box(0, 0, 0.74, 0.9, 1.6, 0.04, "wood"),)
            + _legs(0.41, 0.74, 0.72, 0.06, "wood_dark"),
            surfaces=(Surface("top", 0.76, -0.45, -0.8, 0.45, 0.8),),
            width_range=(1.2, 2.0),
        ),
        Asset(
            "chair",
            0.45,
            0.45,
            0.9,
            parts=(
                box(0, 0, 0.45, 0.45, 0.45, 0.04, "wood"),
                box(-0.205, 0, 0.69, 0.04, 0.42, 0.42, "wood"),
            )
            + _legs(0.19, 0.19, 0.43, 0.04, "wood_dark"),
        ),
        Asset(
            "sofa",
            0.95,
            2.0,
            0.85,
            parts=(
                box(0, 0, 0.15, 0.95, 1.64, 0.3, "fabric"),
                box(0, -0.91, 0.31, 0.95, 0.18, 0.62, "fabric"),
                box(0, 0.91, 0.31, 0.95, 0.18, 0.62, "fabric"),
                box(-0.365, 0, 0.6, 0.22, 1.64, 0.5, "fabric"),
                box(0.1, 0, 0.38, 0.7, 1.6, 0.16, "fabric2"),
                box(-0.2, -0.5, 0.62, 0.12, 0.4, 0.35, "accent"),
                box(-0.2, 0.5, 0.62, 0.12, 0.4, 0.35, "accent2"),
            ),
            width_range=(1.6, 2.4),
        ),
        Asset(
            "coffee_table",
            0.6,
            1.0,
            0.42,
            parts=(box(0, 0, 0.4, 0.6, 1.0, 0.04, "wood"),)
            + _legs(0.25, 0.45, 0.38, 0.05, "wood_dark"),
            surfaces=(Surface("top", 0.42, -0.3, -0.5, 0.3, 0.5),),
            width_range=(0.8, 1.2),
        ),
        Asset(
            "tv_unit",
            0.4,
            1.5,
            1.0,
            parts=(
                box(0, 0, 0.225, 0.4, 1.5, 0.45, "wood"),
                box(-0.1, 0, 0.725, 0.05, 1.0, 0.55, "dark"),
                box(-0.1, 0, 0.465, 0.15, 0.3, 0.03, "dark"),
                box(-0.072, 0, 0.725, 0.005, 0.94, 0.5, "screen", collide=False),
            ),
            width_range=(1.2, 1.9),
        ),
        Asset(
            "bed",
            2.05,
            1.6,
            0.95,
            parts=(
                box(0, 0, 0.125, 2.05, 1.6, 0.25, "wood_dark"),
                box(0.05, 0, 0.35, 1.95, 1.5, 0.2, "white"),
                box(0.3, 0, 0.455, 1.3, 1.52, 0.05, "fabric"),
                box(-0.7, -0.38, 0.51, 0.35, 0.55, 0.12, "fabric2"),
                box(-0.7, 0.38, 0.51, 0.35, 0.55, 0.12, "fabric2"),
                box(-0.985, 0, 0.475, 0.08, 1.6, 0.95, "wood"),
            ),
            surfaces=(Surface("top", 0.48, -0.9, -0.75, 1.0, 0.75),),
            width_range=(0.9, 1.8),
        ),
        Asset(
            "nightstand",
            0.4,
            0.45,
            0.5,
            parts=(
                box(0, 0, 0.25, 0.4, 0.45, 0.5, "wood"),
                box(0.19, 0, 0.38, 0.02, 0.06, 0.02, "metal"),
            ),
            surfaces=(Surface("top", 0.5, -0.2, -0.225, 0.2, 0.225),),
        ),
        Asset(
            "wardrobe",
            0.6,
            1.6,
            2.0,
            parts=(
                box(0, 0, 1.0, 0.6, 1.6, 2.0, "cabinet"),
                box(0.298, 0, 1.0, 0.004, 0.01, 1.9, "dark", collide=False),
                box(0.29, -0.06, 1.0, 0.02, 0.03, 0.3, "metal"),
                box(0.29, 0.06, 1.0, 0.02, 0.03, 0.3, "metal"),
            ),
            width_range=(0.9, 2.2),
        ),
        Asset(
            "desk",
            0.7,
            1.4,
            1.12,
            parts=(
                box(0, 0, 0.735, 0.7, 1.4, 0.03, "wood"),
                box(0.0, -0.675, 0.36, 0.65, 0.03, 0.72, "wood_dark"),
                box(0.0, 0.675, 0.36, 0.65, 0.03, 0.72, "wood_dark"),
                box(-0.3, 0, 0.5, 0.02, 1.3, 0.4, "wood_dark"),
                box(-0.2, 0, 0.795, 0.1, 0.12, 0.09, "dark"),
                box(-0.22, 0, 0.95, 0.025, 0.55, 0.32, "dark", collide=False),
                box(0.05, 0, 0.7575, 0.15, 0.4, 0.015, "dark", collide=False),
            ),
            surfaces=(Surface("top", 0.75, -0.35, -0.7, 0.35, 0.7),),
            width_range=(1.0, 1.8),
        ),
        Asset(
            "office_chair",
            0.55,
            0.55,
            1.0,
            parts=(
                cyl(0, 0, 0.02, 0.25, 0.03, "dark"),
                cyl(0, 0, 0.2, 0.03, 0.3, "dark"),
                box(0, 0, 0.46, 0.5, 0.5, 0.08, "fabric"),
                box(-0.22, 0, 0.74, 0.08, 0.46, 0.5, "fabric"),
            ),
        ),
        Asset(
            "bathtub",
            0.75,
            1.7,
            0.66,
            parts=(
                box(0, 0, 0.275, 0.75, 1.7, 0.55, "white"),
                box(0, 0, 0.55, 0.62, 1.55, 0.02, "water", collide=False),
                box(-0.33, 0.7, 0.6, 0.04, 0.06, 0.1, "metal"),
            ),
            width_range=(1.5, 1.8),
        ),
        Asset(
            "shower",
            0.9,
            0.9,
            2.0,
            parts=(
                box(0, 0, 0.04, 0.9, 0.9, 0.08, "white"),
                box(-0.435, 0, 1.0, 0.03, 0.9, 2.0, "tile"),
                box(0.445, 0, 1.03, 0.01, 0.9, 1.9, "glass", alpha=0.25),
                box(-0.01, 0.445, 1.03, 0.88, 0.01, 1.9, "glass", alpha=0.25),
            ),
        ),
        Asset(
            "toilet",
            0.7,
            0.4,
            0.78,
            parts=(
                box(0.1, 0, 0.2, 0.5, 0.38, 0.4, "white"),
                box(-0.25, 0, 0.58, 0.2, 0.38, 0.4, "white"),
                box(0.1, 0, 0.41, 0.3, 0.34, 0.02, "white"),
            ),
        ),
        Asset(
            "sink",
            0.45,
            0.6,
            1.6,
            parts=(
                box(0, 0, 0.4, 0.45, 0.6, 0.8, "cabinet"),
                box(0, 0, 0.815, 0.45, 0.6, 0.03, "white"),
                box(-0.215, 0, 1.3, 0.02, 0.5, 0.6, "mirror", collide=False),
                box(-0.15, 0, 0.88, 0.04, 0.04, 0.1, "metal"),
            ),
            surfaces=(Surface("top", 0.83, -0.225, -0.3, 0.225, 0.3),),
            width_range=(0.5, 0.9),
        ),
        Asset(
            "shelves",
            0.32,
            1.0,
            1.8,
            parts=(
                box(0, -0.485, 0.9, 0.32, 0.03, 1.8, "wood"),
                box(0, 0.485, 0.9, 0.32, 0.03, 1.8, "wood"),
                box(-0.155, 0, 0.9, 0.01, 0.94, 1.8, "wood_dark"),
            )
            + tuple(box(0, 0, z, 0.32, 0.94, 0.03, "wood") for z in (0.015, 0.5, 0.95, 1.4, 1.785))
            + _books(),
            surfaces=tuple(
                Surface(f"shelf_{i}", z, -0.16, -0.47, 0.16, 0.47)
                for i, z in enumerate((0.515, 0.965, 1.415), start=1)
            ),
            width_range=(0.6, 1.4),
        ),
        Asset(
            "plant",
            0.45,
            0.45,
            1.25,
            parts=(
                cyl(0, 0, 0.15, 0.18, 0.3, "pot"),
                cyl(0, 0, 0.55, 0.025, 0.5, "wood_dark"),
                sphere(0, 0, 0.9, 0.22, "green"),
                sphere(0.03, 0.03, 1.03, 0.18, "green"),
            ),
            width_range=(0.4, 0.5),
        ),
        Asset(
            "rug",
            2.0,
            1.4,
            0.012,
            parts=(
                box(0, 0, 0.005, 2.0, 1.4, 0.01, "fabric", collide=False),
                box(0, 0, 0.006, 1.6, 1.0, 0.011, "fabric2", collide=False),
                box(0, -0.45, 0.006, 2.0, 0.1, 0.012, "accent", collide=False),
                box(0, 0.45, 0.006, 2.0, 0.1, 0.012, "accent", collide=False),
            ),
            width_range=(1.2, 2.2),
            depth_range=(1.6, 2.6),
            walkable=True,
        ),
    )
}


@dataclass(frozen=True)
class Instance:
    """One sized copy of an asset: scale factors per axis and resolved colours."""

    asset: Asset
    sx: float
    sy: float
    sz: float
    colors: dict[str, tuple[float, float, float]] = field(compare=False)

    @property
    def depth(self) -> float:
        return self.asset.depth * self.sx

    @property
    def width(self) -> float:
        return self.asset.width * self.sy

    @property
    def height(self) -> float:
        return self.asset.height * self.sz

    def extent(self, yaw: float) -> Point:
        """Axis-aligned footprint size (x, y) for a yaw that is a multiple of 90 degrees."""
        swap = round(yaw / (math.pi / 2)) % 2 == 1
        return (self.width, self.depth) if swap else (self.depth, self.width)

    def footprint(self, x: float, y: float, yaw: float) -> Rect:
        ex, ey = self.extent(yaw)
        return Rect(x - ex / 2, y - ey / 2, x + ex / 2, y + ey / 2)

    def scaled_parts(self) -> list[tuple[Part, tuple[float, float, float], tuple[float, ...]]]:
        radial = min(self.sx, self.sy)
        out = []
        for p in self.asset.parts:
            pos = (p.pos[0] * self.sx, p.pos[1] * self.sy, p.pos[2] * self.sz)
            if p.shape == "box":
                size = (p.size[0] * self.sx, p.size[1] * self.sy, p.size[2] * self.sz)
            elif p.shape == "cyl":
                size = (p.size[0] * radial, p.size[1] * self.sz)
            else:
                size = (p.size[0] * radial,)
            out.append((p, pos, size))
        return out

    def surface_polygons(self, x: float, y: float, yaw: float) -> list[dict[str, object]]:
        res = []
        for s in self.asset.surfaces:
            pts = []
            for lx, ly in ((s.x0, s.y0), (s.x1, s.y0), (s.x1, s.y1), (s.x0, s.y1)):
                rx, ry = rotate((lx * self.sx, ly * self.sy), yaw)
                pts.append([round(x + rx, 3), round(y + ry, 3)])
            res.append({"name": s.name, "height": round(s.height * self.sz, 3), "polygon": pts})
        return res


def instantiate(
    rng: random.Random,
    cls: str,
    colors: dict[str, tuple[float, float, float]],
    width: float | None = None,
) -> Instance:
    a = ASSETS[cls]
    var = 0.05
    if width is None:
        lo, hi = a.width_range or (a.width * (1 - var), a.width * (1 + var))
        width = rng.uniform(lo, hi)
    d_lo, d_hi = a.depth_range or (a.depth * (1 - var), a.depth * (1 + var))
    depth = rng.uniform(d_lo, d_hi)
    sz = 1 + rng.uniform(-var, var) if a.height > 0.1 else 1.0
    return Instance(a, round(depth / a.depth, 4), round(width / a.width, 4), round(sz, 4), colors)
