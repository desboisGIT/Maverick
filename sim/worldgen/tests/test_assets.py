import random

from worldgen.assets import ASSETS, instantiate
from worldgen.materials import PALETTES, furniture_colors

EPS = 1e-6


def _extent(part):
    if part.shape == "box":
        return part.size[0] / 2, part.size[1] / 2, part.size[2] / 2
    if part.shape == "cyl":
        return part.size[0], part.size[0], part.size[1] / 2
    return part.size[0], part.size[0], part.size[0]


def test_parts_lie_inside_nominal_box():
    for a in ASSETS.values():
        for p in a.parts:
            ex, ey, ez = _extent(p)
            assert abs(p.pos[0]) + ex <= a.depth / 2 + EPS, (a.cls, p)
            assert abs(p.pos[1]) + ey <= a.width / 2 + EPS, (a.cls, p)
            assert p.pos[2] - ez >= -EPS and p.pos[2] + ez <= a.height + EPS, (a.cls, p)


def test_surfaces_lie_on_the_asset():
    for a in ASSETS.values():
        for s in a.surfaces:
            assert -a.depth / 2 <= s.x0 < s.x1 <= a.depth / 2 + EPS
            assert -a.width / 2 <= s.y0 < s.y1 <= a.width / 2 + EPS
            assert 0 < s.height <= a.height + EPS


def test_roles_are_defined_in_every_palette():
    rng = random.Random(0)
    for pal in PALETTES:
        roles = furniture_colors(rng, pal)
        for a in ASSETS.values():
            assert {p.role for p in a.parts} <= roles.keys(), a.cls


def test_instances_scale_within_ranges_and_stay_below_ceiling():
    rng = random.Random(1)
    colors = furniture_colors(rng, PALETTES[0])
    for a in ASSETS.values():
        for _ in range(20):
            inst = instantiate(rng, a.cls, colors)
            assert inst.height < 2.3
            if a.width_range:
                assert a.width_range[0] - 1e-3 <= inst.width <= a.width_range[1] + 1e-3
