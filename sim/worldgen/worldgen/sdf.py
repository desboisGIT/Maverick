"""SDF writer: turns the generated flat into a Gazebo Harmonic world file."""

from __future__ import annotations

from dataclasses import dataclass

from .decor import Lamp, Panel
from .floorplan import FloorPlan
from .floors import RoomFloor
from .furniture import Placed
from .materials import RGB, Palette, shade
from .openings import DOOR_HEIGHT, Opening
from .walls import THICKNESS, WallSegment

# Models that are not furniture; every other model in the world is a manifest furniture id.
STRUCTURE_MODELS = ("ground_plane", "floor", "walls", "ceiling", "decor")
SKIRTING_H = 0.08


@dataclass(frozen=True)
class Lighting:
    ambient: float
    sun: float
    sun_color: RGB
    lamp: float
    lamp_color: RGB
    window: RGB


LIGHTING: dict[str, Lighting] = {
    "day": Lighting(0.5, 0.8, (1.0, 0.98, 0.92), 0.25, (1.0, 0.97, 0.9), (0.8, 0.9, 1.0)),
    "evening": Lighting(0.22, 0.15, (1.0, 0.65, 0.4), 0.9, (1.0, 0.78, 0.5), (0.12, 0.15, 0.28)),
    "lamp_only": Lighting(0.1, 0.0, (1.0, 1.0, 1.0), 0.9, (1.0, 0.9, 0.72), (0.03, 0.04, 0.08)),
}


def n(v: float) -> str:
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("", "-0") else s


def rgb(c: RGB) -> str:
    return f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f}"


def _material(c: RGB, emissive: RGB | None = None) -> str:
    e = f"<emissive>{rgb(emissive)} 1</emissive>" if emissive else ""
    return f"<material><ambient>{rgb(c)} 1</ambient><diffuse>{rgb(c)} 1</diffuse>{e}</material>"


def _geom(shape: str, size: tuple[float, ...]) -> str:
    if shape == "box":
        return f"<box><size>{n(size[0])} {n(size[1])} {n(size[2])}</size></box>"
    if shape == "cyl":
        return f"<cylinder><radius>{n(size[0])}</radius><length>{n(size[1])}</length></cylinder>"
    return f"<sphere><radius>{n(size[0])}</radius></sphere>"


def visual(
    name: str,
    pos: tuple[float, float, float],
    shape: str,
    size: tuple[float, ...],
    color: RGB,
    alpha: float = 1.0,
    emissive: RGB | None = None,
) -> str:
    t = "" if alpha >= 1 else f"<transparency>{n(1 - alpha)}</transparency>"
    return (
        f'<visual name="{name}"><pose>{n(pos[0])} {n(pos[1])} {n(pos[2])} 0 0 0</pose>'
        f"<geometry>{_geom(shape, size)}</geometry>{t}{_material(color, emissive)}</visual>"
    )


def collision(
    name: str, pos: tuple[float, float, float], shape: str, size: tuple[float, ...]
) -> str:
    return (
        f'<collision name="{name}"><pose>{n(pos[0])} {n(pos[1])} {n(pos[2])} 0 0 0</pose>'
        f"<geometry>{_geom(shape, size)}</geometry></collision>"
    )


def model(name: str, body: list[str], pose: str = "0 0 0 0 0 0") -> str:
    inner = "\n      ".join(body)
    return (
        f'  <model name="{name}"><static>true</static><pose>{pose}</pose>\n'
        f'    <link name="l">\n      {inner}\n    </link>\n  </model>'
    )


def _floor_model(floors: list[RoomFloor]) -> str:
    body = []
    for f in floors:
        r = f.rect
        cx, cy = r.center
        body.append(visual(f"{f.room}_base", (cx, cy, 0.001), "box", (r.w, r.h, 0.002), f.grout))
        for i, t in enumerate(f.tiles):
            body.append(
                visual(f"{f.room}_{i}", (t.cx, t.cy, 0.002), "box", (t.sx, t.sy, 0.004), t.color)
            )
        if not f.tiles:
            body.append(visual(f"{f.room}_flat", (cx, cy, 0.002), "box", (r.w, r.h, 0.004), f.base))
    return model("floor", body)


def _wall_model(
    walls: list[WallSegment],
    openings: list[Opening],
    wall_colors: dict[str, RGB],
    exterior: RGB,
    trim: RGB,
    height: float,
) -> str:
    body = []
    half = THICKNESS / 2
    for i, s in enumerate(walls):
        mid = (s.start + s.end) / 2
        for tag, room, sign in (("a", s.low, -1), ("b", s.high, 1)):
            color = wall_colors[room] if room else exterior
            off = sign * THICKNESS / 4
            if s.orient == "v":
                pos, size = (s.pos + off, mid, height / 2), (half, s.length, height)
            else:
                pos, size = (mid, s.pos + off, height / 2), (s.length, half, height)
            body.append(visual(f"w{i}{tag}", pos, "box", size, color))
            if room:
                skirt = sign * (half + 0.006)
                if s.orient == "v":
                    spos, ssize = (
                        (s.pos + skirt, mid, SKIRTING_H / 2),
                        (0.012, s.length, SKIRTING_H),
                    )
                else:
                    spos, ssize = (
                        (mid, s.pos + skirt, SKIRTING_H / 2),
                        (s.length, 0.012, SKIRTING_H),
                    )
                body.append(visual(f"s{i}{tag}", spos, "box", ssize, trim))
        cpos = (s.pos, mid, height / 2) if s.orient == "v" else (mid, s.pos, height / 2)
        csize = (THICKNESS, s.length, height) if s.orient == "v" else (s.length, THICKNESS, height)
        body.append(collision(f"c{i}", cpos, "box", csize))
    top = height - DOOR_HEIGHT
    for o in openings:
        if o.kind != "door":
            continue
        mid, ln = (o.start + o.end) / 2, o.end - o.start
        z = DOOR_HEIGHT + top / 2
        pos = (o.pos, mid, z) if o.orient == "v" else (mid, o.pos, z)
        size = (THICKNESS, ln, top) if o.orient == "v" else (ln, THICKNESS, top)
        color = wall_colors[o.rooms[0]]
        body.append(visual(f"{o.id}_lintel", pos, "box", size, color))
        body.append(collision(f"{o.id}_lintel_c", pos, "box", size))
    return model("walls", body)


def _ceiling_model(
    plan: FloorPlan, height: float, color: RGB, lamps: list[Lamp], lamp_color: RGB
) -> str:
    body = []
    for i, w in enumerate(plan.wings):
        w = w.inset(-THICKNESS / 2)
        cx, cy = w.center
        pos, size = (cx, cy, height + 0.05), (w.w, w.h, 0.1)
        body.append(visual(f"slab{i}", pos, "box", size, color))
        body.append(collision(f"slab{i}_c", pos, "box", size))
    for i, lamp in enumerate(lamps):
        body.append(
            visual(
                f"lamp{i}",
                (lamp.x, lamp.y, height - 0.02),
                "cyl",
                (0.15, 0.04),
                (0.95, 0.95, 0.9),
                emissive=lamp_color,
            )
        )
    return model("ceiling", body)


def _panel_box(p: Panel, along: float, z: float, sw: float, sh: float, lift: float):
    c = p.face + p.inward * (0.006 + lift)
    if p.orient == "v":
        return (c, along, z), (0.012, sw, sh)
    return (along, c, z), (sw, 0.012, sh)


def _decor_model(panels: list[Panel], trim: RGB, window_glow: RGB) -> str:
    body = []
    for i, p in enumerate(panels):
        window = p.kind == "window"
        frame = 0.045 if window else 0.02
        pos, size = _panel_box(p, p.u, p.z, p.w + 2 * frame, p.h + 2 * frame, 0.0)
        body.append(visual(f"p{i}f", pos, "box", size, trim if window else (0.15, 0.12, 0.1)))
        pos, size = _panel_box(p, p.u, p.z, p.w, p.h, 0.004)
        body.append(
            visual(f"p{i}g", pos, "box", size, p.color, emissive=window_glow if window else None)
        )
    return model("decor", body)


def _furniture_model(p: Placed) -> str:
    body = []
    for i, (part, pos, size) in enumerate(p.inst.scaled_parts()):
        color = p.inst.colors[part.role]
        body.append(visual(f"v{i}", pos, part.shape, size, color, part.alpha))
        if part.collide:
            body.append(collision(f"c{i}", pos, part.shape, size))
    return model(p.id, body, pose=f"{n(p.x)} {n(p.y)} 0 0 0 {n(p.yaw)}")


def _lights(lamps: list[Lamp], height: float, light: Lighting) -> list[str]:
    out = []
    for i, lamp in enumerate(lamps):
        out.append(
            f'  <light name="lamp{i}" type="point">'
            f"<pose>{n(lamp.x)} {n(lamp.y)} {n(height - 0.2)} 0 0 0</pose>"
            f"<cast_shadows>false</cast_shadows><diffuse>{rgb(light.lamp_color)} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular><intensity>{n(light.lamp)}</intensity>"
            "<attenuation><range>9</range><constant>0.4</constant><linear>0.1</linear>"
            "<quadratic>0.02</quadratic></attenuation></light>"
        )
    return out


GROUND_PLANE = """  <model name="ground_plane">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
        <surface><friction><ode/></friction><bounce/><contact/></surface>
      </collision>
      <visual name="visual">
        <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
    </link>
  </model>"""

# World-level settings copied from sim/worlds/textured.sdf, which PX4 SITL is known to run in.
# That file has no world-level <plugin> elements: the systems come from PX4's server config.
HEADER = """<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
<world name="{name}">
  <physics type="ode">
    <max_step_size>0.004</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>250</real_time_update_rate>
  </physics>
  <gravity>0 0 -9.8</gravity>
  <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
  <atmosphere type="adiabatic"/>
  <scene>
    <grid>false</grid>
    <ambient>{ambient} {ambient} {ambient} 1</ambient>
    <background>0.7 0.7 0.7 1</background>
    <shadows>true</shadows>
  </scene>
"""

SUN = """  <light name="sunUTC" type="directional">
    <pose>0 0 500 0 0 0</pose>
    <cast_shadows>false</cast_shadows>
    <intensity>{intensity}</intensity>
    <direction>0.001 0.625 -0.78</direction>
    <diffuse>{color} 1</diffuse>
    <specular>0.1 0.1 0.1 1</specular>
    <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
  </light>
"""

FOOTER = """  <spherical_coordinates>
    <surface_model>EARTH_WGS84</surface_model>
    <world_frame_orientation>ENU</world_frame_orientation>
    <latitude_deg>47.397971057728974</latitude_deg>
    <longitude_deg>8.546163739800146</longitude_deg>
    <elevation>0</elevation>
  </spherical_coordinates>
</world>
</sdf>
"""


def write_sdf(
    name: str,
    plan: FloorPlan,
    openings: list[Opening],
    floors: list[RoomFloor],
    walls: list[WallSegment],
    wall_colors: dict[str, RGB],
    panels: list[Panel],
    lamps: list[Lamp],
    furniture: list[Placed],
    palette: Palette,
    ceiling_height: float,
    lighting: str,
    ceiling: bool,
) -> str:
    light = LIGHTING[lighting]
    parts = [HEADER.format(name=name, ambient=n(light.ambient)), GROUND_PLANE]
    parts.append(SUN.format(intensity=n(light.sun), color=rgb(light.sun_color)))
    parts += _lights(lamps, ceiling_height, light)
    parts.append(_floor_model(floors))
    parts.append(
        _wall_model(
            walls, openings, wall_colors, shade(palette.walls[0], 0.8), palette.trim, ceiling_height
        )
    )
    if ceiling:
        parts.append(_ceiling_model(plan, ceiling_height, palette.ceiling, lamps, light.lamp_color))
    parts.append(_decor_model(panels, palette.trim, light.window))
    parts += [_furniture_model(p) for p in furniture]
    parts.append(FOOTER)
    return "\n".join(parts)
