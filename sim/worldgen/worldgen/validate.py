"""Validity checks on a manifest (and optionally its SDF). Returns a list of problems."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import deque
from typing import Any

from .clearance import door_zones, entries_connected, entry_points
from .floorplan import Room
from .geometry import Rect, polygon_area, rect_in_polygon
from .openings import Opening
from .profile import PROFILES
from .programme import BEDROOMS, RULES
from .sdf import STRUCTURE_MODELS

WALL_HALF = 0.05


def _rooms(m: dict[str, Any]) -> list[Room]:
    return [Room(r["id"], r["type"], Rect.from_points(r["polygon"])) for r in m["rooms"]]


def _openings(m: dict[str, Any], rooms: dict[str, Room]) -> list[Opening]:
    out = []
    for d in m["doors"]:
        a, b = (rooms[i].rect for i in d["rooms"])
        cx, cy = d["center"]
        half = d["width"] / 2
        if abs(a.x1 - b.x0) < 1e-6 or abs(b.x1 - a.x0) < 1e-6:
            out.append(
                Opening(d["id"], tuple(d["rooms"]), d["kind"], "v", cx, cy - half, cy + half)
            )
        else:
            out.append(
                Opening(d["id"], tuple(d["rooms"]), d["kind"], "h", cy, cx - half, cx + half)
            )
    return out


def validate(m: dict[str, Any], sdf: str | None = None) -> list[str]:
    errs: list[str] = []
    prof = PROFILES[m["drone_profile"]]
    rooms = _rooms(m)
    by_id = {r.id: r for r in rooms}
    foot = m["footprint"]

    # rooms: inside the footprint, disjoint, filling it, sensible sizes
    for i, a in enumerate(rooms):
        if not rect_in_polygon(a.rect, foot):
            errs.append(f"{a.id} outside footprint")
        for b in rooms[i + 1 :]:
            if a.rect.intersects(b.rect):
                errs.append(f"{a.id} overlaps {b.id}")
        min_side, (_, min_area, _) = prof.min_sides[a.type], RULES[a.type]
        if min(a.rect.w, a.rect.h) < min_side - 1e-6:
            errs.append(f"{a.id} side below {min_side}")
        if a.rect.area < min_area:
            errs.append(f"{a.id} area below {min_area}")
    total = sum(r.rect.area for r in rooms)
    if abs(total - polygon_area(foot)) > 0.01 * polygon_area(foot):
        errs.append("room areas do not fill the footprint")
    if not 25 <= total <= 115:
        errs.append(f"flat area {total:.1f} m2 out of range")
    if not 2.4 <= m["ceiling_height"] <= 2.7:
        errs.append("ceiling height out of range")
    types = [r.type for r in rooms]
    for need in ("kitchen", "living", "bathroom"):
        if types.count(need) != 1:
            errs.append(f"expected one {need}")
    if types.count("bedroom") != BEDROOMS[m["flat_type"]]:
        errs.append("bedroom count does not match flat type")

    # doors
    if len(by_id) != len(rooms):
        errs.append("duplicate room ids")
    graph: dict[str, list[str]] = {r.id: [] for r in rooms}
    for d in m["doors"]:
        a, b = d["rooms"]
        if (
            d["kind"] == "door"
            and not prof.door_range[0] - 1e-6 <= d["width"] <= prof.door_range[1] + 1e-6
        ):
            errs.append(f"{d['id']} width {d['width']}")
        if d["kind"] == "open" and d["width"] < prof.open_min_dm / 10 - 1e-6:
            errs.append(f"{d['id']} open gap too narrow")
        if d["kind"] == "open" and {by_id[a].type, by_id[b].type} != {"kitchen", "living"}:
            errs.append(f"{d['id']} open gap not between kitchen and living")
        if not _touching(by_id[a].rect, by_id[b].rect):
            errs.append(f"{d['id']} rooms do not share a wall")
        graph[a].append(b)
        graph[b].append(a)
    spawn = m["spawn"]
    seen = {spawn["room"]}
    queue = deque([spawn["room"]])
    while queue:
        for nxt in graph[queue.popleft()]:
            if nxt not in seen:
                seen.add(nxt)
                queue.append(nxt)
    for r in rooms:
        if r.id not in seen:
            errs.append(f"{r.id} unreachable from spawn room")

    # furniture
    ops = _openings(m, by_id)
    items = m["furniture"]
    ids = [f["id"] for f in items]
    if len(set(ids)) != len(ids):
        errs.append("duplicate furniture ids")
    rects = {}
    solid = []
    for f in items:
        room = by_id[f["room"]]
        (x0, y0, z0), (x1, y1, z1) = f["bbox"]
        r = Rect(x0, y0, x1, y1)
        rects[f["id"]] = r
        if not room.rect.inset(WALL_HALF).contains(r, 1e-3):
            errs.append(f"{f['id']} intersects a wall")
        if z1 > m["ceiling_height"] - 0.1:
            errs.append(f"{f['id']} touches the ceiling")
        for s in f["surfaces"]:
            if (
                not r.inset(-1e-3).contains(Rect.from_points(s["polygon"]))
                or s["height"] > z1 + 1e-3
            ):
                errs.append(f"{f['id']} surface {s['name']} outside bbox")
        if f["class"] != "rug":
            solid.append(f)
            if any(z.intersects(r) for z in door_zones(room, ops, prof)):
                errs.append(f"{f['id']} blocks a door")
    for i, a in enumerate(solid):
        for b in solid[i + 1 :]:
            if a["room"] == b["room"] and rects[a["id"]].intersects(rects[b["id"]], 1e-4):
                errs.append(f"{a['id']} overlaps {b['id']}")
    for room in rooms:
        obstacles = [rects[f["id"]] for f in solid if f["room"] == room.id]
        if not entries_connected(
            room.rect, obstacles, entry_points(room, ops, prof), prof.corridor_radius
        ):
            errs.append(f"{room.id} doors not connected by a {prof.corridor} m corridor")

    # spawn
    sx, sy, sz, _ = spawn["pose"]
    sroom = by_id[spawn["room"]]
    zone = Rect(
        sx - prof.spawn_clearance,
        sy - prof.spawn_clearance,
        sx + prof.spawn_clearance,
        sy + prof.spawn_clearance,
    )
    if sroom.type not in ("living", "hallway"):
        errs.append("spawn not in living room or hallway")
    if not sroom.rect.inset(WALL_HALF).contains(zone):
        errs.append("spawn closer than 1 m to a wall")
    for f in solid:
        if zone.intersects(rects[f["id"]]):
            errs.append(f"spawn within 1 m of {f['id']}")
    if sz < 0:
        errs.append("spawn below floor")

    if sdf is not None:
        errs += validate_sdf(sdf, m)
    return errs


def _touching(a: Rect, b: Rect) -> bool:
    return (
        abs(a.x1 - b.x0) < 1e-6
        or abs(b.x1 - a.x0) < 1e-6
        or abs(a.y1 - b.y0) < 1e-6
        or abs(b.y1 - a.y0) < 1e-6
    )


def validate_sdf(sdf: str, m: dict[str, Any]) -> list[str]:
    try:
        root = ET.fromstring(sdf)
    except ET.ParseError as e:
        return [f"SDF not well-formed: {e}"]
    world = root.find("world")
    errs = []
    if world is None or world.get("name") != m["world"]:
        errs.append("world name does not match the manifest")
    if world is not None:
        names = [mod.get("name") for mod in world.findall("model")]
        furniture = [n for n in names if n not in STRUCTURE_MODELS]
        if furniture != [f["id"] for f in m["furniture"]]:
            errs.append("SDF furniture models differ from the manifest")
        if len(set(names)) != len(names):
            errs.append("duplicate model names in SDF")
    return errs
