"""S4b extras: small objects (bananas, bread) on kitchen surfaces and a static person.

Drawn from their own random stream (`sub_rng(seed, "extras")`) so the apartment layout of a seed
is unchanged by them. Each small object class is present with probability `PRESENCE` (absent
30 % of the worlds). Everything here is ground truth for the referee only (manifest keys
`small_objects`, `presence`, `people`); the drone's stack never reads it.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any

from .clearance import door_zones
from .floorplan import FloorPlan, Room
from .furniture import Placed, Spawn
from .geometry import Rect
from .openings import Opening
from .profile import DroneProfile

PRESENCE = 0.7
SMALL_OBJECTS = ("banana", "bread")
PERSON_ROOMS = ("living", "bedroom", "office")
PERSON_HEIGHT = 1.75
FALLBACK_LEVELS = ((0.9, 0.8), (0.75, 0.65), (0.6, 0.5), (0.45, 0.4), (0.4, 0.3))
PERSON_SPAWN_MIN = 1.6  # m from the take-off point (come_to must have somewhere to fly)
EDGE = 0.08  # keep the object this far from the surface edge
MIN_SEPARATION = 0.4  # banana and bread


@dataclass(frozen=True)
class SmallObject:
    id: str
    cls: str
    room: str
    on: str  # furniture id
    x: float
    y: float
    z: float  # surface height
    yaw: float


@dataclass(frozen=True)
class Person:
    id: str
    room: str
    x: float
    y: float
    yaw: float  # faces this way (towards the room centre)

    @property
    def rect(self) -> Rect:
        return Rect(self.x - 0.25, self.y - 0.25, self.x + 0.25, self.y + 0.25)


def _surfaces(furniture: list[Placed], rooms: dict[str, Room]):
    """(furniture, surface polygon dict) of counters and tables in kitchen rooms."""
    out = []
    for f in furniture:
        if f.cls not in ("kitchen_counter", "dining_table") or rooms[f.room].type != "kitchen":
            continue
        for s in f.inst.surface_polygons(f.x, f.y, f.yaw):
            out.append((f, s))
    return out


def place_small_objects(
    rng: random.Random, plan: FloorPlan, furniture: list[Placed]
) -> list[SmallObject]:
    rooms = {r.id: r for r in plan.rooms}
    surfaces = _surfaces(furniture, rooms)
    placed: list[SmallObject] = []
    for cls in SMALL_OBJECTS:
        if rng.random() >= PRESENCE or not surfaces:  # the draw is made even without a surface
            continue
        for _ in range(40):
            f, s = rng.choice(surfaces)
            r = Rect.from_points(s["polygon"]).inset(EDGE)
            if r.w < 0.05 or r.h < 0.05:
                continue
            x, y = round(rng.uniform(r.x0, r.x1), 3), round(rng.uniform(r.y0, r.y1), 3)
            if any(math.hypot(x - o.x, y - o.y) < MIN_SEPARATION for o in placed):
                continue
            yaw = round(rng.uniform(-math.pi, math.pi), 3)
            placed.append(SmallObject(f"{cls}_0", cls, f.room, f.id, x, y, s["height"], yaw))
            break
    return placed


def place_person(
    rng: random.Random,
    plan: FloorPlan,
    openings: list[Opening],
    furniture: list[Placed],
    spawn: Spawn,
    prof: DroneProfile,
) -> Person | None:
    """A free standing spot in a living / bedroom / office, away from furniture, walls, doors
    and the take-off point, so that the drone can hover 1.5 m from it. When no such room has a
    spot (about 13 % of the tuning seeds 0-99 had no mannequin), the operator stands in
    a hallway or, last, the kitchen, with the clearances relaxed and the spawn distance
    lowered: a scenario that needs an operator always gets one. Seeds that had a spot before
    get the same spot (the first pass is unchanged)."""
    for rooms, levels, spawn_min in (
        (PERSON_ROOMS, ((0.9, 0.8), (0.75, 0.65), (0.6, 0.5), (0.45, 0.4)), PERSON_SPAWN_MIN),
        (PERSON_ROOMS, FALLBACK_LEVELS, 1.2),
        (PERSON_ROOMS + ("hallway",), FALLBACK_LEVELS, 1.2),
        (PERSON_ROOMS + ("hallway", "kitchen"), FALLBACK_LEVELS, 1.2),
    ):
        found = _person_spots(plan, openings, furniture, spawn, prof, rooms, levels, spawn_min)
        if found:
            room, x, y = rng.choice(found)
            cx, cy = room.rect.center
            return Person("person_0", room.id, x, y, round(math.atan2(cy - y, cx - x), 3))
    return None


def _person_spots(plan, openings, furniture, spawn, prof, rooms, levels, spawn_min):
    candidates: list[tuple[Room, float, float]] = []
    for room in plan.rooms:
        if room.type not in rooms:
            continue
        solid = [p.rect for p in furniture if p.room == room.id and p.solid]
        zones = door_zones(room, openings, prof)
        # strict clearance first, then relax; every level needs the wall margin
        for wall, furn in levels:
            area = room.rect.inset(wall + 0.05)
            if area.w < 0 or area.h < 0:
                continue
            spots = []
            for i in range(math.floor(area.w * 10 + 1e-6) + 1):
                for j in range(math.floor(area.h * 10 + 1e-6) + 1):
                    x, y = round(area.x0 + i / 10, 1), round(area.y0 + j / 10, 1)
                    body = Rect(x - 0.25, y - 0.25, x + 0.25, y + 0.25)
                    if any(body.inset(-furn).intersects(r) for r in solid):
                        continue
                    if any(body.inset(-0.3).intersects(z) for z in zones):
                        continue
                    if math.hypot(x - spawn.x, y - spawn.y) < spawn_min:
                        continue
                    spots.append((x, y))
            if spots:
                candidates += [(room, x, y) for x, y in spots]
                break
    return candidates


def extras_manifest(small: list[SmallObject], people: list[Person]) -> dict[str, Any]:
    return {
        "small_objects": [
            {
                "id": o.id,
                "class": o.cls,
                "room": o.room,
                "on": o.on,
                "position": [o.x, o.y, o.z],
                "yaw": o.yaw,
            }
            for o in small
        ],
        "presence": {c: any(o.cls == c for o in small) for c in SMALL_OBJECTS},
        "people": [
            {
                "id": p.id,
                "class": "person",
                "room": p.room,
                "pose": [p.x, p.y, 0.0, p.yaw],
                "bbox": [[p.x - 0.25, p.y - 0.25, 0.0], [p.x + 0.25, p.y + 0.25, PERSON_HEIGHT]],
            }
            for p in people
        ],
    }
