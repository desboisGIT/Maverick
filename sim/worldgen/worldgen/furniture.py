"""Furniture placement: per-room recipes on top of the asset library."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from .assets import ASSETS, Instance, instantiate
from .clearance import door_zones, entries_connected, entry_points
from .floorplan import FloorPlan, Room
from .geometry import Rect
from .materials import RGB, Palette, furniture_colors
from .openings import Opening
from .profile import DroneProfile

WALL_HALF = 0.05
MARGIN = 0.01  # extra clearance between wall faces and furniture
SIDES = ("W", "E", "S", "N")
OPPOSITE = {"W": "E", "E": "W", "S": "N", "N": "S"}
_SIDE_YAW = {"W": 0.0, "E": math.pi, "S": math.pi / 2, "N": -math.pi / 2}
_YAWS = (0.0, math.pi / 2, math.pi, -math.pi / 2)


class LayoutFailure(Exception):
    """A required piece did not fit; the caller retries with fresh random draws."""


@dataclass
class Placed:
    id: str
    cls: str
    room: str
    x: float
    y: float
    yaw: float
    inst: Instance
    rect: Rect
    side: str = ""

    @property
    def solid(self) -> bool:
        return not self.inst.asset.walkable


@dataclass(frozen=True)
class Spawn:
    room: str
    x: float
    y: float
    yaw: float


def wrap(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def inner_rect(room: Room) -> Rect:
    return room.rect.inset(WALL_HALF + MARGIN)


def find_spawn(
    rng: random.Random, plan: FloorPlan, placed: list[Placed], prof: DroneProfile
) -> Spawn | None:
    """A point with 1 m of clear floor around it, in the living room (or a wide hallway)."""
    pad = prof.spawn_clearance + 0.05
    options: list[tuple[Room, list[tuple[float, float]], float]] = []
    for room in plan.rooms:
        if room.type not in ("living", "hallway"):
            continue
        area = room.rect.inset(pad + WALL_HALF)
        if area.w < -1e-9 or area.h < -1e-9:
            continue
        solid = [p.rect for p in placed if p.room == room.id and p.solid]
        spots = []
        for i in range(math.floor(area.w * 10 + 1e-6) + 1):
            for j in range(math.floor(area.h * 10 + 1e-6) + 1):
                x, y = round(area.x0 + i / 10, 1), round(area.y0 + j / 10, 1)
                zone = Rect(x - pad, y - pad, x + pad, y + pad)
                if not any(zone.intersects(r) for r in solid):
                    spots.append((x, y))
        if spots:
            options.append((room, spots, 4.0 if room.type == "living" else 1.0))
    if not options:
        return None
    room, spots, _ = rng.choices(options, [o[2] for o in options])[0]
    x, y = rng.choice(spots)
    return Spawn(room.id, x, y, round(wrap(rng.randrange(8) * math.pi / 4), 4))


class Furnisher:
    def __init__(
        self,
        rng: random.Random,
        plan: FloorPlan,
        openings: list[Opening],
        palette: Palette,
        prof: DroneProfile,
    ) -> None:
        self.rng, self.plan, self.palette, self.prof = rng, plan, palette, prof
        self.placed: list[Placed] = []
        self.counts: dict[str, int] = {}
        self.zones = {r.id: door_zones(r, openings, prof) for r in plan.rooms}
        self.entries = {r.id: entry_points(r, openings, prof) for r in plan.rooms}
        self.colors: dict[str, dict[str, RGB]] = {}

    # ---- primitives -------------------------------------------------------------------------

    def _inst(self, room: Room, cls: str, width: float | None = None) -> Instance:
        if room.id not in self.colors:
            self.colors[room.id] = furniture_colors(self.rng, self.palette)
        return instantiate(self.rng, cls, self.colors[room.id], width)

    def _free(self, room: Room, rect: Rect, solid: bool, gap: float = 0.05) -> bool:
        if not inner_rect(room).contains(rect, 2e-3):
            return False
        if not solid:
            return True
        if any(z.intersects(rect) for z in self.zones[room.id]):
            return False
        others = [p.rect for p in self.placed if p.room == room.id and p.solid]
        if any(o.inset(-gap).intersects(rect) for o in others):
            return False
        return entries_connected(
            room.rect, others + [rect], self.entries[room.id], self.prof.corridor_radius
        )

    def _add(
        self, room: Room, inst: Instance, x: float, y: float, yaw: float, side: str = ""
    ) -> Placed:
        cls = inst.asset.cls
        n = self.counts.get(cls, 0)
        self.counts[cls] = n + 1
        yaw = wrap(yaw)
        p = Placed(
            f"{cls}_{n}",
            cls,
            room.id,
            round(x, 3),
            round(y, 3),
            round(yaw, 4),
            inst,
            inst.footprint(round(x, 3), round(y, 3), yaw),
            side,
        )
        self.placed.append(p)
        return p

    def put(
        self,
        room: Room,
        inst: Instance,
        x: float,
        y: float,
        yaw: float,
        gap: float = 0.05,
        side: str = "",
    ) -> Placed | None:
        yaw = wrap(yaw)
        rect = inst.footprint(round(x, 3), round(y, 3), yaw)
        if not self._free(room, rect, not inst.asset.walkable, gap):
            return None
        return self._add(room, inst, x, y, yaw, side)

    def wall(
        self,
        room: Room,
        cls: str,
        *,
        sides: tuple[str, ...] | None = None,
        width: float | None = None,
        tries: int = 120,
    ) -> Placed | None:
        """Put an asset with its back against a wall, anywhere along it."""
        rng, inner = self.rng, inner_rect(room)
        inst = self._inst(room, cls, width)
        order = list(sides) if sides else rng.sample(SIDES, 4)
        for t in range(tries):
            side = order[t % len(order)] if sides else rng.choice(order)
            yaw = _SIDE_YAW[side]
            ex, ey = inst.extent(yaw)
            if side in "WE":
                lo, hi = inner.y0 + ey / 2, inner.y1 - ey / 2
                x = inner.x0 + ex / 2 if side == "W" else inner.x1 - ex / 2
            else:
                lo, hi = inner.x0 + ex / 2, inner.x1 - ex / 2
                y = inner.y0 + ey / 2 if side == "S" else inner.y1 - ey / 2
            if hi < lo:
                continue
            u = rng.choice((lo, hi)) if rng.random() < 0.3 else rng.uniform(lo, hi)
            u = min(hi, max(lo, round(u * 20) / 20))
            x, y = (x, u) if side in "WE" else (u, y)
            p = self.put(room, inst, x, y, yaw, side=side)
            if p:
                return p
        return None

    def centered(self, room: Room, cls: str, margin: float, tries: int = 60) -> Placed | None:
        rng, inner = self.rng, inner_rect(room)
        inst = self._inst(room, cls)
        for _ in range(tries):
            yaw = rng.choice(_YAWS)
            ex, ey = inst.extent(yaw)
            lo_x, hi_x = inner.x0 + ex / 2 + margin, inner.x1 - ex / 2 - margin
            lo_y, hi_y = inner.y0 + ey / 2 + margin, inner.y1 - ey / 2 - margin
            if hi_x < lo_x or hi_y < lo_y:
                continue
            p = self.put(
                room,
                inst,
                round(rng.uniform(lo_x, hi_x), 2),
                round(rng.uniform(lo_y, hi_y), 2),
                yaw,
            )
            if p:
                return p
        return None

    def relative(
        self,
        room: Room,
        anchor: Placed,
        cls: str,
        lx: float,
        ly: float,
        dyaw: float = 0.0,
        gap: float = 0.0,
        width: float | None = None,
    ) -> Placed | None:
        """Place `cls` at local offset (lx, ly) in the anchor's frame."""
        c, s = math.cos(anchor.yaw), math.sin(anchor.yaw)
        x, y = anchor.x + c * lx - s * ly, anchor.y + s * lx + c * ly
        return self.put(room, self._inst(room, cls, width), x, y, anchor.yaw + dyaw, gap)

    def chain(self, room: Room, side: str, classes: list[str]) -> list[Placed]:
        """Line items up along one wall from the start of the wall, skipping blocked spans."""
        inner, out = inner_rect(room), []
        lo, hi = (inner.y0, inner.y1) if side in "WE" else (inner.x0, inner.x1)
        cursor = lo
        for cls in classes:
            widths: list[float | None] = [None]
            if cls == "kitchen_counter":
                top = self.rng.choice((2.4, 1.8, 1.2))
                widths = [w for w in (2.4, 1.8, 1.2, 0.9) if w <= top]
            for width in widths:
                inst = self._inst(room, cls, width)
                p = self._scan(room, inst, side, cursor, hi)
                if p:
                    out.append(p)
                    cursor = p.rect.y1 if side in "WE" else p.rect.x1
                    break
        return out

    def _scan(
        self, room: Room, inst: Instance, side: str, start: float, end: float
    ) -> Placed | None:
        inner, yaw = inner_rect(room), _SIDE_YAW[side]
        ex, ey = inst.extent(yaw)
        length = ey if side in "WE" else ex
        u = start
        while u + length <= end + 1e-9:
            if side in "WE":
                x = inner.x0 + ex / 2 if side == "W" else inner.x1 - ex / 2
                pos = (x, u + length / 2)
            else:
                y = inner.y0 + ey / 2 if side == "S" else inner.y1 - ey / 2
                pos = (u + length / 2, y)
            p = self.put(room, inst, pos[0], pos[1], yaw, gap=0.0, side=side)
            if p:
                return p
            u += 0.1
        return None

    def rug_under(
        self, room: Room, anchor: Placed, forward: float, min_width: float
    ) -> Placed | None:
        for scale in (1.0, 0.8, 0.65):
            p = self.relative(room, anchor, "rug", forward, 0.0, width=max(1.2, min_width) * scale)
            if p:
                return p
        return None

    # ---- room recipes -----------------------------------------------------------------------

    def _need(self, p: Placed | None) -> Placed:
        if p is None:
            raise LayoutFailure
        return p

    def _extras(self, room: Room, shelves: float, plants: float) -> None:
        rng = self.rng
        if rng.random() < shelves:
            self.wall(room, "shelves")
        if rng.random() < shelves * 0.5:
            self.wall(room, "shelves")
        if rng.random() < plants:
            self.wall(room, "plant")
        if rng.random() < plants * 0.5:
            self.wall(room, "plant")

    def dining(self, room: Room) -> bool:
        table = self.centered(room, "dining_table", margin=0.85)
        if table is None:
            return False
        td, tw = table.inst.depth, table.inst.width
        ys = (-tw / 4, tw / 4) if tw >= 1.4 else (0.0,)
        chairs = []
        for s in (1, -1):
            for ly in ys:
                cd = ASSETS["chair"].depth
                lx = s * (td / 2 + cd / 2 + 0.04)
                c = self.relative(room, table, "chair", lx, ly, dyaw=math.pi if s > 0 else 0.0)
                if c:
                    chairs.append(c)
        if len(chairs) < 2:
            for c in chairs + [table]:
                self.placed.remove(c)
            return False
        return True

    def living(self, room: Room, dining: bool) -> None:
        sofa = self._need(self.wall(room, "sofa"))
        if self.rng.random() < 0.9:
            self.wall(room, "tv_unit", sides=(OPPOSITE[sofa.side], *SIDES))
        table = self.relative(
            room, sofa, "coffee_table", sofa.inst.depth / 2 + 0.45 + 0.3, 0.0, gap=0.0
        )
        if self.rng.random() < 0.6:
            anchor = table or sofa
            self.rug_under(
                room,
                anchor,
                0.0 if table else sofa.inst.depth / 2 + 0.8,
                (table.inst.width if table else sofa.inst.width) + 0.6,
            )
        if dining:
            self.dining(room)
        self._extras(room, shelves=0.6, plants=0.7)

    def kitchen(self, room: Room, dining: bool) -> None:
        rng = self.rng
        classes = ["fridge", "kitchen_counter", "oven", "kitchen_counter"]
        if rng.random() < 0.5:
            classes.reverse()
        for side in rng.sample(SIDES, 4):
            got = {p.cls for p in self.chain(room, side, classes)}
            if "kitchen_counter" in got and "fridge" in got:
                break
            self._reset_room(room)
        else:
            raise LayoutFailure
        if rng.random() < 0.5:
            self.wall(room, "kitchen_counter", width=rng.choice((0.9, 1.2, 1.5)))
        if dining:
            self.dining(room)
        if rng.random() < 0.3:
            self.wall(room, "plant")

    def bedroom(self, room: Room) -> None:
        rng = self.rng
        bed = self._need(self.wall(room, "bed", width=rng.choice((0.9, 1.4, 1.6, 1.8))))
        for s in (-1, 1):
            if rng.random() < 0.8:
                self.relative(
                    room,
                    bed,
                    "nightstand",
                    -bed.inst.depth / 2 + 0.2,
                    s * (bed.inst.width / 2 + 0.25),
                )
        if rng.random() < 0.9:
            self.wall(room, "wardrobe")
        if rng.random() < 0.25:
            self.desk(room)
        if rng.random() < 0.5:
            self.rug_under(room, bed, bed.inst.depth / 2 - 0.1, bed.inst.width + 0.8)
        self._extras(room, shelves=0.3, plants=0.4)

    def desk(self, room: Room) -> None:
        desk = self.wall(room, "desk")
        if desk is None:
            return
        chair = self.relative(
            room,
            desk,
            "office_chair",
            desk.inst.depth / 2 + 0.3,
            self.rng.uniform(-0.15, 0.15),
            dyaw=math.pi,
            gap=0.0,
        )
        if chair is None:
            self.placed.remove(desk)
            self.counts["desk"] -= 1

    def office(self, room: Room) -> None:
        self.desk(room)
        if not any(p.room == room.id and p.cls == "desk" for p in self.placed):
            raise LayoutFailure
        self._extras(room, shelves=0.8, plants=0.5)

    def bathroom(self, room: Room) -> None:
        self._need(self.wall(room, "toilet"))
        self._need(self.wall(room, "sink"))
        order = ["bathtub", "shower"] if self.rng.random() < 0.5 else ["shower"]
        if not any(self.wall(room, cls) for cls in order):
            raise LayoutFailure

    def hallway(self, room: Room) -> None:
        self._extras(room, shelves=0.5, plants=0.4)

    def _reset_room(self, room: Room) -> None:
        self.placed = [p for p in self.placed if p.room != room.id]

    def run(self, dining_room: str | None) -> list[Placed]:
        for room in self.plan.rooms:
            dine = room.id == dining_room
            if room.type == "living":
                self.living(room, dine)
            elif room.type == "kitchen":
                self.kitchen(room, dine)
            elif room.type == "bedroom":
                self.bedroom(room)
            elif room.type == "office":
                self.office(room)
            elif room.type == "bathroom":
                self.bathroom(room)
            else:
                self.hallway(room)
        return self.placed


def furnish(
    rng: random.Random,
    plan: FloorPlan,
    openings: list[Opening],
    palette: Palette,
    prof: DroneProfile,
    attempts: int = 25,
) -> tuple[list[Placed], Spawn] | None:
    kitchen = next(r for r in plan.rooms if r.type == "kitchen")
    living = next(r for r in plan.rooms if r.type == "living")
    for _ in range(attempts):
        dining_room = None
        if kitchen.rect.area >= 11 and rng.random() < 0.6:
            dining_room = kitchen.id
        elif living.rect.area >= 19 and rng.random() < 0.7:
            dining_room = living.id
        try:
            placed = Furnisher(rng, plan, openings, palette, prof).run(dining_room)
        except LayoutFailure:
            continue
        spawn = find_spawn(rng, plan, placed, prof)
        if spawn is not None:
            return placed, spawn
    return None
