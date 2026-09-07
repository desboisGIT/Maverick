"""Visual-only wall decor (windows, pictures) and ceiling lamps."""

from __future__ import annotations

import random
from dataclasses import dataclass

from .floorplan import Room
from .furniture import Placed
from .geometry import Rect
from .materials import RGB, Palette
from .walls import THICKNESS, WallSegment

WINDOWS_PER_ROOM = {
    "living": 2,
    "kitchen": 1,
    "bedroom": 1,
    "office": 1,
    "bathroom": 1,
    "hallway": 0,
}


@dataclass(frozen=True)
class Panel:
    """A thin box flush with a wall face, on the room side."""

    kind: str  # "window" | "picture"
    room: str
    orient: str  # orientation of the wall it hangs on
    face: float  # coordinate of the wall face it touches
    inward: int  # +1 / -1: direction from the wall into the room
    u: float  # centre along the wall
    z: float  # centre height
    w: float
    h: float
    color: RGB

    @property
    def rect(self) -> Rect:
        """Footprint on the floor plane, 0.3 m deep into the room."""
        a, b = self.u - self.w / 2, self.u + self.w / 2
        lo, hi = sorted((self.face, self.face + self.inward * 0.3))
        return Rect(lo, a, hi, b) if self.orient == "v" else Rect(a, lo, b, hi)


@dataclass(frozen=True)
class Lamp:
    room: str
    x: float
    y: float


def _blocked(panel: Panel, furniture: list[Placed]) -> bool:
    bottom, r = panel.z - panel.h / 2, panel.rect
    return any(
        p.solid and p.inst.height > bottom and p.rect.intersects(r)
        for p in furniture
        if p.room == panel.room
    )


def _faces(walls: list[WallSegment]) -> list[tuple[WallSegment, str, int, float, bool]]:
    """(segment, room id, inward direction, face coordinate, exterior) for every room-side face."""
    out = []
    half = THICKNESS / 2
    for s in walls:
        if s.low:
            out.append((s, s.low, -1, s.pos - half, s.high is None))
        if s.high:
            out.append((s, s.high, 1, s.pos + half, s.low is None))
    return out


def make_panels(
    rng: random.Random,
    rooms: list[Room],
    walls: list[WallSegment],
    furniture: list[Placed],
    pal: Palette,
    ceiling_height: float,
) -> list[Panel]:
    panels: list[Panel] = []
    faces = _faces(walls)
    ext_order = list(range(len(faces)))
    rng.shuffle(ext_order)
    for room in rooms:
        want = WINDOWS_PER_ROOM[room.type]
        if room.type == "hallway" and rng.random() < 0.4:
            want = 1
        for fi in ext_order:
            seg, rid, inward, face, exterior = faces[fi]
            if want == 0:
                break
            if rid != room.id or not exterior or seg.length < 1.8:
                continue
            bath = room.type == "bathroom"
            w = round(rng.uniform(0.6, 0.9) if bath else rng.uniform(0.9, 1.6), 2)
            h = 0.6 if bath else round(rng.uniform(1.0, 1.3), 2)
            sill = 1.4 if bath else 0.9
            lo, hi = seg.start + 0.4 + w / 2, seg.end - 0.4 - w / 2
            if hi < lo:
                continue
            u = round(rng.uniform(lo, hi), 2)
            if any(
                p.kind == "window"
                and p.room == rid
                and abs(p.u - u) < 1.6
                and p.orient == seg.orient
                and abs(p.face - face) < 1e-6
                for p in panels
            ):
                continue
            cand = Panel(
                "window", rid, seg.orient, face, inward, u, sill + h / 2, w, h, (0.8, 0.9, 1.0)
            )
            if sill + h > ceiling_height - 0.2 or _blocked(cand, furniture):
                continue
            panels.append(cand)
            want -= 1
    # pictures on interior faces
    for seg, rid, inward, face, _exterior in faces:
        for _ in range(2 if seg.length >= 3.2 else 1):
            if seg.length < 1.5 or rng.random() > 0.45:
                continue
            w = round(rng.uniform(0.4, 0.9), 2)
            h = round(w * rng.uniform(0.6, 1.3), 2)
            lo, hi = seg.start + 0.4 + w / 2, seg.end - 0.4 - w / 2
            if hi < lo:
                continue
            u = round(rng.uniform(lo, hi), 2)
            z = round(rng.uniform(1.4, 1.7), 2)
            color = rng.choice(pal.accent + pal.fabric)
            cand = Panel("picture", rid, seg.orient, face, inward, u, z, w, h, color)
            clash = any(
                q.room == rid
                and q.orient == seg.orient
                and abs(q.face - face) < 1e-6
                and abs(q.u - u) < (q.w + w) / 2 + 0.2
                and q.z - q.h / 2 < z + h / 2
                and z - h / 2 < q.z + q.h / 2
                for q in panels
            )
            if not clash and not _blocked(cand, furniture):
                panels.append(cand)
    return panels


def make_lamps(rooms: list[Room]) -> list[Lamp]:
    lamps = []
    for r in rooms:
        cx, cy = r.rect.center
        if r.rect.area > 22:
            if r.rect.w >= r.rect.h:
                lamps += [Lamp(r.id, cx - r.rect.w / 4, cy), Lamp(r.id, cx + r.rect.w / 4, cy)]
            else:
                lamps += [Lamp(r.id, cx, cy - r.rect.h / 4), Lamp(r.id, cx, cy + r.rect.h / 4)]
        else:
            lamps.append(Lamp(r.id, cx, cy))
    return lamps
