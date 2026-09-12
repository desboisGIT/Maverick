"""Door clearance zones and the walking-corridor test shared by the generator and validator."""

from __future__ import annotations

from collections import deque

from .floorplan import Room
from .geometry import Point, Rect
from .openings import Opening
from .profile import DroneProfile

GRID = 0.2


def _along_range(o: Opening, prof: DroneProfile) -> tuple[float, float]:
    if o.kind == "open":  # only the middle metre of an open-plan gap must stay free
        mid = (o.start + o.end) / 2
        return mid - prof.open_zone_half, mid + prof.open_zone_half
    return o.start - 0.1, o.end + 0.1


def _side(room: Room, o: Opening) -> int:
    """+1 when the room lies on the high (+x / +y) side of the opening wall, else -1."""
    r = room.rect
    low_edge = r.x0 if o.orient == "v" else r.y0
    return 1 if abs(low_edge - o.pos) < 1e-6 else -1


def door_zones(room: Room, openings: list[Opening], prof: DroneProfile) -> list[Rect]:
    zones = []
    for o in openings:
        if room.id not in o.rooms:
            continue
        a, b = _along_range(o, prof)
        s = _side(room, o)
        lo, hi = (o.pos, o.pos + prof.zone_depth) if s > 0 else (o.pos - prof.zone_depth, o.pos)
        zones.append(Rect(lo, a, hi, b) if o.orient == "v" else Rect(a, lo, b, hi))
    return zones


def entry_points(room: Room, openings: list[Opening], prof: DroneProfile) -> list[Point]:
    pts = []
    for o in openings:
        if room.id not in o.rooms:
            continue
        mid = (o.start + o.end) / 2
        d = _side(room, o) * prof.entry_depth
        pts.append((o.pos + d, mid) if o.orient == "v" else (mid, o.pos + d))
    return pts


def entries_connected(
    room: Rect, obstacles: list[Rect], entries: list[Point], radius: float
) -> bool:
    """All entry points reachable from each other with the corridor radius kept from obstacles."""
    if len(entries) < 2:
        return True
    nx, ny = max(1, round(room.w / GRID)), max(1, round(room.h / GRID))
    cw, ch = room.w / nx, room.h / ny
    blocked = [[False] * ny for _ in range(nx)]
    r = radius
    for o in obstacles:
        i0 = max(0, int((o.x0 - r - room.x0) / cw - 0.5) - 1)
        i1 = min(nx, int((o.x1 + r - room.x0) / cw + 0.5) + 2)
        j0 = max(0, int((o.y0 - r - room.y0) / ch - 0.5) - 1)
        j1 = min(ny, int((o.y1 + r - room.y0) / ch + 0.5) + 2)
        for i in range(i0, i1):
            cx = room.x0 + (i + 0.5) * cw
            if not o.x0 - r + 1e-9 < cx < o.x1 + r - 1e-9:
                continue
            for j in range(j0, j1):
                cy = room.y0 + (j + 0.5) * ch
                if o.y0 - r + 1e-9 < cy < o.y1 + r - 1e-9:
                    blocked[i][j] = True

    def cell(p: Point) -> tuple[int, int]:
        return (
            min(nx - 1, max(0, int((p[0] - room.x0) / cw))),
            min(ny - 1, max(0, int((p[1] - room.y0) / ch))),
        )

    cells = [cell(p) for p in entries]
    for i, j in cells:
        blocked[i][j] = False
    seen = {cells[0]}
    queue = deque([cells[0]])
    while queue:
        i, j = queue.popleft()
        for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= n[0] < nx and 0 <= n[1] < ny and n not in seen and not blocked[n[0]][n[1]]:
                seen.add(n)
                queue.append(n)
    return all(c in seen for c in cells)
