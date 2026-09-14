"""Scene graph: rooms (from the 2D map, typed from objects) and objects, with the agent queries."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .lifting import Detection, ObjectObservation, lift_detection
from .room_typing import RoomType, TypingConfig, score_room
from .rooms import (
    FREE,
    RoomConfig,
    RoomRegion,
    RoomTracker,
    distance_cells,
    label4,
    segment_rooms,
)
from .store import ObjectStore, StoreConfig, StoredObject, canonical_label

LABELS = {
    "living": "Living room",
    "kitchen": "Kitchen",
    "bedroom": "Bedroom",
    "bathroom": "Bathroom",
    "hallway": "Hallway",
    "office": "Office",
    "unknown": "Room",
}


@dataclass
class SceneGraphConfig:
    store: StoreConfig = field(default_factory=StoreConfig)
    rooms: RoomConfig = field(default_factory=RoomConfig)
    typing: TypingConfig = field(default_factory=TypingConfig)
    room_snap: float = 0.5  # m: a point this close to a room counts as inside it
    robot_clear: float = 0.45  # m: navigation goals keep this far from any obstacle
    wall_clear: float = 1.5  # m: preferred goal distance from the room edge (as far as fits)


@dataclass
class Room:
    id: str
    type: str
    confidence: float
    scores: dict
    secondary: str | None
    polygon: list[tuple[float, float]]
    centroid: tuple[float, float]
    area: float
    elongation: float
    width: float
    region: RoomRegion = field(repr=False)
    idx: int = 0  # index in SceneGraph.room_list / label map value - 1


def _shape(free: np.ndarray, res: float) -> tuple[float, float]:
    """(elongation, short side in m) of a cell set by PCA, exact for rectangles."""
    ii, jj = np.nonzero(free)
    if len(ii) < 3:
        return 1.0, 0.0
    c = np.cov(np.stack([ii, jj]).astype(float))
    ev = np.sort(np.maximum(np.linalg.eigvalsh(c), 1e-9))
    return float(math.sqrt(ev[1] / ev[0])), float(math.sqrt(12.0 * ev[0]) * res)


class SceneGraph:
    """Objects and rooms of the explored space.

    Feed it detections (`add_detections`, `add_observations`) per camera frame and the 2D occupancy
    grid (`update_rooms`) every second or so. Query it with room_of / objects_in / find /
    nearest_room_of_type / goal_in_room. Not thread safe: call from one executor.
    """

    def __init__(self, cfg: SceneGraphConfig | None = None) -> None:
        self.cfg = cfg or SceneGraphConfig()
        self.store = ObjectStore(self.cfg.store)
        self.tracker = RoomTracker()
        self.rooms: dict[str, Room] = {}
        self.visited: set[str] = set()
        self._grid: np.ndarray | None = None
        self._res = 0.1
        self._origin = (0.0, 0.0)
        self._lab = np.zeros((0, 0), np.int32)
        self._list: list[Room] = []

    # --- objects ------------------------------------------------------------------------------
    def add_observations(self, obs: list[ObjectObservation], now: float | None = None) -> None:
        self.store.add_frame(obs, now)
        self._assign_objects()

    def add_detections(
        self,
        dets: list[Detection],
        depth: np.ndarray,
        intr,
        t_wc: np.ndarray,
        r_wc: np.ndarray,
        now: float = 0.0,
    ) -> list[ObjectObservation]:
        """Lift 2D detections with the depth image and camera pose, then store them."""
        obs = [o for d in dets if (o := lift_detection(d, depth, intr, t_wc, r_wc, t=now))]
        self.add_observations(obs, now)
        return obs

    def objects(self, confirmed_only: bool = True) -> list[StoredObject]:
        objs = self.store.confirmed() if confirmed_only else list(self.store.objects.values())
        return sorted(objs, key=lambda o: o.id)

    def find(self, label: str, min_confidence: float = 0.0) -> list[StoredObject]:
        """Confirmed objects of a class, most confident first."""
        lab = canonical_label(label)
        found = [o for o in self.objects() if o.label == lab and o.confidence >= min_confidence]
        return sorted(found, key=lambda o: -o.confidence)

    def _assign_objects(self) -> None:
        for o in self.store.objects.values():
            r = self.room_of(o.position[:2])
            o.room_id = r.id if r else None

    # --- rooms --------------------------------------------------------------------------------
    def update_rooms(self, grid: np.ndarray, res: float, origin: tuple[float, float]) -> None:
        """Re-segment the 2D map (int8 -1/0/100, grid[ix, iy]); ids stay stable across calls."""
        self._grid, self._res, self._origin = np.asarray(grid), float(res), tuple(origin)
        regions = segment_rooms(self._grid, res, origin, self.cfg.rooms)
        ids = self.tracker.assign(regions)
        self._lab = np.zeros(self._grid.shape, np.int32)
        self._list = []
        self.rooms = {}
        for k, (rid, reg) in enumerate(zip(ids, regions, strict=True)):
            elong, width = _shape(reg.free, res)
            room = Room(
                rid,
                "unknown",
                0.0,
                {},
                None,
                reg.polygon,
                reg.centroid,
                reg.area,
                elong,
                width,
                reg,
            )
            room.idx = k
            self._lab[reg.mask] = k + 1
            self._list.append(room)
            self.rooms[rid] = room
        self._assign_objects()
        self._type_rooms()

    def _type_rooms(self) -> None:
        by_room: dict[str, list[tuple[str, float]]] = {}
        for o in self.objects():
            if o.room_id:
                by_room.setdefault(o.room_id, []).append((o.label, o.confidence))
        for r in self._list:
            t: RoomType = score_room(by_room.get(r.id, []), r.elongation, r.width, self.cfg.typing)
            r.type, r.confidence, r.scores, r.secondary = (
                t.type,
                t.confidence,
                t.scores,
                t.secondary,
            )

    def retype(self) -> None:
        """Re-type rooms after new objects arrived (cheap; no map re-segmentation)."""
        self._assign_objects()
        self._type_rooms()

    def room_list(self) -> list[Room]:
        return list(self._list)

    def _cell(self, xy) -> tuple[int, int]:
        return (
            int(math.floor((xy[0] - self._origin[0]) / self._res)),
            int(math.floor((xy[1] - self._origin[1]) / self._res)),
        )

    def room_of(self, xy, snap: float | None = None) -> Room | None:
        """The room containing a map-frame point (x, y[, z]), or the nearest within `snap` m."""
        if self._grid is None or not self._list:
            return None
        snap = self.cfg.room_snap if snap is None else snap
        i, j = self._cell(xy)
        nx, ny = self._lab.shape
        k = int(math.ceil(snap / self._res))
        i0, i1, j0, j1 = max(i - k, 0), min(i + k + 1, nx), max(j - k, 0), min(j + k + 1, ny)
        if i0 >= i1 or j0 >= j1:
            return None
        win = self._lab[i0:i1, j0:j1]
        if 0 <= i < nx and 0 <= j < ny and self._lab[i, j]:
            return self._list[self._lab[i, j] - 1]
        ii, jj = np.nonzero(win)
        if len(ii) == 0:
            return None
        d = np.hypot(ii + i0 - i, jj + j0 - j) * self._res
        n = int(np.argmin(d))
        if d[n] > snap:
            return None
        return self._list[win[ii[n], jj[n]] - 1]

    def objects_in(self, room_id: str, confirmed_only: bool = True) -> list[StoredObject]:
        return [o for o in self.objects(confirmed_only) if o.room_id == room_id]

    def rooms_of_type(self, room_type: str) -> list[Room]:
        return sorted((r for r in self._list if r.type == room_type), key=lambda r: -r.confidence)

    def nearest_room_of_type(self, room_type: str, xy=None, min_confidence: float = 0.0):
        """Closest room of the type (by centroid distance to xy), else the most confident."""
        rs = [r for r in self.rooms_of_type(room_type) if r.confidence >= min_confidence]
        if not rs:  # open plan: a room whose secondary type matches
            rs = [r for r in self._list if r.secondary == room_type]
        if not rs:
            return None
        if xy is None:
            return rs[0]
        return min(rs, key=lambda r: math.hypot(r.centroid[0] - xy[0], r.centroid[1] - xy[1]))

    def visit(self, xy) -> None:
        """Mark the room the drone is in as visited (for the console)."""
        r = self.room_of(xy, snap=0.0)
        if r:
            self.visited.add(r.id)

    # --- navigation goals ---------------------------------------------------------------------
    def goal_in_room(
        self,
        room_id: str,
        start=None,
        anchor=None,
        robot_clear: float | None = None,
        wall_clear: float | None = None,
        near_label: str | None = None,
    ) -> tuple[float, float] | None:
        """A free point inside the room, preferably >= wall_clear m from its edge and
        >= robot_clear m from any obstacle, reachable from `start` (x, y) through such cells.

        Among the preferred cells it picks the one closest to `anchor` (default: the confirmed
        objects of class `near_label` in the room, else the room centroid). If the room is too
        narrow for wall_clear, the cells with the largest clearance win. None if nothing fits.
        """
        room = self.rooms.get(room_id)
        if room is None or self._grid is None:
            return None
        res = self._res
        rc = (self.cfg.robot_clear if robot_clear is None else robot_clear) / res
        wc = self.cfg.wall_clear if wall_clear is None else wall_clear
        free = self._grid == FREE
        # cells from the cell centre to the nearest obstacle face
        d_obs = distance_cells(free) - 0.5
        ok = room.region.free & (d_obs >= rc)
        if start is not None:
            sok = free & (d_obs >= rc)
            lab, _ = label4(sok)
            si, sj = self._cell(start)
            nx, ny = lab.shape
            sl = 0
            for r in range(0, 7):  # snap the start to the nearest passable cell
                win = lab[max(si - r, 0) : si + r + 1, max(sj - r, 0) : sj + r + 1]
                if win.size and win.max() > 0:
                    sl = int(np.bincount(win[win > 0]).argmax())
                    break
            if sl:
                ok &= lab == sl
        if not ok.any():
            return None
        # the mask reaches 0.1 m past the wall face, so subtract one cell to measure to the wall
        d_wall = (distance_cells(room.region.mask) - 1.0) * res
        cw = np.minimum(d_wall, wc)
        best = float(cw[ok].max())
        cand = ok & (cw >= best - 0.05)
        if anchor is None and near_label:
            pts = [o.position[:2] for o in self.objects_in(room_id) if o.label == near_label]
            if pts:
                anchor = np.mean(pts, axis=0)
        if anchor is None:
            anchor = room.centroid
        ii, jj = np.nonzero(cand)
        x = self._origin[0] + (ii + 0.5) * res
        y = self._origin[1] + (jj + 0.5) * res
        n = int(np.argmin(np.hypot(x - anchor[0], y - anchor[1])))
        return float(x[n]), float(y[n])

    def goal_for_type(self, room_type: str, start=None) -> tuple[Room, tuple[float, float]] | None:
        """'Go to the kitchen': nearest room of the type and a goal in it (near its key objects
        when the room is open plan)."""
        room = self.nearest_room_of_type(room_type, start)
        if room is None:
            return None
        key = {"kitchen": "fridge", "living": "sofa", "bedroom": "bed", "bathroom": "toilet"}
        goal = self.goal_in_room(room.id, start=start, near_label=key.get(room_type))
        return (room, goal) if goal else None

    # --- console views ------------------------------------------------------------------------
    def gateway_rooms(self) -> list[dict]:
        return [
            {
                "id": r.id,
                "type": r.type,
                "label": LABELS.get(r.type, "Room"),
                "polygon": [[round(x, 2), round(y, 2)] for x, y in r.polygon],
                "confidence": round(r.confidence, 3),
                "visited": r.id in self.visited,
            }
            for r in self._list
        ]

    def gateway_objects(self, time_offset: float = 0.0) -> list[dict]:
        return [
            {
                "id": o.id,
                "label": o.label,
                "position": [round(float(v), 3) for v in o.position],
                "size": [round(float(v), 3) for v in o.size],
                "room_id": o.room_id,
                "confidence": round(o.confidence, 3),
                "last_seen": o.last_seen + time_offset,
                "status": o.status,
            }
            for o in self.objects()
        ]
