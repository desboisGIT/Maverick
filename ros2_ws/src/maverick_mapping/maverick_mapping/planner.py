"""Inflation, A*, path shortening and the frontier exploration policy (numpy + stdlib)."""

from __future__ import annotations

import dataclasses
import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from .frontiers import Frontier, detect_frontiers
from .voxel_map import FREE, OCCUPIED, UNKNOWN

_SQ2 = math.sqrt(2.0)
_NB = [(-1, -1, _SQ2), (-1, 0, 1.0), (-1, 1, _SQ2), (0, -1, 1.0), (0, 1, 1.0),
       (1, -1, _SQ2), (1, 0, 1.0), (1, 1, _SQ2)]  # fmt: skip


@dataclass
class PlannerConfig:
    robot_radius: float = 0.35
    squeeze_radius: float = 0.30  # retried when nothing is reachable at robot_radius
    margin: float = 0.0  # radius is measured to voxel centres: surface clearance >= radius - res/2
    los_extra: float = 0.06  # extra clearance demanded of straight-line shortcuts
    unknown_radius: float = 0.3  # keep this far from cells never observed (walls may hide there)
    soft_radius: float = 0.9  # cells closer than this to an obstacle cost more
    soft_weight: float = 4.0
    altitude: float = 1.2
    min_cluster: int = 3
    hysteresis: float = 1.5  # score bonus for staying on the previous goal
    cost_offset: float = 1.5  # metres added to path cost in gain / cost
    unknown_weight: float = 0.03
    max_gap: float = 3.5  # unreachable frontiers are approached to within this (m) at best
    gap_weight: float = 3.0
    fail_radius: float = 1.2  # frontier cells within this of a failed viewpoint are dropped
    band: tuple[float, float] = (0.3, 2.0)


def disk_offsets(radius_cells: float) -> list[tuple[int, int]]:
    r = int(math.ceil(radius_cells))
    return [
        (i, j)
        for i in range(-r, r + 1)
        for j in range(-r, r + 1)
        if i * i + j * j <= radius_cells * radius_cells + 1e-9
    ]


def dilate(mask: np.ndarray, radius_cells: float) -> np.ndarray:
    """Binary dilation of a boolean grid with a disk (Euclidean, in cell units)."""
    r = int(math.ceil(radius_cells))
    pad = np.pad(mask, r, constant_values=False)
    out = np.zeros_like(mask)
    n0, n1 = mask.shape
    for di, dj in disk_offsets(radius_cells):
        out |= pad[r + di : r + di + n0, r + dj : r + dj + n1]
    return out


@dataclass
class PlanningGrid:
    """Inflated planning view of an occupancy grid: hard-blocked cells and per-cell step cost."""

    blocked: np.ndarray  # bool (nx, ny): occupied+inflated, or unknown
    blocked_los: np.ndarray  # stricter, for line-of-sight shortcuts (extra clearance)
    cost: np.ndarray  # float (nx, ny) >= 1
    res: float
    origin: tuple[float, float]

    @staticmethod
    def build(
        grid: np.ndarray,
        res: float,
        origin: tuple[float, float],
        cfg: PlannerConfig,
        start_xy: tuple[float, float] | None = None,
    ) -> PlanningGrid:
        occ = grid == OCCUPIED
        hard = dilate(occ, (cfg.robot_radius + cfg.margin) / res)
        unk = grid == UNKNOWN
        blocked = hard | unk | dilate(unk, cfg.unknown_radius / res)
        soft = dilate(occ, cfg.soft_radius / res) & ~hard
        cost = 1.0 + cfg.soft_weight * soft
        los = dilate(occ, (cfg.robot_radius + cfg.margin + cfg.los_extra) / res) | blocked
        pg = PlanningGrid(blocked, los, cost, res, origin)
        if start_xy is not None:  # let the robot leave a spot that is slightly too close
            si, sj = pg.to_cell(start_xy)
            if (
                0 <= si < grid.shape[0]
                and 0 <= sj < grid.shape[1]
                and pg.blocked[si, sj]
                and grid[si, sj] != OCCUPIED
            ):
                esc = np.zeros_like(blocked)
                esc[si, sj] = True
                esc = dilate(esc, cfg.robot_radius / res) & (grid == FREE) & pg.blocked
                pg.blocked &= ~esc
                pg.blocked_los &= ~esc
                pg.cost = np.where(esc, 1.0 + 4.0 * cfg.soft_weight, pg.cost)
        return pg

    def to_cell(self, xy: tuple[float, float]) -> tuple[int, int]:
        return (
            int(math.floor((xy[0] - self.origin[0]) / self.res)),
            int(math.floor((xy[1] - self.origin[1]) / self.res)),
        )

    def to_xy(self, cell: tuple[int, int]) -> tuple[float, float]:
        return (
            self.origin[0] + (cell[0] + 0.5) * self.res,
            self.origin[1] + (cell[1] + 0.5) * self.res,
        )

    def free_at(self, i: int, j: int) -> bool:
        n0, n1 = self.blocked.shape
        return 0 <= i < n0 and 0 <= j < n1 and not self.blocked[i, j]


def _padded(pg: PlanningGrid):
    n0, n1 = pg.blocked.shape
    w = n1 + 2
    cost = np.where(pg.blocked, np.inf, pg.cost)
    cost = np.pad(cost, 1, constant_values=np.inf)
    return cost.ravel().tolist(), w, n0, n1


def dijkstra_all(pg: PlanningGrid, start: tuple[int, int]) -> np.ndarray:
    """Cost-to-reach (metres, weighted) from start for every cell; inf where unreachable."""
    cost, w, n0, n1 = _padded(pg)
    s = (start[0] + 1) * w + start[1] + 1
    dist = [math.inf] * len(cost)
    nbs = [(di * w + dj, ln * pg.res) for di, dj, ln in _NB]
    if cost[s] == math.inf:
        return np.full((n0, n1), np.inf)
    dist[s] = 0.0
    heap = [(0.0, s)]
    pop, push = heapq.heappop, heapq.heappush
    while heap:
        d, u = pop(heap)
        if d > dist[u]:
            continue
        for off, ln in nbs:
            v = u + off
            c = cost[v]
            if c == math.inf:
                continue
            nd = d + ln * c
            if nd < dist[v]:
                dist[v] = nd
                push(heap, (nd, v))
    return np.array(dist).reshape(n0 + 2, w)[1:-1, 1:-1]


def astar(
    pg: PlanningGrid, start: tuple[int, int], goal: tuple[int, int]
) -> list[tuple[int, int]] | None:
    """8-connected A* on the planning grid; returns cell path including both ends, or None."""
    cost, w, n0, n1 = _padded(pg)
    s = (start[0] + 1) * w + start[1] + 1
    g = (goal[0] + 1) * w + goal[1] + 1
    if cost[s] == math.inf or cost[g] == math.inf:
        return None
    gi, gj = goal[0] + 1, goal[1] + 1
    nbs = [(di * w + dj, ln * pg.res) for di, dj, ln in _NB]

    def h(u: int) -> float:
        di, dj = abs(u // w - gi), abs(u % w - gj)
        return pg.res * (max(di, dj) + (_SQ2 - 1.0) * min(di, dj))

    best = {s: 0.0}
    parent = {s: -1}
    heap = [(h(s), 0.0, s)]
    pop, push = heapq.heappop, heapq.heappush
    while heap:
        _, d, u = pop(heap)
        if u == g:
            break
        if d > best.get(u, math.inf):
            continue
        for off, ln in nbs:
            v = u + off
            c = cost[v]
            if c == math.inf:
                continue
            nd = d + ln * c
            if nd < best.get(v, math.inf):
                best[v] = nd
                parent[v] = u
                push(heap, (nd + h(v), nd, v))
    if g not in parent:
        return None
    path = []
    u = g
    while u != -1:
        path.append((u // w - 1, u % w - 1))
        u = parent[u]
    return path[::-1]


def line_free(pg: PlanningGrid, a: tuple[float, float], b: tuple[float, float]) -> bool:
    """True if the straight segment a-b (metres) stays on unblocked cells."""
    n = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (0.5 * pg.res))) + 1)
    t = np.linspace(0.0, 1.0, n)
    xs = a[0] + t * (b[0] - a[0])
    ys = a[1] + t * (b[1] - a[1])
    ii = np.floor((xs - pg.origin[0]) / pg.res).astype(int)
    jj = np.floor((ys - pg.origin[1]) / pg.res).astype(int)
    n0, n1 = pg.blocked.shape
    if (ii < 0).any() or (jj < 0).any() or (ii >= n0).any() or (jj >= n1).any():
        return False
    return not pg.blocked_los[ii, jj].any()


def shorten(
    pg: PlanningGrid, cells: list[tuple[int, int]], start_xy: tuple[float, float] | None = None
) -> list[tuple[float, float]]:
    """String-pull a cell path into straight segments with line-of-sight checks (metres)."""
    pts = [pg.to_xy(c) for c in cells]
    if start_xy is not None:
        pts[0] = start_xy
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not line_free(pg, pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


@dataclass
class PlanResult:
    status: str  # "goal" or "done"
    reason: str = ""
    goal: Frontier | None = None
    waypoints: list[tuple[float, float, float]] = field(default_factory=list)
    frontiers: list[Frontier] = field(default_factory=list)
    path_cost: float = 0.0
    squeezed: bool = False  # planned with the reduced squeeze_radius


class ExplorationPlanner:
    """Frontier-based exploration policy with hysteresis and a failed-viewpoint blacklist."""

    def __init__(self, cfg: PlannerConfig | None = None) -> None:
        self.cfg = cfg or PlannerConfig()
        self.goal_centroid: tuple[float, float] | None = None
        self.failed: list[tuple[float, float]] = []
        self.failed_vp: list[tuple[float, float]] = []

    def mark_viewpoint_failed(self, xy: tuple[float, float]) -> None:
        """Never choose a viewpoint within 0.5 m of xy again (arrived, frontier still there)."""
        self.failed_vp.append(xy)

    def mark_failed(self, xy: tuple[float, float]) -> None:
        """Drop frontier cells near xy from now on (a viewpoint that revealed nothing new)."""
        self.failed.append(xy)

    @staticmethod
    def _min_filter(a: np.ndarray, r: int) -> np.ndarray:
        pad = np.pad(a, r, constant_values=np.inf)
        out = np.full(a.shape, np.inf)
        n0, n1 = a.shape
        for di in range(2 * r + 1):
            for dj in range(2 * r + 1):
                np.minimum(out, pad[di : di + n0, dj : dj + n1], out=out)
        return out

    def _exclude(self, shape, res, origin) -> np.ndarray | None:
        if not self.failed:
            return None
        ii, jj = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing="ij")
        x = origin[0] + (ii + 0.5) * res
        y = origin[1] + (jj + 0.5) * res
        ex = np.zeros(shape, bool)
        for fx, fy in self.failed:
            ex |= (x - fx) ** 2 + (y - fy) ** 2 <= self.cfg.fail_radius**2
        return ex

    def plan(
        self,
        grid: np.ndarray,
        res: float,
        origin: tuple[float, float],
        robot_xy: tuple[float, float],
    ) -> PlanResult:
        out = self._plan(grid, res, origin, robot_xy, self.cfg)
        if out.status == "done" and out.reason == "no reachable frontier":
            tight = dataclasses.replace(
                self.cfg, robot_radius=self.cfg.squeeze_radius, los_extra=0.0
            )
            out = self._plan(grid, res, origin, robot_xy, tight)
            out.squeezed = out.status == "goal"
        return out

    def _plan(
        self,
        grid: np.ndarray,
        res: float,
        origin: tuple[float, float],
        robot_xy: tuple[float, float],
        cfg: PlannerConfig,
    ) -> PlanResult:
        fronts = detect_frontiers(
            grid, res, origin, cfg.min_cluster, self._exclude(grid.shape, res, origin)
        )
        if not fronts:
            return PlanResult("done", "no frontier left")
        pg = PlanningGrid.build(grid, res, origin, cfg, robot_xy)
        start = pg.to_cell(robot_xy)
        dist = dijkstra_all(pg, start)
        best, best_score = None, -1.0
        near = self._min_filter(dist, 4)  # cost to the cheapest reachable cell within 0.4 m
        reach_idx = np.argwhere(np.isfinite(dist))
        reach_cost = dist[reach_idx[:, 0], reach_idx[:, 1]]
        for f in fronts:
            dn = near[f.cells[:, 0], f.cells[:, 1]]
            k = int(np.argmin(dn))
            gap = 0.0
            if np.isfinite(dn[k]):
                ci, cj = f.cells[k]
                i0, j0 = max(ci - 4, 0), max(cj - 4, 0)
                sub = dist[i0 : ci + 5, j0 : cj + 5]
                m = int(np.argmin(sub))
                vp = (int(m // sub.shape[1] + i0), int(m % sub.shape[1] + j0))
            else:
                # not reachable (e.g. door narrowed by map noise): approach as close as possible
                if len(reach_idx) == 0:
                    continue
                cs = f.cells[:: max(1, len(f.cells) // 24)]
                d2 = ((reach_idx[:, None, :] - cs[None, :, :]) ** 2).sum(axis=2).min(axis=1)
                m = int(np.argmin(d2 + 1e-4 * reach_cost))
                gap = math.sqrt(float(d2[m])) * res
                if gap > cfg.max_gap:
                    continue
                vp = (int(reach_idx[m, 0]), int(reach_idx[m, 1]))
            xy = pg.to_xy(vp)
            if any(math.hypot(xy[0] - a[0], xy[1] - a[1]) < 0.5 for a in self.failed_vp):
                continue
            f.viewpoint = vp
            f.viewpoint_xy = xy
            cost = float(dist[vp]) + cfg.gap_weight * gap
            gain = f.size + cfg.unknown_weight * f.unknown_gain
            score = gain / (cost + cfg.cost_offset)
            if self.goal_centroid is not None and (
                math.hypot(
                    f.centroid[0] - self.goal_centroid[0], f.centroid[1] - self.goal_centroid[1]
                )
                < 1.2
            ):
                score *= cfg.hysteresis
            if score > best_score:
                best, best_score = f, score
        if best is None:
            return PlanResult("done", "no reachable frontier", frontiers=fronts)
        self.goal_centroid = best.centroid
        cells = astar(pg, start, best.viewpoint)
        if cells is None:
            self.mark_failed(best.viewpoint_xy)
            return self._plan(grid, res, origin, robot_xy, cfg)
        pts = shorten(pg, cells, robot_xy)
        wps = [(x, y, cfg.altitude) for x, y in pts[1:]] or [(*best.viewpoint_xy, cfg.altitude)]
        return PlanResult(
            "goal",
            goal=best,
            waypoints=wps,
            frontiers=fronts,
            path_cost=float(dist[best.viewpoint]),
        )
