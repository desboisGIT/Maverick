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
    robot_radius: float = 0.39  # x500 circumscribed radius
    squeeze_radius: float | None = None  # optional retry radius when nothing is reachable
    # voxel quantisation (res / 2) + map and tracking residual; 0.10 let the drone clip door frames
    margin: float = 0.22
    los_extra: float = 0.0  # extra clearance demanded of straight-line shortcuts
    # Doors (1.1 m for the 0.78 m x500) cannot be crossed with robot_radius + margin on both sides.
    # Cells between tight_radius and robot_radius + margin from an obstacle are passable only as a
    # "squeeze": penalised, never a viewpoint, crossed slowly on a straight, string-pulled-with-
    # tight-clearance segment. None disables squeezing.
    tight_radius: float | None = 0.46
    # when nothing is reachable (or no path exists) the squeeze radius is relaxed step by step
    tight_fallbacks: tuple[float, ...] = (0.43, 0.40)
    trail_radius: float = 0.2  # flown cells (the robot's own track) are passable within this
    squeeze_cost: float = 10.0
    squeeze_pad_m: float = 0.5  # the cell path this far before / after a squeeze stays in it
    start_free_radius: float = 0.45  # unobserved cells this close to the robot count as free
    unknown_radius: float = 0.3  # keep this far from cells never observed (walls may hide there)
    soft_width: float = 0.6  # clearance cost acts this far beyond the hard inflation
    soft_weight: float = 6.0
    altitude: float = 1.2
    min_cluster: int = 3
    hysteresis: float = 1.5  # score bonus for staying on the previous goal
    cost_offset: float = 1.5  # metres added to path cost in gain / cost
    unknown_weight: float = 0.03
    max_gap: float = 3.5  # unreachable frontiers are approached to within this (m) at best
    gap_weight: float = 3.0
    fail_radius: float = 1.2  # frontier cells within this of a failed viewpoint are dropped
    band: tuple[float, float] = (0.3, 2.0)
    # Careful flying (S3 tuning seeds 20-59: scrapes at door frames, overshoot at a viewpoint beside
    # an unmapped frame, a reversal at speed through a door): path segments that pass within
    # `careful_m` beyond the hard inflation of an obstacle or unobserved cell, and the two
    # segments around a turn sharper than `turn_slow_rad`, are flown at the slow speed.
    careful_m: float = 0.25
    turn_slow_rad: float = 0.9
    # a viewpoint is chosen among the reachable cells near a frontier by cost + weight x (number of
    # clearance levels 0.3 / 0.4 / 0.5 m it falls within of an obstacle or unobserved cell)
    vp_clear_weight: float = 1.0
    vp_window: int = 5  # cells: how far from a frontier cell a viewpoint may be (was 4)


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


def _narrow(occ: np.ndarray, max_span_cells: float) -> np.ndarray:
    """Cells whose nearest obstacles along x (or along y) on both sides span <= max_span_cells."""
    out = np.zeros_like(occ)
    for axis in (0, 1):
        o = np.moveaxis(occ, axis, 0)
        n = o.shape[0]
        idx = np.arange(n).reshape(-1, *([1] * (o.ndim - 1)))
        prev = np.maximum.accumulate(np.where(o, idx, -10**6), axis=0)
        later = np.flip(np.where(o, idx, 10**6), axis=0)
        nxt = np.flip(np.minimum.accumulate(later, axis=0), axis=0)
        span = nxt - prev
        out |= np.moveaxis(span <= max_span_cells, 0, axis)
    return out


@dataclass
class PlanningGrid:
    """Inflated planning view of an occupancy grid: hard-blocked cells and per-cell step cost."""

    blocked: np.ndarray  # bool (nx, ny): occupied+inflated (tight radius), or unknown
    blocked_los: np.ndarray  # stricter, for line-of-sight shortcuts (extra clearance)
    cost: np.ndarray  # float (nx, ny) >= 1
    res: float
    origin: tuple[float, float]
    squeeze: np.ndarray | None = None  # passable only as a squeeze (tight < clearance < hard)
    escape: np.ndarray | None = None  # unblocked only so the robot can leave a too-close spot
    near: np.ndarray | None = None  # within `careful_m` beyond the hard inflation: fly slowly
    vp_pen: np.ndarray | None = None  # clearance levels (0..3) a viewpoint cell falls within

    def __post_init__(self) -> None:
        if self.squeeze is None:
            self.squeeze = np.zeros_like(self.blocked)
        if self.escape is None:
            self.escape = np.zeros_like(self.blocked)

    @staticmethod
    def build(
        grid: np.ndarray,
        res: float,
        origin: tuple[float, float],
        cfg: PlannerConfig,
        start_xy: tuple[float, float] | None = None,
        trail: list[tuple[float, float]] | None = None,
    ) -> PlanningGrid:
        if start_xy is not None:
            # the robot is physically here: cells around it that no ray crossed (the camera sits
            # ahead of the body, so the cells behind the lens may never be observed) are free
            si, sj = int(math.floor((start_xy[0] - origin[0]) / res)), int(
                math.floor((start_xy[1] - origin[1]) / res)
            )
            if 0 <= si < grid.shape[0] and 0 <= sj < grid.shape[1]:
                near = np.zeros(grid.shape, bool)
                near[si, sj] = True
                near = dilate(near, cfg.start_free_radius / res) & (grid == UNKNOWN)
                grid = np.where(near, FREE, grid).astype(grid.dtype)
        occ = grid == OCCUPIED
        hard = dilate(occ, (cfg.robot_radius + cfg.margin) / res)
        unk = grid == UNKNOWN
        unk_b = unk | dilate(unk, cfg.unknown_radius / res)
        strict = hard | unk_b
        if cfg.tight_radius is not None and cfg.tight_radius < cfg.robot_radius + cfg.margin:
            tight = dilate(occ, cfg.tight_radius / res)
            strip = hard & ~tight & ~unk_b
            # a squeeze is a strip cell with obstacles on both sides (a door, a narrow corridor),
            # not the margin beside any single wall
            narrow = strip & _narrow(occ, 2.0 * (cfg.robot_radius + cfg.margin) / res + 1.0)
            # plus the strip cells at the mouth of the passage (the door frame's corners)
            squeeze = strip & dilate(narrow, 0.6 / res)
            narrow = squeeze
            blocked = tight | unk_b | (strip & ~narrow)
        else:
            blocked, squeeze = strict, np.zeros_like(strict)
        # graded clearance cost: grows towards obstacles so narrow passages are crossed centrally
        r_hard = cfg.robot_radius + cfg.margin
        levels = np.arange(r_hard + 0.1, r_hard + cfg.soft_width + 1e-9, 0.1)
        cost = np.ones(grid.shape)
        for rk in levels:
            cost += (cfg.soft_weight / len(levels)) * dilate(occ, rk / res)
        cost += cfg.squeeze_cost * squeeze
        los = dilate(occ, (cfg.robot_radius + cfg.margin + cfg.los_extra) / res) | strict
        if trail:
            # cells the robot itself flew through are passable whatever the (noisy, fat) map says
            tm = np.zeros(grid.shape, bool)
            for x, y in trail:
                i = int(math.floor((x - origin[0]) / res))
                j = int(math.floor((y - origin[1]) / res))
                if 0 <= i < grid.shape[0] and 0 <= j < grid.shape[1]:
                    tm[i, j] = True
            tm = dilate(tm, cfg.trail_radius / res)
            blocked = blocked & ~tm
            los = los & ~tm
            squeeze = squeeze & ~tm
            cost = np.where(tm, np.minimum(cost, 1.0 + cfg.squeeze_cost), cost)
        pg = PlanningGrid(blocked, los, cost, res, origin, squeeze)
        pg.near = dilate(occ, (r_hard + cfg.careful_m) / res) | dilate(
            unk, (cfg.unknown_radius + cfg.careful_m) / res
        )
        ou = occ | unk
        pg.vp_pen = sum(dilate(ou, rk / res).astype(int) for rk in (0.30, 0.40, 0.50))
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
                pg.squeeze &= ~esc
                pg.escape = esc
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


def line_free(
    pg: PlanningGrid, a: tuple[float, float], b: tuple[float, float], tight: bool = False
) -> bool:
    """True if the straight segment a-b (metres) stays on unblocked cells (`tight`: the squeeze
    mask, otherwise the stricter line-of-sight mask)."""
    n = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (0.5 * pg.res))) + 1)
    t = np.linspace(0.0, 1.0, n)
    xs = a[0] + t * (b[0] - a[0])
    ys = a[1] + t * (b[1] - a[1])
    ii = np.floor((xs - pg.origin[0]) / pg.res).astype(int)
    jj = np.floor((ys - pg.origin[1]) / pg.res).astype(int)
    n0, n1 = pg.blocked.shape
    if (ii < 0).any() or (jj < 0).any() or (ii >= n0).any() or (jj >= n1).any():
        return False
    return not (pg.blocked if tight else pg.blocked_los)[ii, jj].any()


def shorten_ex(
    pg: PlanningGrid,
    cells: list[tuple[int, int]],
    start_xy: tuple[float, float] | None = None,
    pad_m: float = 0.5,
) -> tuple[list[tuple[float, float]], list[bool]]:
    """String-pull a cell path into straight segments with line-of-sight checks (metres).

    Returns (points, slow): `slow[k]` is True when the segment ending at points[k] touches a
    squeeze (door); slow[0] is False. Segments inside a squeeze (plus `pad_m` before and after,
    so the door is approached along its axis) are checked against the tight mask, all others
    against the strict one, so only doors are crossed with reduced clearance.
    """
    pts = [pg.to_xy(c) for c in cells]
    if start_xy is not None:
        pts[0] = start_xy
    n = len(cells)
    sq = np.array([bool(pg.squeeze[c]) for c in cells])
    ext = sq.copy()
    for k in range(1, int(round(pad_m / pg.res)) + 1):
        ext[k:] |= sq[:-k]
        ext[:-k] |= sq[k:]
    out = [pts[0]]
    slow = [False]
    i = 0
    while i < n - 1:
        j = n - 1
        while j > i + 1:
            if line_free(pg, pts[i], pts[j]) or (
                ext[i : j + 1].all() and line_free(pg, pts[i], pts[j], tight=True)
            ):
                break
            j -= 1
        out.append(pts[j])
        slow.append(bool(sq[i : j + 1].any() or (ext[i : j + 1].all() and ext[i : j + 1].any())))
        i = j
    return out, slow


def mark_careful(
    pg: PlanningGrid,
    start_xy: tuple[float, float],
    pts: list[tuple[float, float]],
    slow: list[bool],
    turn_slow_rad: float,
) -> list[bool]:
    """Slow flags (per waypoint, `pts` excludes the start) extended to segments that pass close
    to obstacles or unobserved cells (`pg.near`) and to both segments around a sharp turn."""
    out = list(slow)
    if pg.near is None or not pts:
        return out
    path = [start_xy, *pts]
    n0, n1 = pg.near.shape
    for k in range(1, len(path)):
        a, b = path[k - 1], path[k]
        m = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (0.5 * pg.res))) + 1)
        t = np.linspace(0.0, 1.0, m)
        ii = np.floor((a[0] + t * (b[0] - a[0]) - pg.origin[0]) / pg.res).astype(int)
        jj = np.floor((a[1] + t * (b[1] - a[1]) - pg.origin[1]) / pg.res).astype(int)
        ok = (ii >= 0) & (jj >= 0) & (ii < n0) & (jj < n1)
        if ok.any() and pg.near[ii[ok], jj[ok]].any():
            out[k - 1] = True
    for k in range(1, len(path) - 1):
        h1 = math.atan2(path[k][1] - path[k - 1][1], path[k][0] - path[k - 1][0])
        h2 = math.atan2(path[k + 1][1] - path[k][1], path[k + 1][0] - path[k][0])
        d = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
        if d > turn_slow_rad:
            out[k - 1] = out[k] = True
    return out


def shorten(
    pg: PlanningGrid, cells: list[tuple[int, int]], start_xy: tuple[float, float] | None = None
) -> list[tuple[float, float]]:
    """String-pull a cell path into straight segments with line-of-sight checks (metres)."""
    return shorten_ex(pg, cells, start_xy)[0]


@dataclass
class PlanResult:
    status: str  # "goal" or "done"
    reason: str = ""
    goal: Frontier | None = None
    waypoints: list[tuple[float, float, float]] = field(default_factory=list)
    slow: list[bool] = field(default_factory=list)  # per waypoint: crossing a squeeze (door)
    frontiers: list[Frontier] = field(default_factory=list)
    path_cost: float = 0.0
    tight: float | None = None  # relaxed squeeze radius this plan needed (None: the default)
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
        if self.cfg.tight_radius is not None:
            for tr in self.cfg.tight_fallbacks:
                if out.status == "done" and out.reason == "no reachable frontier":
                    out = self._plan(
                        grid, res, origin, robot_xy, dataclasses.replace(self.cfg, tight_radius=tr)
                    )
                    out.tight = tr
        if (
            self.cfg.squeeze_radius is not None
            and out.status == "done"
            and out.reason == "no reachable frontier"
        ):
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
        dist_all = dijkstra_all(pg, start)
        # viewpoints: never in a squeeze (door) nor in the escape zone around the robot
        dist = np.where(pg.squeeze | pg.escape, np.inf, dist_all)
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
                w = cfg.vp_window
                i0, j0 = max(ci - w, 0), max(cj - w, 0)
                sub = dist[i0 : ci + w + 1, j0 : cj + w + 1]
                pen = pg.vp_pen[i0 : ci + w + 1, j0 : cj + w + 1]
                m = int(np.argmin(sub + cfg.vp_clear_weight * pen))
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
        pts, slow = shorten_ex(pg, cells, robot_xy, cfg.squeeze_pad_m)
        wps = [(x, y, cfg.altitude) for x, y in pts[1:]] or [(*best.viewpoint_xy, cfg.altitude)]
        slow = slow[1:] if len(pts) > 1 else [False]
        slow = mark_careful(pg, robot_xy, [(x, y) for x, y, _ in wps], slow, cfg.turn_slow_rad)
        slow[-1] = True  # stop at the viewpoint slowly: PX4 overshoots a fast stop by ~0.15 m
        return PlanResult(
            "goal",
            goal=best,
            waypoints=wps,
            slow=slow,
            frontiers=fronts,
            path_cost=float(dist[best.viewpoint]),
        )
