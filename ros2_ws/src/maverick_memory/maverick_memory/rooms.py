"""Room segmentation of a 2D occupancy grid and stable room ids across map updates (numpy only).

Method (rectilinear walls, as in apartments): find openings, close them with virtual barriers,
split the free space into connected components, and keep the components that are big enough.
An opening is a gap in a wall line: two thin wall pieces on the same grid row (or column) with
a fully observed free gap of 0.5 to 2.8 m between them. This covers doors and the wide open-plan
openings that a distance-transform split cannot separate (the opening is as wide as the room).
Each region is then hole-filled (furniture), grown one cell into the walls, and outlined as a
simplified polygon. Unknown cells are never free, so unexplored space never joins two rooms.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

FREE, OCC = 0, 100
_K = 1 << 20  # absolute cell key = (ai + _K // 2) * _K + (aj + _K // 2)


@dataclass
class RoomConfig:
    gap_min: float = 0.5  # m, narrowest opening
    gap_max: float = 2.8  # m, widest opening closed with a barrier
    run_min: float = 0.25  # m, shortest end-on wall piece that bounds an opening
    thick_max: float = 0.3  # m, thicker obstacles (furniture) do not bound an opening
    min_area: float = 1.5  # m^2, smaller free regions are not rooms
    grow: int = 1  # cells grown into walls so neighbouring rooms meet at the wall
    simplify: float = 0.15  # m, polygon simplification tolerance


@dataclass
class RoomRegion:
    mask: np.ndarray  # bool grid: free cells + filled holes + grown wall cells
    free: np.ndarray  # bool grid: the free cells of the component
    area: float  # m^2 (mask cells)
    centroid: tuple[float, float]  # m, map frame
    polygon: list[tuple[float, float]]  # m, CCW, map frame
    keys: np.ndarray  # absolute cell keys of `mask` (stable across map growth)


def label4(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """4-connected components; returns (labels with 0 = background, count)."""
    nx, ny = mask.shape
    lab = np.zeros((nx, ny), np.int32)
    n = 0
    for sx, sy in zip(*np.nonzero(mask), strict=True):
        if lab[sx, sy]:
            continue
        n += 1
        lab[sx, sy] = n
        dq = deque([(sx, sy)])
        while dq:
            x, y = dq.popleft()
            for u, v in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if 0 <= u < nx and 0 <= v < ny and mask[u, v] and not lab[u, v]:
                    lab[u, v] = n
                    dq.append((u, v))
    return lab, n


def _runs(line: np.ndarray) -> list[tuple[int, int]]:
    """[start, end) of the True runs of a 1D bool array."""
    d = np.diff(np.concatenate([[0], line.astype(np.int8), [0]]))
    return list(zip(np.nonzero(d == 1)[0].tolist(), np.nonzero(d == -1)[0].tolist(), strict=True))


def _thickness(occ: np.ndarray, i: int, j: int) -> int:
    """Contiguous occupied cells through (i, j) along the second axis."""
    n = occ.shape[1]
    a = j
    while a > 0 and occ[i, a - 1]:
        a -= 1
    b = j
    while b < n - 1 and occ[i, b + 1]:
        b += 1
    return b - a + 1


def _thin_end(occ: np.ndarray, j: int, cells, tmax: int, need: int) -> bool:
    return sum(_thickness(occ, i, j) <= tmax for i in cells) >= min(need, len(cells))


def _opening_barriers(grid: np.ndarray, res: float, cfg: RoomConfig) -> np.ndarray:
    occ = grid == OCC
    free = grid == FREE
    # trim spurs (cells with <= 1 occupied 4-neighbour): depth speckle on a wall face and wall tips
    nb = np.zeros(occ.shape, np.int8)
    nb[1:] += occ[:-1]
    nb[:-1] += occ[1:]
    nb[:, 1:] += occ[:, :-1]
    nb[:, :-1] += occ[:, 1:]
    trimmed = occ & (nb < 2)
    occ = occ & (nb >= 2)
    free = free | trimmed
    barrier = np.zeros(grid.shape, bool)
    gmin, gmax = int(round(cfg.gap_min / res)), int(round(cfg.gap_max / res))
    rmin, tmax = int(round(cfg.run_min / res)), int(round(cfg.thick_max / res))
    for transpose in (False, True):
        o = occ.T if transpose else occ  # lines run along axis 0
        f = free.T if transpose else free
        b = barrier.T if transpose else barrier
        # o has shape (n_along, n_across); wall thickness is measured across (axis 1)
        for j in range(o.shape[1]):
            runs = [r for r in _runs(o[:, j]) if r[1] - r[0] >= rmin]
            for (a0, a1), (b0, b1) in zip(runs[:-1], runs[1:], strict=True):
                g = b0 - a1
                if g < gmin or g > gmax or not f[a1:b0, j].all():
                    continue
                # both bounding pieces must be wall, not furniture: thin across the line at the
                # tip, or in at least 3 of the 6 cells nearest the opening (a shelf in front of the
                # wall end). Furniture is thicker than a wall in all of them.
                if not (
                    _thickness(o, a1 - 1, j) <= tmax
                    or _thin_end(o, j, range(a1 - 1, max(a0 - 1, a1 - 7), -1), tmax, 3)
                ):
                    continue
                if not (
                    _thickness(o, b0, j) <= tmax
                    or _thin_end(o, j, range(b0, min(b1, b0 + 6)), tmax, 3)
                ):
                    continue
                b[a1:b0, j] = True
    return barrier


def _fill_holes(mask: np.ndarray) -> np.ndarray:
    pad = np.pad(~mask, 1, constant_values=True)
    lab, _ = label4(pad)
    outside = lab == lab[0, 0]
    return ~outside[1:-1, 1:-1]


def _grow(lab: np.ndarray, allowed: np.ndarray, iters: int) -> np.ndarray:
    """Grow each label into `allowed` unclaimed cells, 4-neighbourhood, `iters` times."""
    lab = lab.copy()
    for _ in range(iters):
        new = lab.copy()
        for sh, ax in ((1, 0), (-1, 0), (1, 1), (-1, 1)):
            nb = np.roll(lab, sh, axis=ax)
            if ax == 0:
                if sh == 1:
                    nb[0] = 0
                else:
                    nb[-1] = 0
            else:
                if sh == 1:
                    nb[:, 0] = 0
                else:
                    nb[:, -1] = 0
            take = (new == 0) & (nb > 0) & allowed
            new[take] = nb[take]
        lab = new
    return lab


def mask_outline(mask: np.ndarray, res: float, origin: tuple[float, float], tol: float):
    """Outer boundary of a cell mask as a CCW polygon in metres, simplified (RDP + collinear)."""
    nx, ny = mask.shape
    m = np.pad(mask, 1)
    edges: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for i, j in zip(*np.nonzero(mask), strict=True):
        p = (m[i + 1, j + 1 - 1], m[i + 1 + 1, j + 1], m[i + 1, j + 1 + 1], m[i + 1 - 1, j + 1])
        if not p[0]:
            edges.setdefault((i, j), []).append((i + 1, j))
        if not p[1]:
            edges.setdefault((i + 1, j), []).append((i + 1, j + 1))
        if not p[2]:
            edges.setdefault((i + 1, j + 1), []).append((i, j + 1))
        if not p[3]:
            edges.setdefault((i, j + 1), []).append((i, j))
    loops = []
    while edges:
        start = next(iter(edges))
        loop = [start]
        cur = start
        while True:
            outs = edges.get(cur)
            if not outs:
                break
            nxt = outs.pop()
            if not outs:
                del edges[cur]
            if nxt == start:
                break
            loop.append(nxt)
            cur = nxt
        if len(loop) >= 4:
            loops.append(loop)
    if not loops:
        return []

    def area(lp):
        a = np.array(lp, float)
        return 0.5 * float(np.sum(a[:, 0] * np.roll(a[:, 1], -1) - np.roll(a[:, 0], -1) * a[:, 1]))

    loop = max(loops, key=lambda lp: area(lp))
    pts = np.array(loop, float)
    # drop collinear vertices
    keep = []
    n = len(pts)
    for k in range(n):
        a, b, c = pts[k - 1], pts[k], pts[(k + 1) % n]
        if abs((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) > 1e-9:
            keep.append(b)
    pts = np.array(keep) if len(keep) >= 3 else pts
    pts = _rdp_closed(pts * res, tol)
    return [(float(origin[0] + x), float(origin[1] + y)) for x, y in pts]


def _rdp(pts: np.ndarray, tol: float) -> np.ndarray:
    if len(pts) < 3:
        return pts
    a, b = pts[0], pts[-1]
    ab = b - a
    nab = float(np.hypot(*ab))
    d = (
        np.hypot(*(pts - a).T)
        if nab < 1e-12
        else np.abs(ab[0] * (pts[:, 1] - a[1]) - ab[1] * (pts[:, 0] - a[0])) / nab
    )
    k = int(np.argmax(d))
    if d[k] <= tol:
        return np.array([a, b])
    return np.vstack([_rdp(pts[: k + 1], tol)[:-1], _rdp(pts[k:], tol)])


def _rdp_closed(pts: np.ndarray, tol: float) -> np.ndarray:
    if len(pts) < 4:
        return pts
    # split at the two points farthest apart so neither end is arbitrary
    k = int(np.argmax(np.hypot(*(pts - pts[0]).T)))
    a = _rdp(np.vstack([pts[: k + 1]]), tol)
    b = _rdp(np.vstack([pts[k:], pts[:1]]), tol)
    return np.vstack([a[:-1], b[:-1]])


def abs_offset(origin: tuple[float, float], res: float) -> tuple[int, int]:
    """Integer index of cell [0, 0] on the global lattice (origin / res, on a fixed lattice)."""
    return int(np.floor(origin[0] / res + 1e-6)), int(np.floor(origin[1] / res + 1e-6))


def segment_rooms(
    grid: np.ndarray,
    res: float,
    origin: tuple[float, float],
    cfg: RoomConfig | None = None,
) -> list[RoomRegion]:
    cfg = cfg or RoomConfig()
    if grid.size == 0:
        return []
    free = grid == FREE
    barrier = _opening_barriers(grid, res, cfg)
    lab, n = label4(free & ~barrier)
    out_lab = np.zeros_like(lab)
    k = 0
    free_masks: dict[int, np.ndarray] = {}
    for c in range(1, n + 1):
        m = lab == c
        if m.sum() * res * res >= cfg.min_area:
            k += 1
            out_lab[m] = k
            free_masks[k] = m
    if k == 0:
        return []
    # fill furniture holes (only cells not claimed by another region), then grow into walls
    for c, m in free_masks.items():
        holes = _fill_holes(m) & ~m & (out_lab == 0)
        out_lab[holes] = c
    out_lab = _grow(out_lab, (grid == OCC) | barrier | (out_lab == 0) & (grid != -1), cfg.grow)
    return [
        region_from_mask(out_lab == c, free_masks[c], res, origin, cfg.simplify)
        for c in range(1, k + 1)
    ]


def region_from_mask(
    mask: np.ndarray, free: np.ndarray, res: float, origin: tuple[float, float], simplify: float
) -> RoomRegion:
    """RoomRegion (area, centroid, outline, stable cell keys) of a cell mask."""
    ox, oy = abs_offset(origin, res)
    ii, jj = np.nonzero(mask)
    keys = (ii + ox + _K // 2).astype(np.int64) * _K + (jj + oy + _K // 2)
    cen = (
        float(origin[0] + (ii.mean() + 0.5) * res),
        float(origin[1] + (jj.mean() + 0.5) * res),
    )
    return RoomRegion(
        mask,
        free,
        float(mask.sum() * res * res),
        cen,
        mask_outline(mask, res, origin, simplify),
        np.sort(keys),
    )


class RoomTracker:
    """Gives regions stable ids across updates by cell overlap with earlier rooms.

    Every id ever issued keeps its last cell set, so a room that merged into a neighbour for a
    while (an opening not yet seen) gets its id back when it splits again. A region inherits the
    id with the largest overlap when that overlap is at least `min_frac` of the smaller of the
    two; otherwise it gets a new id.
    """

    def __init__(self, min_frac: float = 0.5) -> None:
        self.min_frac = min_frac
        self._last: dict[str, np.ndarray] = {}  # id -> keys of its last known cells
        self._n = 0

    def assign(self, regions: list[RoomRegion]) -> list[str]:
        cand = []
        for ri, r in enumerate(regions):
            for rid, keys in self._last.items():
                inter = int(np.intersect1d(r.keys, keys, assume_unique=True).size)
                if inter:
                    frac = inter / min(len(r.keys), len(keys))
                    if frac >= self.min_frac:
                        cand.append((inter, ri, rid))
        cand.sort(key=lambda c: -c[0])
        ids: list[str | None] = [None] * len(regions)
        used: set[str] = set()
        for _, ri, rid in cand:
            if ids[ri] is None and rid not in used:
                ids[ri] = rid
                used.add(rid)
        for ri, r in enumerate(regions):
            if ids[ri] is None:
                self._n += 1
                ids[ri] = f"room_{self._n}"
            self._last[ids[ri]] = r.keys
        return ids  # type: ignore[return-value]


def distance_cells(mask: np.ndarray) -> np.ndarray:
    """Exact Euclidean distance (cells, between cell centres) from each True cell to the nearest
    False cell; cells outside the array count as False. Separable: 1D passes along both axes."""
    m = np.pad(mask, 1)
    nx, ny = m.shape
    inf = float(nx + ny) ** 2
    # pass 1: squared distance to the nearest False cell along axis 0 (forward/backward fill)
    idx = np.arange(nx)[:, None]
    last = np.maximum.accumulate(np.where(~m, idx, -nx * 4), axis=0)
    nxt = np.minimum.accumulate(np.where(~m, idx, nx * 5)[::-1], axis=0)[::-1]
    g = np.minimum((idx - last) ** 2, (nxt - idx) ** 2).astype(float)
    g[g > inf] = inf
    # pass 2: lower envelope along axis 1, row by row
    jj = np.arange(ny)
    d2 = (jj[:, None] - jj[None, :]) ** 2
    out = np.empty_like(g)
    for i in range(nx):
        out[i] = (g[i][None, :] + d2).min(axis=1)
    return np.sqrt(out)[1:-1, 1:-1] * mask
