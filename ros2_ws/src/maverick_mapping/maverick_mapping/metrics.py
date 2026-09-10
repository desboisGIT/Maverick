"""Exit-test metrics against the synthetic ground truth: C1 coverage, surface error, C6 recall."""

from __future__ import annotations

import numpy as np

from .frontiers import cluster_cells, detect_frontiers
from .synthetic import Scene
from .voxel_map import UNKNOWN, VoxelMap


class GroundTruth:
    """Reachable free volume (3D, flight band) and free columns of a scene, on the map lattice."""

    def __init__(
        self,
        scene: Scene,
        res: float = 0.1,
        band: tuple[float, float] = (0.3, 2.0),
        col_band: tuple[float, float] | None = None,
    ):
        self.scene, self.res, self.band = scene, res, band
        free, origin = scene.gt_free_voxels(res, None, band[0], band[1])
        self.free3d, self.origin = free, origin
        self.reach3d = scene.reachable_voxels(free, origin, res)
        # columns: free over the whole band; reachable by 4-connectivity from the spawn column
        cb = col_band or band
        k0 = int(round((cb[0] - band[0]) / res))
        k1 = free.shape[2] - int(round((band[1] - cb[1]) / res))
        col = free[:, :, k0:k1].all(axis=2)
        sx, sy, _ = scene.spawn
        i = int(np.floor((sx - origin[0]) / res))
        j = int(np.floor((sy - origin[1]) / res))
        reach = np.zeros_like(col)
        if col[i, j]:
            reach[i, j] = True
            while True:
                nxt = reach.copy()
                nxt[1:] |= reach[:-1]
                nxt[:-1] |= reach[1:]
                nxt[:, 1:] |= reach[:, :-1]
                nxt[:, :-1] |= reach[:, 1:]
                nxt &= col
                if np.array_equal(nxt, reach):
                    break
                reach = nxt
        self.reach2d = reach
        idx = np.argwhere(self.reach3d)
        self.reach_centers = origin + (idx + 0.5) * res
        self.volume_m3 = len(idx) * res**3

    def coverage(self, vmap: VoxelMap) -> float:
        """C1: fraction of reachable free voxels (flight band) observed by the map."""
        idx = vmap.index(self.reach_centers)
        ok = vmap._inside(idx)
        seen = np.zeros(len(idx), bool)
        i = idx[ok]
        seen[ok] = vmap.seen[i[:, 0], i[:, 1], i[:, 2]]
        return float(seen.mean()) if len(seen) else 0.0

    def surface_error(self, vmap: VoxelMap) -> dict:
        """Distance of occupied voxel centres to the nearest ground-truth surface (m)."""
        pts = vmap.occupied_centers()
        if len(pts) == 0:
            return {"mean": float("nan"), "p95": float("nan"), "n": 0}
        d = self.scene.surface_distance(pts)
        return {"mean": float(d.mean()), "p95": float(np.percentile(d, 95)), "n": len(pts)}

    def frontier_recall(
        self,
        grid: np.ndarray,
        res: float,
        origin: tuple[float, float],
        min_cluster: int = 3,
        min_region_cells: int = 25,
        near: float = 0.3,
    ) -> dict:
        """C6: share of ground-truth unknown-but-reachable regions that a frontier cluster touches.

        A region is a 8-connected component (>= min_region_cells columns, 0.25 m^2 at 0.1 m) of
        columns that are reachable in ground truth but unknown in `grid`. It counts as detected
        if any detected frontier cell lies within `near` metres of one of its cells.
        """
        unknown_reach = np.zeros(grid.shape, bool)
        di = int(round((origin[0] - self.origin[0]) / self.res))
        dj = int(round((origin[1] - self.origin[1]) / self.res))
        gi0, gj0 = max(-di, 0), max(-dj, 0)  # grid index where GT starts
        n0 = min(grid.shape[0] - gi0, self.reach2d.shape[0] - (gi0 + di))
        n1 = min(grid.shape[1] - gj0, self.reach2d.shape[1] - (gj0 + dj))
        if n0 > 0 and n1 > 0:
            gt = self.reach2d[gi0 + di : gi0 + di + n0, gj0 + dj : gj0 + dj + n1]
            unknown_reach[gi0 : gi0 + n0, gj0 : gj0 + n1] = gt & (
                grid[gi0 : gi0 + n0, gj0 : gj0 + n1] == UNKNOWN
            )
        fronts = detect_frontiers(grid, res, origin, min_cluster)
        fcells = np.concatenate([f.cells for f in fronts]) if fronts else np.zeros((0, 2), int)
        r = int(round(near / res))
        regions = cluster_cells(unknown_reach, min_region_cells)
        hit = 0
        for reg in regions:
            if len(fcells) == 0:
                continue
            lo, hi = reg.min(axis=0) - r, reg.max(axis=0) + r
            cand = fcells[(fcells >= lo).all(axis=1) & (fcells <= hi).all(axis=1)]
            if len(cand) == 0:
                continue
            d2 = ((reg[None, :, :] - cand[:, None, :]) ** 2).sum(axis=2).min()
            hit += int(d2 <= r * r)
        return {"regions": len(regions), "detected": hit}
