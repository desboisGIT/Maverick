"""Log-odds voxel map with vectorised depth-image integration (numpy only)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics in pixels for a width x height image."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @staticmethod
    def from_hfov(width: int, height: int, hfov_deg: float) -> CameraIntrinsics:
        f = (width / 2.0) / np.tan(np.radians(hfov_deg) / 2.0)
        return CameraIntrinsics(width, height, f, f, (width - 1) / 2.0, (height - 1) / 2.0)

    def scaled(self, sx: float, sy: float | None = None) -> CameraIntrinsics:
        sy = sx if sy is None else sy
        return CameraIntrinsics(
            int(round(self.width * sx)),
            int(round(self.height * sy)),
            self.fx * sx,
            self.fy * sy,
            (self.cx + 0.5) * sx - 0.5,
            (self.cy + 0.5) * sy - 0.5,
        )

    def rays(self) -> np.ndarray:
        """Unit-z ray directions in the optical frame, shape (height, width, 3)."""
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        return np.stack(
            [(u - self.cx) / self.fx, (v - self.cy) / self.fy, np.ones(u.shape)], axis=-1
        )


def optical_rotation(yaw: float, pitch: float = 0.0) -> np.ndarray:
    """Rotation optical frame -> map frame for a camera looking along (yaw, pitch up), no roll."""
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    fwd = np.array([cy * cp, sy * cp, sp])
    left = np.array([-sy, cy, 0.0])
    up = np.cross(fwd, left)
    return np.stack([-left, -up, fwd], axis=1)


class VoxelMap:
    """Dense, growable log-odds voxel grid.

    Voxel (i, j, k) covers [origin + (i, j, k) * res, origin + (i + 1, j + 1, k + 1) * res).
    `grid_offset` shifts the lattice so that voxel centres sit at (n + 0.5) * res + grid_offset;
    the default 0.05 puts 0.1 m walls on 0.1 m multiples exactly into one voxel.
    """

    def __init__(
        self,
        resolution: float = 0.1,
        max_range: float = 5.0,
        min_range: float = 0.15,
        l_hit: float = 0.85,
        l_free: float = -0.4,
        l_min: float = -2.0,
        l_max: float = 2.0,
        occ_thresh: float = 0.4,
        carve_sigmas: float = 0.0,
        sigma_base: float = 0.005,
        sigma_quad: float = 0.002,
        grid_offset: float | None = None,
        initial_bounds: tuple[tuple[float, float, float], tuple[float, float, float]] | None = None,
    ) -> None:
        self.res = float(resolution)
        self.max_range = max_range
        self.min_range = min_range
        self.l_hit, self.l_free, self.l_min, self.l_max = l_hit, l_free, l_min, l_max
        self.occ_thresh = occ_thresh
        self.carve_sigmas = carve_sigmas
        self.sigma_base, self.sigma_quad = sigma_base, sigma_quad  # depth sigma = a + b z^2
        off = 0.5 * self.res if grid_offset is None else grid_offset
        self._lattice = off  # voxel boundaries at lattice + n * res
        self.origin = np.zeros(3)
        self.logodds = np.zeros((0, 0, 0), dtype=np.float32)
        self.seen = np.zeros((0, 0, 0), dtype=bool)
        self.frames = 0
        if initial_bounds is not None:
            self.ensure(np.asarray(initial_bounds[0], float), np.asarray(initial_bounds[1], float))

    # --- geometry -----------------------------------------------------------------------------
    @property
    def shape(self) -> tuple[int, int, int]:
        return self.logodds.shape  # type: ignore[return-value]

    def _snap(self, p: np.ndarray) -> np.ndarray:
        return np.floor((p - self._lattice) / self.res).astype(int)

    def ensure(self, lo: np.ndarray, hi: np.ndarray) -> None:
        """Grow the grid (with a small pad) so it contains the box [lo, hi]."""
        pad = 2
        if self.logodds.size == 0:
            i0 = self._snap(lo) - pad
            i1 = self._snap(hi) + pad + 1
            self.origin = i0 * self.res + self._lattice
            shape = tuple(int(n) for n in (i1 - i0))
            self.logodds = np.zeros(shape, dtype=np.float32)
            self.seen = np.zeros(shape, dtype=bool)
            return
        cur0 = self._snap(self.origin + 1e-6)
        cur1 = cur0 + np.array(self.shape)
        i0 = np.minimum(cur0, self._snap(lo) - pad)
        i1 = np.maximum(cur1, self._snap(hi) + pad + 1)
        if np.array_equal(i0, cur0) and np.array_equal(i1, cur1):
            return
        shape = tuple(int(n) for n in (i1 - i0))
        lo_new = np.zeros(shape, dtype=np.float32)
        se_new = np.zeros(shape, dtype=bool)
        a = cur0 - i0
        b = a + np.array(self.shape)
        lo_new[a[0] : b[0], a[1] : b[1], a[2] : b[2]] = self.logodds
        se_new[a[0] : b[0], a[1] : b[1], a[2] : b[2]] = self.seen
        self.logodds, self.seen = lo_new, se_new
        self.origin = i0 * self.res + self._lattice

    def index(self, xyz: np.ndarray) -> np.ndarray:
        """Integer voxel indices of points (n, 3) (may be outside the grid)."""
        return np.floor((np.asarray(xyz, float) - self.origin) / self.res + 1e-9).astype(int)

    def centers(self, idx: np.ndarray) -> np.ndarray:
        return self.origin + (np.asarray(idx, float) + 0.5) * self.res

    def _inside(self, idx: np.ndarray) -> np.ndarray:
        s = np.array(self.shape)
        return np.all((idx >= 0) & (idx < s), axis=-1)

    # --- integration --------------------------------------------------------------------------
    def integrate_depth(
        self,
        depth: np.ndarray,
        intr: CameraIntrinsics,
        t_wc: np.ndarray,
        r_wc: np.ndarray,
        stride: int = 1,
    ) -> None:
        """Integrate one z-depth image taken at camera position t_wc with rotation r_wc.

        Rays are cast from the camera to the measured point (free space), the endpoint voxel is
        marked occupied. Depth NaN/inf/<= min_range is ignored; depth beyond max_range clears free
        space up to max_range and marks nothing occupied. Each voxel is updated at most once per
        sensor per class (free / occupied) per call.
        """
        rays = intr.rays()[::stride, ::stride]
        d = np.asarray(depth, dtype=np.float64)[::stride, ::stride]
        valid = np.isfinite(d) & (d > self.min_range)
        if not valid.any():
            return
        dirs_c = rays[valid]
        z = d[valid]
        norm = np.linalg.norm(dirs_c, axis=1)
        dirs = (dirs_c / norm[:, None]) @ np.asarray(r_wc, float).T
        rng = z * norm
        hit = rng <= self.max_range
        rng_c = np.minimum(rng, self.max_range)
        sigma = self.sigma_base + self.sigma_quad * rng_c**2
        # keep free-space carving off the surface by 2 sigma so noisy far returns don't erase walls
        end_free = rng_c - np.where(hit, 0.6 * self.res + self.carve_sigmas * sigma, 0.0)
        o = np.asarray(t_wc, float)

        pts_hit = o + dirs[hit] * (rng[hit] + 0.25 * self.res)[:, None]
        far = o + dirs * rng_c[:, None]
        lo = np.minimum(np.minimum(far.min(axis=0), o), pts_hit.min(axis=0) if hit.any() else o)
        hi = np.maximum(np.maximum(far.max(axis=0), o), pts_hit.max(axis=0) if hit.any() else o)
        self.ensure(lo - self.res, hi + self.res)

        step = 0.5 * self.res
        n_steps = int(np.ceil(self.max_range / step)) + 1
        s = np.arange(n_steps) * step
        mask = s[None, :] <= end_free[:, None]
        ri, si = np.nonzero(mask)
        p = o + dirs[ri] * s[si][:, None]
        idx = self.index(p)
        ok = self._inside(idx)
        idx = idx[ok]
        flat = np.ravel_multi_index(idx.T, self.shape)
        lf = self.logodds.reshape(-1)
        sf = self.seen.reshape(-1)
        flat = self._unique(flat, lf.size)
        lf[flat] = np.maximum(lf[flat] + self.l_free, self.l_min)
        sf[flat] = True
        if hit.any():
            ih = self.index(pts_hit)
            ih = ih[self._inside(ih)]
            fh = self._unique(np.ravel_multi_index(ih.T, self.shape), lf.size)
            lf[fh] = np.minimum(lf[fh] + self.l_hit, self.l_max)
            sf[fh] = True
        self.frames += 1

    @staticmethod
    def _unique(flat: np.ndarray, size: int) -> np.ndarray:
        mark = np.zeros(size, dtype=bool)
        mark[flat] = True
        return np.flatnonzero(mark)

    # --- queries ------------------------------------------------------------------------------
    def occupied_mask(self) -> np.ndarray:
        return self.logodds > self.occ_thresh

    def state(self, xyz: np.ndarray) -> np.ndarray:
        """UNKNOWN (-1), FREE (0) or OCCUPIED (100) for points (n, 3) or a single point."""
        xyz = np.asarray(xyz, float)
        single = xyz.ndim == 1
        idx = self.index(np.atleast_2d(xyz))
        out = np.full(len(idx), UNKNOWN, dtype=np.int8)
        ok = self._inside(idx)
        i = idx[ok]
        seen = self.seen[i[:, 0], i[:, 1], i[:, 2]]
        occ = self.logodds[i[:, 0], i[:, 1], i[:, 2]] > self.occ_thresh
        vals = np.where(seen, np.where(occ, OCCUPIED, FREE), UNKNOWN).astype(np.int8)
        out[ok] = vals
        return out[0] if single else out

    def _k_range(self, zmin: float, zmax: float) -> tuple[int, int]:
        k0 = int(np.floor((zmin - self.origin[2]) / self.res + 1e-9))
        k1 = int(np.floor((zmax - self.origin[2]) / self.res - 1e-9))  # inclusive
        return max(k0, 0), min(k1, self.shape[2] - 1)

    def project_2d(
        self,
        zmin: float = 0.3,
        zmax: float = 2.0,
        free_z: float | None = None,
        free_half: float = 0.15,
    ) -> tuple[np.ndarray, float, tuple[float, float]]:
        """Project the height band [zmin, zmax] to a 2D grid.

        Returns (grid int8 with -1 unknown / 0 free / 100 occupied, resolution, (x, y) origin of
        cell [0, 0]'s corner). Occupied: any occupied voxel in the band. Free: observed and no
        occupied voxel; if `free_z` is given some voxel within +-free_half of that height must have
        been observed (the flight altitude must be known, not just any height in the band).
        Otherwise unknown. Index order is grid[ix, iy].
        """
        nx, ny, _ = self.shape
        if nx == 0:
            return np.zeros((0, 0), np.int8), self.res, (0.0, 0.0)
        k0, k1 = self._k_range(zmin, zmax)
        if k1 < k0:
            return np.full((nx, ny), UNKNOWN, np.int8), self.res, (self.origin[0], self.origin[1])
        occ = (self.logodds[:, :, k0 : k1 + 1] > self.occ_thresh).any(axis=2)
        if free_z is None:
            seen = self.seen[:, :, k0 : k1 + 1].any(axis=2)
        else:
            a, b = self._k_range(free_z - free_half, free_z + free_half)
            seen = self.seen[:, :, a : b + 1].any(axis=2)
        grid = np.full((nx, ny), UNKNOWN, np.int8)
        grid[seen] = FREE
        grid[occ] = OCCUPIED
        return grid, self.res, (float(self.origin[0]), float(self.origin[1]))

    def occupied_centers(self) -> np.ndarray:
        return self.centers(np.argwhere(self.occupied_mask()))

    def observed_count(self, zmin: float = 0.3, zmax: float = 2.0) -> int:
        k0, k1 = self._k_range(zmin, zmax)
        return int(self.seen[:, :, k0 : k1 + 1].sum()) if k1 >= k0 else 0

    def stats(self, zmin: float = 0.3, zmax: float = 2.0) -> dict:
        k0, k1 = self._k_range(zmin, zmax)
        band = slice(k0, k1 + 1)
        occ = int((self.logodds[:, :, band] > self.occ_thresh).sum()) if k1 >= k0 else 0
        seen = self.observed_count(zmin, zmax)
        v = self.res**3
        return {
            "observed_voxels": seen,
            "occupied_voxels": occ,
            "free_voxels": seen - occ,
            "observed_volume_m3": seen * v,
            "free_volume_m3": (seen - occ) * v,
            "shape": tuple(int(n) for n in self.shape),
        }
