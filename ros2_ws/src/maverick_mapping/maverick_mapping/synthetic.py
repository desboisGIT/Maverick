"""Offline harness: worldgen world -> box scene, ray-cast depth camera, kinematic drone."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

from .voxel_map import CameraIntrinsics, optical_rotation


def _rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _pose(text: str | None) -> tuple[np.ndarray, np.ndarray]:
    v = [float(x) for x in text.split()] if text else [0.0] * 6
    return np.array(v[:3]), _rpy_matrix(*v[3:6])


def _geom_half(geom: ET.Element) -> np.ndarray | None:
    """Half extents of the box (or the bounding box of a cylinder / sphere) of a collision."""
    box = geom.find("box/size")
    if box is not None and box.text:
        return np.array([float(x) for x in box.text.split()]) / 2.0
    cyl = geom.find("cylinder")
    if cyl is not None:
        r = float(cyl.findtext("radius", "0"))
        return np.array([r, r, float(cyl.findtext("length", "0")) / 2.0])
    sph = geom.find("sphere/radius")
    if sph is not None and sph.text:
        r = float(sph.text)
        return np.array([r, r, r])
    return None


class Scene:
    """Static scene of oriented boxes (walls with door gaps, floor, ceiling, furniture)."""

    def __init__(self, centers, rots, halves, manifest: dict | None = None) -> None:
        c = np.asarray(centers, float).reshape(-1, 3)
        rot = np.asarray(rots, float).reshape(-1, 3, 3)
        h = np.asarray(halves, float).reshape(-1, 3)
        # fold axis-aligned rotations into the extents so most boxes take the fast path
        absr = np.abs(rot)
        aligned = np.all(np.sort(absr.reshape(-1, 9), axis=1)[:, :6] < 1e-6, axis=1)
        h = h.copy()
        for i in np.nonzero(aligned)[0]:
            h[i] = absr[i] @ h[i]
        self.aabb_lo = (c - h)[aligned]
        self.aabb_hi = (c + h)[aligned]
        self.obb_c, self.obb_r, self.obb_h = c[~aligned], rot[~aligned], h[~aligned]
        self.manifest = manifest or {}
        sp = self.manifest.get("spawn", {}).get("pose", [0, 0, 0, 0])
        self.spawn = (float(sp[0]), float(sp[1]), float(sp[3]))
        self.ceiling_height = float(self.manifest.get("ceiling_height", 2.55))

    # --- construction ------------------------------------------------------------------------
    @staticmethod
    def from_sdf(sdf_text: str, manifest: dict | None = None) -> Scene:
        root = ET.fromstring(sdf_text)
        centers, rots, halves = [], [], []
        for model in root.iter("model"):
            if model.get("name") == "ground_plane":
                continue
            mt, mr = _pose(model.findtext("pose"))
            for link in model.findall("link"):
                lt, lr = _pose(link.findtext("pose"))
                for col in link.findall("collision"):
                    geom = col.find("geometry")
                    half = _geom_half(geom) if geom is not None else None
                    if half is None:
                        continue
                    ct, cr = _pose(col.findtext("pose"))
                    t_link = mt + mr @ lt
                    r_link = mr @ lr
                    centers.append(t_link + r_link @ ct)
                    rots.append(r_link @ cr)
                    halves.append(half)
        fp = np.array((manifest or {}).get("footprint", [[0, 0], [8, 8]]), float)
        lo, hi = fp.min(axis=0) - 1.0, fp.max(axis=0) + 1.0
        centers.append([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, -0.5])  # floor slab
        halves.append([(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, 0.5])
        rots.append(np.eye(3))
        return Scene(centers, rots, halves, manifest)

    @staticmethod
    def from_world(world) -> Scene:
        return Scene.from_sdf(world.sdf, world.manifest)

    @staticmethod
    def from_seed(seed: int) -> Scene:
        from worldgen import generate

        return Scene.from_world(generate(seed))

    @property
    def n_boxes(self) -> int:
        return len(self.aabb_lo) + len(self.obb_c)

    @property
    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        fp = np.array(self.manifest["footprint"], float)
        return (
            np.array([fp[:, 0].min(), fp[:, 1].min(), 0.0]),
            np.array([fp[:, 0].max(), fp[:, 1].max(), self.ceiling_height]),
        )

    # --- ray casting -------------------------------------------------------------------------
    def raycast(self, origin: np.ndarray, dirs: np.ndarray, max_t: float = 10.0) -> np.ndarray:
        """First-hit parameter t along each ray (dirs not required to be unit); inf if none."""
        o = np.asarray(origin, np.float32)
        d = np.asarray(dirs, np.float32)
        best = np.full(len(d), np.inf, np.float32)
        # cull boxes that cannot be within range
        if len(self.aabb_lo):
            gap = np.maximum(np.maximum(self.aabb_lo - o, o - self.aabb_hi), 0.0)
            keep = np.linalg.norm(gap, axis=1) <= max_t * float(np.linalg.norm(d, axis=1).max())
            lo = self.aabb_lo[keep].astype(np.float32)
            hi = self.aabb_hi[keep].astype(np.float32)
            if len(lo):
                with np.errstate(divide="ignore", invalid="ignore"):
                    inv = 1.0 / np.where(np.abs(d) < 1e-9, 1e-9, d)
                    t1 = (lo[None] - o) * inv[:, None, :]
                    t2 = (hi[None] - o) * inv[:, None, :]
                tn = np.minimum(t1, t2).max(axis=2)
                tf = np.maximum(t1, t2).min(axis=2)
                tn = np.where((tf >= tn) & (tf > 0) & (tn > 0), tn, np.inf)
                best = np.minimum(best, tn.min(axis=1))
        for c, r, h in zip(self.obb_c, self.obb_r, self.obb_h, strict=True):
            ol = r.T @ (o - c)
            dl = d @ r
            with np.errstate(divide="ignore", invalid="ignore"):
                inv = 1.0 / np.where(np.abs(dl) < 1e-9, 1e-9, dl)
                t1 = (-h - ol) * inv
                t2 = (h - ol) * inv
            tn = np.minimum(t1, t2).max(axis=1)
            tf = np.maximum(t1, t2).min(axis=1)
            tn = np.where((tf >= tn) & (tf > 0) & (tn > 0), tn, np.inf)
            best = np.minimum(best, tn)
        return best

    # --- distance queries --------------------------------------------------------------------
    def signed_distances(self, pts: np.ndarray, z_pad: float = 0.0) -> np.ndarray:
        """Per-box signed distance (negative inside), shape (n, n_boxes).

        z_pad grows every box vertically, so a point's distance approximates that of a vertical
        cylinder of half-height z_pad (the drone body) rather than a sphere.
        """
        pts = np.asarray(pts, float)
        pad = np.array([0.0, 0.0, z_pad])
        out = []
        if len(self.aabb_lo):
            c = (self.aabb_lo + self.aabb_hi) / 2
            h = (self.aabb_hi - self.aabb_lo) / 2 + pad
            out.append(_box_sdf(pts[:, None, :] - c[None], h[None]))
        for c, r, h in zip(self.obb_c, self.obb_r, self.obb_h, strict=True):
            out.append(_box_sdf(((pts - c) @ r)[:, None, :], (h + pad)[None]))
        return np.concatenate(out, axis=1) if out else np.zeros((len(pts), 0))

    def surface_distance(self, pts: np.ndarray, chunk: int = 4000) -> np.ndarray:
        """Distance of each point to the nearest box surface (inside or outside)."""
        res = []
        for i in range(0, len(pts), chunk):
            res.append(np.abs(self.signed_distances(pts[i : i + chunk])).min(axis=1))
        return np.concatenate(res) if res else np.zeros(0)

    def min_clearance(self, p: np.ndarray, z_pad: float = 0.0) -> float:
        """Distance from a point to the nearest solid (negative inside)."""
        return float(self.signed_distances(np.asarray(p, float)[None], z_pad).min())

    def horizontal_clearance(self, p: np.ndarray, half_height: float = 0.1) -> float:
        """Horizontal distance from p to the nearest box overlapping its height (cylinder body)."""
        p = np.asarray(p, float)
        lo = np.concatenate([self.aabb_lo, self.obb_c - self.obb_h])
        hi = np.concatenate([self.aabb_hi, self.obb_c + self.obb_h])
        ok = (lo[:, 2] <= p[2] + half_height) & (hi[:, 2] >= p[2] - half_height)
        best = np.inf
        if len(self.aabb_lo):
            a = ok[: len(self.aabb_lo)]
            c = (self.aabb_lo[a] + self.aabb_hi[a]) / 2
            h = (self.aabb_hi[a] - self.aabb_lo[a]) / 2
            if len(c):
                d = _box_sdf(p[None, :2] - c[:, :2], h[:, :2])
                best = min(best, float(d.min()))
        for k, (c, r, h) in enumerate(zip(self.obb_c, self.obb_r, self.obb_h, strict=True)):
            if ok[len(self.aabb_lo) + k]:
                loc = (p - c) @ r
                best = min(best, float(_box_sdf(loc[None, :2], h[None, :2])[0]))
        return best

    def nearest_box(self, p: np.ndarray) -> tuple[list[float], list[float], float]:
        """(centre, half extents, signed distance) of the box closest to point p."""
        d = self.signed_distances(np.asarray(p, float)[None], 0.1)[0]
        k = int(np.argmin(d))
        n = len(self.aabb_lo)
        if k < n:
            c, h = (self.aabb_lo[k] + self.aabb_hi[k]) / 2, (self.aabb_hi[k] - self.aabb_lo[k]) / 2
        else:
            c, h = self.obb_c[k - n], self.obb_h[k - n]
        return [round(float(v), 2) for v in c], [round(float(v), 2) for v in h], float(d[k])

    def occupied_points(self, pts: np.ndarray) -> np.ndarray:
        return self.signed_distances(pts).min(axis=1) <= 0.0

    # --- ground-truth volumes ----------------------------------------------------------------
    def gt_free_voxels(
        self, res: float = 0.1, offset: float | None = None, zmin: float = 0.3, zmax: float = 2.0
    ) -> tuple[np.ndarray, np.ndarray]:
        """Boolean grid of voxels (centre not inside any box) in the band, plus its origin.

        The lattice matches VoxelMap's default (boundaries at offset + n * res).
        """
        off = 0.5 * res if offset is None else offset
        lo, hi = self.bounds
        i0 = np.floor((lo[:2] - off) / res).astype(int) - 1
        i1 = np.floor((hi[:2] - off) / res).astype(int) + 2
        k0 = int(math.floor((zmin - off) / res))
        k1 = int(math.floor((zmax - off) / res - 1e-9))
        n = np.array([i1[0] - i0[0], i1[1] - i0[1], k1 - k0 + 1])
        origin = np.array([i0[0] * res + off, i0[1] * res + off, k0 * res + off])
        axes = [origin[a] + (np.arange(n[a]) + 0.5) * res for a in range(3)]
        occ = np.zeros(tuple(n), bool)

        def rng(lo_v, hi_v, a):
            a0 = max(int(math.floor((lo_v - origin[a]) / res)), 0)
            a1 = min(int(math.ceil((hi_v - origin[a]) / res)), n[a])
            return a0, a1

        for lo_b, hi_b in zip(self.aabb_lo, self.aabb_hi, strict=True):
            s = [rng(lo_b[a], hi_b[a], a) for a in range(3)]
            if any(x1 <= x0 for x0, x1 in s):
                continue
            sub = [axes[a][s[a][0] : s[a][1]] for a in range(3)]
            m = (
                ((sub[0] >= lo_b[0]) & (sub[0] <= hi_b[0]))[:, None, None]
                & ((sub[1] >= lo_b[1]) & (sub[1] <= hi_b[1]))[None, :, None]
                & ((sub[2] >= lo_b[2]) & (sub[2] <= hi_b[2]))[None, None, :]
            )
            occ[s[0][0] : s[0][1], s[1][0] : s[1][1], s[2][0] : s[2][1]] |= m
        for c, r, h in zip(self.obb_c, self.obb_r, self.obb_h, strict=True):
            ext = np.abs(r) @ h
            s = [rng(c[a] - ext[a], c[a] + ext[a], a) for a in range(3)]
            if any(x1 <= x0 for x0, x1 in s):
                continue
            g = np.meshgrid(*[axes[a][s[a][0] : s[a][1]] for a in range(3)], indexing="ij")
            pts = np.stack(g, axis=-1).reshape(-1, 3)
            loc = (pts - c) @ r
            m = (np.abs(loc) <= h).all(axis=1).reshape(g[0].shape)
            occ[s[0][0] : s[0][1], s[1][0] : s[1][1], s[2][0] : s[2][1]] |= m
        return ~occ, origin

    def reachable_voxels(
        self, free: np.ndarray, origin: np.ndarray, res: float = 0.1
    ) -> np.ndarray:
        """Free voxels 6-connected to the spawn (at flight height ~1.2 m)."""
        sx, sy, _ = self.spawn
        i = int(math.floor((sx - origin[0]) / res))
        j = int(math.floor((sy - origin[1]) / res))
        k = min(max(int(math.floor((1.2 - origin[2]) / res)), 0), free.shape[2] - 1)
        reach = np.zeros_like(free)
        if not free[i, j, k]:
            return reach
        reach[i, j, k] = True
        while True:
            nxt = reach.copy()
            nxt[1:] |= reach[:-1]
            nxt[:-1] |= reach[1:]
            nxt[:, 1:] |= reach[:, :-1]
            nxt[:, :-1] |= reach[:, 1:]
            nxt[:, :, 1:] |= reach[:, :, :-1]
            nxt[:, :, :-1] |= reach[:, :, 1:]
            nxt &= free
            if np.array_equal(nxt, reach):
                return reach
            reach = nxt


def _box_sdf(p: np.ndarray, h: np.ndarray) -> np.ndarray:
    q = np.abs(p) - h
    return np.linalg.norm(np.maximum(q, 0.0), axis=-1) + np.minimum(q.max(axis=-1), 0.0)


@dataclass
class CameraSim:
    """Ray-cast z-depth camera (OAK-D Lite class: 640x480, HFOV ~73 deg, rendered downsampled)."""

    width: int = 80
    height: int = 60
    hfov_deg: float = 73.0
    max_depth: float = 8.0
    noise_base: float = 0.005  # sigma = noise_base + noise_quad * z^2
    noise_quad: float = 0.002
    seed: int = 0

    def __post_init__(self) -> None:
        self.intr = CameraIntrinsics.from_hfov(self.width, self.height, self.hfov_deg)
        self._rays = self.intr.rays().reshape(-1, 3)
        self._rng = np.random.default_rng(self.seed)

    def render(self, scene: Scene, t: np.ndarray, r: np.ndarray) -> np.ndarray:
        dirs = self._rays @ r.T
        z = scene.raycast(t, dirs, self.max_depth).astype(np.float64)
        z[~np.isfinite(z) | (z > self.max_depth)] = np.nan
        sigma = self.noise_base + self.noise_quad * z**2
        z = z + self._rng.normal(size=z.shape) * sigma
        return z.reshape(self.height, self.width)


class KinematicDrone:
    """Point-mass drone with speed/acceleration/yaw-rate limits and a slow position drift.

    `pos` is the true position; `est` (pos + drift) is what the flight stack and map see.
    """

    def __init__(
        self,
        pos: tuple[float, float, float],
        yaw: float,
        vmax: float = 1.0,
        accel: float = 1.0,
        yaw_rate: float = math.radians(90.0),
        drift_sigma: float = 0.0,
        seed: int = 0,
    ) -> None:
        self.pos = np.array(pos, float)
        self.vel = np.zeros(2)
        self.yaw = yaw
        self.vmax, self.accel, self.yaw_rate = vmax, accel, yaw_rate
        self.drift = np.zeros(2)
        self.drift_sigma = drift_sigma
        self._rng = np.random.default_rng(seed)
        self.odometer = 0.0

    @property
    def est(self) -> np.ndarray:
        return np.array([self.pos[0] + self.drift[0], self.pos[1] + self.drift[1], self.pos[2]])

    def turn(self, target_yaw: float, dt: float) -> float:
        """Rotate towards target_yaw at the yaw-rate limit; returns remaining error (rad)."""
        err = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        d = max(-self.yaw_rate * dt, min(self.yaw_rate * dt, err))
        self.yaw += d
        return err - d

    def move(self, target_est: np.ndarray, speed_cap: float, dt: float) -> None:
        """Accelerate towards a target (estimated frame) with a commanded speed cap."""
        delta = np.asarray(target_est[:2]) - self.est[:2]
        dist = float(np.linalg.norm(delta))
        want = np.zeros(2) if dist < 1e-6 else delta / dist * min(speed_cap, self.vmax)
        dv = want - self.vel
        n = float(np.linalg.norm(dv))
        lim = self.accel * dt
        self.vel = want if n <= lim else self.vel + dv / n * lim
        step = self.vel * dt
        self.pos[:2] += step
        self.odometer += float(np.linalg.norm(step))
        if self.drift_sigma:
            self.drift += self._rng.normal(size=2) * self.drift_sigma * math.sqrt(dt)

    def hold(self, dt: float) -> None:
        self.vel = np.zeros(2)
        if self.drift_sigma:
            self.drift += self._rng.normal(size=2) * self.drift_sigma * math.sqrt(dt)

    def camera_pose(self, est: bool = True, pitch: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        return (self.est if est else self.pos).copy(), optical_rotation(self.yaw, pitch)


def seed_from_name(name: str) -> int:
    m = re.search(r"(\d+)$", name)
    return int(m.group(1)) if m else 0
