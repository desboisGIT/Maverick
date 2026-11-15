"""World geometry for kinsim: ray casting (depth + semantic labels) and body contact.

Built from the worldgen SDF in the *map frame* (translated so the spawn point is the origin, z up
from the floor), like the drone's own frame. Two box sets:

* collision boxes: what the drone body can touch and what the referee scores collisions against
  (identical to the referee's `maverick_mapping.synthetic.Scene`, which is also kept as
  `ref_scene` so scoring uses the referee's own code);
* render boxes: the collision boxes plus the visual-only boxes of furniture without a collision
  (rugs, bananas, bread), because Gazebo's depth and segmentation cameras see visuals.

Ray casting is exact (slab test) behind a tile / bounding-sphere broad phase, so a 160x120 depth
image costs a few milliseconds instead of the 200 ms of the offline harness' brute force.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

from . import paths as _paths  # noqa: F401

# worldgen label ids (sim/worldgen/worldgen/classes.py): floor 1 ... rug 26, banana 27 ...
FIRST_FURNITURE_ID = 7


def _rpy(r: float, p: float, y: float) -> np.ndarray:
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
    return np.array(v[:3]), _rpy(*v[3:6])


def _half(geom: ET.Element) -> np.ndarray | None:
    box = geom.find("box/size")
    if box is not None and box.text:
        return np.array([float(x) for x in box.text.split()]) / 2.0
    cyl = geom.find("cylinder")
    if cyl is not None:  # bounding box of the cylinder (as the referee's Scene does)
        r = float(cyl.findtext("radius", "0"))
        return np.array([r, r, float(cyl.findtext("length", "0")) / 2.0])
    sph = geom.find("sphere/radius")
    if sph is not None and sph.text:
        r = float(sph.text)
        return np.array([r, r, r])
    return None


@dataclass
class Boxes:
    """Oriented boxes: centre c, rotation r (local -> world), half extents h, label id."""

    c: np.ndarray  # (n, 3)
    r: np.ndarray  # (n, 3, 3)
    h: np.ndarray  # (n, 3)
    label: np.ndarray  # (n,) int
    solid: np.ndarray  # (n,) bool, True: collision box

    @property
    def n(self) -> int:
        return len(self.c)


def parse_sdf(sdf_text: str) -> tuple[Boxes, Boxes]:
    """(collision boxes, render boxes) of a worldgen SDF in the world frame (floor slab added)."""
    root = ET.fromstring(sdf_text)
    col: list[tuple] = []
    vis_only: list[tuple] = []
    for model in root.iter("model"):
        if model.get("name") == "ground_plane":
            continue
        mt, mr = _pose(model.findtext("pose"))
        lab_el = model.find("plugin/label")
        label = int(lab_el.text) if lab_el is not None and lab_el.text else 0
        n_col = 0
        mine: list[tuple] = []
        for link in model.findall("link"):
            lt, lr = _pose(link.findtext("pose"))
            for tag in ("collision", "visual"):
                for el in link.findall(tag):
                    geom = el.find("geometry")
                    half = _half(geom) if geom is not None else None
                    if half is None:
                        continue
                    ct, cr = _pose(el.findtext("pose"))
                    t_link, r_link = mt + mr @ lt, mr @ lr
                    item = (t_link + r_link @ ct, r_link @ cr, half, label)
                    if tag == "collision":
                        col.append(item)
                        n_col += 1
                    else:
                        mine.append(item)
        if n_col == 0 and label >= FIRST_FURNITURE_ID:  # rugs, bananas, bread: seen, not touched
            vis_only.extend(mine)

    def pack(items: list[tuple], solid: bool) -> Boxes:
        if not items:
            return Boxes(
                np.zeros((0, 3)), np.zeros((0, 3, 3)), np.zeros((0, 3)),
                np.zeros(0, int), np.zeros(0, bool),
            )  # fmt: skip
        c, r, h, lab = zip(*items, strict=True)
        return Boxes(
            np.array(c, float),
            np.array(r, float),
            np.array(h, float),
            np.array(lab, int),
            np.full(len(items), solid, bool),
        )

    return pack(col, True), pack(vis_only, False)


def _concat(a: Boxes, b: Boxes) -> Boxes:
    return Boxes(
        np.concatenate([a.c, b.c]),
        np.concatenate([a.r, b.r]),
        np.concatenate([a.h, b.h]),
        np.concatenate([a.label, b.label]),
        np.concatenate([a.solid, b.solid]),
    )


class RaySet:
    """A fixed bundle of camera-frame rays (z = 1), grouped into tiles for the broad phase."""

    def __init__(self, dirs_cam: np.ndarray, shape: tuple[int, int], tile: tuple[int, int]):
        self.dirs = np.asarray(dirs_cam, np.float32).reshape(-1, 3)
        self.shape = shape  # (h, w) of the sampled grid
        h, w = shape
        idx = np.arange(h * w).reshape(h, w)
        self.tiles: list[tuple[np.ndarray, np.ndarray, float]] = []
        th, tw = tile
        for v0 in range(0, h, th):
            for u0 in range(0, w, tw):
                ids = idx[v0 : v0 + th, u0 : u0 + tw].ravel()
                d = self.dirs[ids]
                u = d / np.linalg.norm(d, axis=1, keepdims=True)
                cen = u.mean(axis=0)
                cen /= np.linalg.norm(cen)
                alpha = float(np.arccos(np.clip(u @ cen, -1, 1)).max()) + 1e-4
                self.tiles.append((ids, cen.astype(np.float32), alpha))


class KScene:
    """Static scene in the map frame with ray casting and body-contact queries."""

    def __init__(self, world_sdf: str, manifest: dict, with_ref_scene: bool = True) -> None:
        self.manifest = manifest
        sp = manifest["spawn"]["pose"]
        self.origin = np.array([float(sp[0]), float(sp[1]), 0.0])
        col, vis = parse_sdf(world_sdf)
        fp = np.array(manifest.get("footprint", [[0, 0], [8, 8]]), float)
        lo, hi = fp.min(axis=0) - 1.0, fp.max(axis=0) + 1.0
        floor = Boxes(
            np.array([[(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, -0.5]]),
            np.eye(3)[None],
            np.array([[(hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, 0.5]]),
            np.array([1]),
            np.array([True]),
        )
        col = _concat(col, floor)
        for b in (col, vis):
            b.c = b.c - self.origin
        self.col, self.render = col, _concat(col, vis)
        self.floor_idx = col.n - 1
        self.ceiling_height = float(manifest.get("ceiling_height", 2.55))
        self.spawn = (0.0, 0.0, float(sp[3]))
        self.spawn_world = (float(sp[0]), float(sp[1]))
        self._prep_render()
        self._prep_contact()
        self.ref_scene = None
        if with_ref_scene:
            from maverick_mapping.synthetic import Scene
            from referee.s3_metrics import shift_scene

            self.ref_scene = shift_scene(
                Scene.from_sdf(world_sdf, manifest), (self.origin[0], self.origin[1], 0.0)
            )

    @staticmethod
    def from_seed(seed: int, extras: bool = False, drone: str | None = None) -> KScene:
        from worldgen import generate

        kw = {"small_objects": extras, "people": extras}
        if drone:
            kw["drone"] = drone
        w = generate(seed, **kw)
        return KScene(w.sdf, w.manifest)

    # --- render -----------------------------------------------------------------------------
    def _prep_render(self) -> None:
        b = self.render
        absr = np.abs(b.r)
        aligned = np.all(np.sort(absr.reshape(-1, 9), axis=1)[:, :6] < 1e-4, axis=1)
        ext = np.einsum("nij,nj->ni", absr, b.h)  # world half extents of the AABB
        self._lo = (b.c - ext).astype(np.float32)
        self._hi = (b.c + ext).astype(np.float32)
        self._aligned = aligned
        self._obb = np.nonzero(~aligned)[0]
        # aligned boxes use the AABB itself (the rotation only permutes the extents)
        he = np.where(aligned[:, None], ext, b.h)
        self._he = he
        self._bc = b.c.astype(np.float32)
        self._br = np.linalg.norm(ext, axis=1).astype(np.float32)

    def raycast(self, origin: np.ndarray, r_wc: np.ndarray, rays: RaySet, max_t: float):
        """First hit along each ray of `rays` (camera frame -> world by r_wc), from `origin`.

        Returns (t, box) per ray: t is the z-depth parameter (dirs have z = 1), inf when nothing
        is hit within max_t; box is the render-box index (-1 for none).
        """
        o = np.asarray(origin, np.float32)
        r = np.asarray(r_wc, np.float32)
        n = len(rays.dirs)
        best = np.full(n, np.inf, np.float32)
        who = np.full(n, -1, np.int32)
        rel = self._bc - o
        dist = np.linalg.norm(rel, axis=1)
        near = np.nonzero(dist - self._br <= max_t * 1.5)[0]
        if len(near) == 0:
            return best, who
        rel_u = rel[near] / np.maximum(dist[near, None], 1e-6)
        br = self._br[near]
        sin_r = np.arcsin(np.clip(br / np.maximum(dist[near], 1e-6), 0, 1))
        inside = dist[near] <= br
        lo, hi = self._lo[near], self._hi[near]
        aligned = self._aligned[near]
        for ids, cen, alpha in rays.tiles:
            cw = r @ cen
            ang = np.arccos(np.clip(rel_u @ cw, -1, 1))
            sel = np.nonzero((ang <= alpha + sin_r) | inside)[0]
            if len(sel) == 0:
                continue
            d = rays.dirs[ids] @ r.T  # (k, 3) world dirs (z of the camera frame = 1)
            sa = sel[aligned[sel]]
            if len(sa):
                with np.errstate(divide="ignore", invalid="ignore"):
                    inv = 1.0 / np.where(np.abs(d) < 1e-9, 1e-9, d)
                    t1 = (lo[sa][None] - o) * inv[:, None, :]
                    t2 = (hi[sa][None] - o) * inv[:, None, :]
                tn = np.minimum(t1, t2).max(axis=2)
                tf = np.maximum(t1, t2).min(axis=2)
                ok = (tf >= tn) & (tf > 0) & (tn > 0)
                tn = np.where(ok, tn, np.inf)
                k = tn.argmin(axis=1)
                tmin = tn[np.arange(len(k)), k]
                better = tmin < best[ids]
                best[ids] = np.where(better, tmin, best[ids])
                who[ids] = np.where(better, near[sa][k], who[ids])
            for s in sel[~aligned[sel]]:
                g = near[s]
                rot = self.render.r[g].astype(np.float32)
                ol = rot.T @ (o - self.render.c[g].astype(np.float32))
                dl = d @ rot
                h = self.render.h[g].astype(np.float32)
                with np.errstate(divide="ignore", invalid="ignore"):
                    inv = 1.0 / np.where(np.abs(dl) < 1e-9, 1e-9, dl)
                    t1 = (-h - ol) * inv
                    t2 = (h - ol) * inv
                tn = np.minimum(t1, t2).max(axis=1)
                tf = np.maximum(t1, t2).min(axis=1)
                tn = np.where((tf >= tn) & (tf > 0) & (tn > 0), tn, np.inf)
                better = tn < best[ids]
                best[ids] = np.where(better, tn, best[ids])
                who[ids] = np.where(better, g, who[ids])
        best[best > max_t] = np.inf
        who[~np.isfinite(best)] = -1
        return best, who

    def labels_of(self, box_idx: np.ndarray) -> np.ndarray:
        out = np.zeros(len(box_idx), np.int32)
        ok = box_idx >= 0
        out[ok] = self.render.label[box_idx[ok]]
        return out

    # --- contact ----------------------------------------------------------------------------
    def _prep_contact(self) -> None:
        b = self.col
        self._c_lo_z = b.c[:, 2] - np.einsum("nj,nj->n", np.abs(b.r[:, 2, :]), b.h)
        self._c_hi_z = b.c[:, 2] + np.einsum("nj,nj->n", np.abs(b.r[:, 2, :]), b.h)
        self._c_rad = np.hypot(*np.einsum("nij,nj->ni", np.abs(b.r), b.h)[:, :2].T)

    def contact(self, p: np.ndarray, half_height: float = 0.12) -> tuple[float, np.ndarray, int]:
        """(horizontal clearance, unit normal pointing away from the nearest solid, box index).

        The body is a vertical cylinder of half height `half_height` (the referee's footprint);
        clearance is negative when its centre is inside a box. Same distance as the referee's
        `Scene.horizontal_clearance`.
        """
        b = self.col
        z = float(p[2])
        ok = (self._c_lo_z <= z + half_height) & (self._c_hi_z >= z - half_height)
        near = np.nonzero(ok & (np.hypot(*(b.c[:, :2] - p[:2]).T) < self._c_rad + 1.5))[0]
        if len(near) == 0:
            return math.inf, np.array([1.0, 0.0]), -1
        loc = np.einsum("nji,nj->ni", b.r[near], p[None] - b.c[near])  # R^T (p - c)
        q = np.abs(loc[:, :2]) - b.h[near, :2]
        out = np.maximum(q, 0.0)
        d_out = np.hypot(out[:, 0], out[:, 1])
        d = d_out + np.minimum(q.max(axis=1), 0.0)
        k = int(np.argmin(d))
        g = int(near[k])
        if d_out[k] > 1e-9:
            n_loc = np.sign(loc[k, :2]) * out[k] / d_out[k]
        else:  # inside: leave through the nearest face
            ax = int(np.argmax(q[k]))
            n_loc = np.zeros(2)
            n_loc[ax] = 1.0 if loc[k, ax] >= 0 else -1.0
        n_w = b.r[g][:2, :2] @ n_loc
        nn = float(np.hypot(*n_w)) or 1.0
        return float(d[k]), n_w / nn, g

    # --- helpers ----------------------------------------------------------------------------
    def floor_height(self, xy: np.ndarray) -> float:
        """Height of the highest low obstacle (below the 0.35 m obstacle band) under a point:
        the rangefinder-based height estimate lifts the drone by it. 0 over bare floor."""
        b = self.col
        top = b.c[:, 2] + np.einsum("nj,nj->n", np.abs(b.r[:, 2, :]), b.h)
        cand = np.nonzero((top > 0.01) & (top < 0.35) & (self._c_lo_z < 0.05))[0]
        h = 0.0
        for g in cand:
            loc = b.r[g].T @ (np.array([xy[0], xy[1], b.c[g, 2]]) - b.c[g])
            if abs(loc[0]) <= b.h[g, 0] and abs(loc[1]) <= b.h[g, 1]:
                h = max(h, float(top[g]))
        return h
