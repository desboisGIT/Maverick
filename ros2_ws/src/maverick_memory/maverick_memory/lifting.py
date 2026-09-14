"""Detection lifting: (label, confidence, 2D box and/or mask) + depth + camera pose -> 3D."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from maverick_mapping.voxel_map import CameraIntrinsics


@dataclass
class Detection:
    """A 2D detection. box = (u0, v0, u1, v1) in pixels (inclusive), mask = bool (H, W)."""

    label: str
    confidence: float
    box: tuple[float, float, float, float] | None = None
    mask: np.ndarray | None = None


@dataclass
class ObjectObservation:
    """One object seen in one frame, in the map frame (axis-aligned box of the visible surface)."""

    label: str
    confidence: float
    lo: np.ndarray  # (3,) min corner
    hi: np.ndarray  # (3,) max corner
    centroid: np.ndarray  # (3,) mean of the lifted points
    n_points: int
    depth: float  # robust (cluster median) z-depth, m
    truncated: bool = False  # box touches the image border: extent is a lower bound
    t: float = 0.0

    @property
    def center(self) -> np.ndarray:
        return 0.5 * (self.lo + self.hi)


def _nearest_big_cluster(z: np.ndarray, gap: float, frac: float) -> np.ndarray:
    """Boolean selector (in z's order) of the nearest depth cluster holding >= frac of the largest.

    Clusters are runs of sorted depths separated by more than `gap` (plus 5 % of the depth).
    A surface is continuous in depth, so background seen past a box edge, or a small occluder in
    front, form their own clusters.
    """
    order = np.argsort(z)
    zs = z[order]
    cuts = np.nonzero(np.diff(zs) > gap + 0.05 * zs[:-1])[0] + 1
    starts = np.concatenate([[0], cuts])
    ends = np.concatenate([cuts, [len(zs)]])
    sizes = ends - starts
    pick = int(np.nonzero(sizes >= frac * sizes.max())[0][0])
    sel = np.zeros(len(z), bool)
    sel[order[starts[pick] : ends[pick]]] = True
    return sel


def lift_detection(
    det: Detection,
    depth: np.ndarray,
    intr: CameraIntrinsics,
    t_wc: np.ndarray,
    r_wc: np.ndarray,
    t: float = 0.0,
    min_depth: float = 0.3,
    max_depth: float = 8.0,
    min_points: int = 6,
    gap: float = 0.25,
    cluster_frac: float = 0.3,
    trim_pct: float = 3.0,
) -> ObjectObservation | None:
    """Lift a detection to a 3D observation, or None when there is too little valid depth.

    Pixels: the mask (intersected with the box when both are given), else the box. Depth: the
    nearest substantial depth cluster among those pixels (robust to background and occluders).
    Extent: percentile-trimmed min/max of the back-projected points, in the map frame (axis-
    aligned), clipped at the floor (z >= 0).
    """
    d = np.asarray(depth, float)
    h, w = d.shape
    sel = np.zeros((h, w), bool)
    if det.box is not None:
        u0, v0, u1, v1 = det.box
        a, b = max(int(np.floor(u0)), 0), min(int(np.ceil(u1)) + 1, w)
        c, e = max(int(np.floor(v0)), 0), min(int(np.ceil(v1)) + 1, h)
        if b <= a or e <= c:
            return None
        sel[c:e, a:b] = True
    if det.mask is not None:
        sel = det.mask.astype(bool) if det.box is None else sel & det.mask.astype(bool)
    elif det.box is None:
        return None
    vv, uu = np.nonzero(sel & np.isfinite(d) & (d > min_depth) & (d < max_depth))
    if len(vv) < min_points:
        return None
    z = d[vv, uu]
    keep = _nearest_big_cluster(z, gap, cluster_frac)
    vv, uu, z = vv[keep], uu[keep], z[keep]
    if len(z) < min_points:
        return None
    pc = np.stack([(uu - intr.cx) / intr.fx * z, (vv - intr.cy) / intr.fy * z, z], axis=1)
    pw = np.asarray(t_wc, float) + pc @ np.asarray(r_wc, float).T
    if len(pw) >= 30:
        lo = np.percentile(pw, trim_pct, axis=0)
        hi = np.percentile(pw, 100.0 - trim_pct, axis=0)
    else:
        lo, hi = pw.min(axis=0), pw.max(axis=0)
    lo = lo.copy()
    lo[2] = max(lo[2], 0.0)
    hi = np.maximum(hi, lo)
    truncated = False
    if det.box is not None:
        truncated = det.box[0] <= 1 or det.box[1] <= 1 or det.box[2] >= w - 2 or det.box[3] >= h - 2
    return ObjectObservation(
        det.label,
        float(det.confidence),
        lo,
        hi,
        pw.mean(axis=0),
        int(len(z)),
        float(np.median(z)),
        truncated,
        t,
    )
