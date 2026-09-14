"""Simulated detector for the offline harness: an oracle on worldgen furniture plus a noise model.

Stands in for Gazebo's segmentation camera (S3 plan, "Simulated detector"): every pixel whose
depth hit point lies inside a furniture box gets that instance's class, so occlusion by walls and
other furniture is realistic. Noise: missed detections, false positives, box jitter, confidence.
Results with it are oracle-quality perception and must be reported as such.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from maverick_mapping.voxel_map import CameraIntrinsics

from .lifting import Detection


@dataclass
class DetectorNoise:
    miss_rate: float = 0.10  # per visible object per frame
    fp_rate: float = 0.02  # false detections per frame
    box_jitter: float = 0.05  # sigma of each box edge as a fraction of the box size (>= 0.5 px)
    conf_mean: float = 0.8
    conf_sigma: float = 0.08
    min_pixels: int = 8  # fewer visible pixels: not detected
    mask_dropout: float = 0.05  # fraction of mask pixels randomly lost
    use_mask: bool = True  # False: boxes only (a plain detector, no segmentation)


class SimDetector:
    def __init__(
        self,
        manifest: dict,
        intr: CameraIntrinsics,
        noise: DetectorNoise | None = None,
        seed: int = 0,
    ) -> None:
        self.noise = noise or DetectorNoise()
        self.intr = intr
        self.rng = np.random.default_rng(seed)
        fur = manifest.get("furniture", [])
        self.ids = [f["id"] for f in fur]
        self.labels = [f["class"] for f in fur]
        self.lo = np.array([f["bbox"][0] for f in fur], float).reshape(-1, 3)
        self.hi = np.array([f["bbox"][1] for f in fur], float).reshape(-1, 3)
        self.vol = np.prod(self.hi - self.lo, axis=1)
        self._rays = intr.rays()

    def instance_image(self, depth: np.ndarray, t_wc, r_wc) -> np.ndarray:
        """Per-pixel furniture index (-1 none) from the depth image and camera pose."""
        h, w = depth.shape
        inst = np.full((h, w), -1, np.int32)
        valid = np.isfinite(depth) & (depth > 0.1)
        if not valid.any() or not len(self.ids):
            return inst
        z = depth[valid]
        pts = np.asarray(t_wc, float) + (self._rays[valid] * z[:, None]) @ np.asarray(r_wc).T
        pad = (0.03 + 2.0 * (0.005 + 0.002 * z**2))[:, None]
        inside = np.ones((len(z), len(self.ids)), bool)
        for a in range(3):
            inside &= (pts[:, a : a + 1] >= self.lo[None, :, a] - pad) & (
                pts[:, a : a + 1] <= self.hi[None, :, a] + pad
            )
        score = np.where(inside, self.vol[None, :], np.inf)  # smallest containing box wins
        k = score.argmin(axis=1)
        hit = np.isfinite(score[np.arange(len(z)), k])
        flat = np.full(len(z), -1, np.int32)
        flat[hit] = k[hit]
        inst[valid] = flat
        return inst

    def detect(self, depth: np.ndarray, t_wc, r_wc) -> tuple[list[Detection], list[str]]:
        """Detections for one frame and, in parallel, the true furniture id of each (or "fp")."""
        n = self.noise
        h, w = depth.shape
        inst = self.instance_image(depth, t_wc, r_wc)
        dets: list[Detection] = []
        truth: list[str] = []
        for k in np.unique(inst[inst >= 0]):
            m = inst == k
            cnt = int(m.sum())
            if cnt < n.min_pixels or self.rng.random() < n.miss_rate:
                continue
            vv, uu = np.nonzero(m)
            u0, u1, v0, v1 = uu.min(), uu.max(), vv.min(), vv.max()
            bw, bh = u1 - u0 + 1, v1 - v0 + 1
            jit = self.rng.normal(size=4) * np.maximum(
                n.box_jitter * np.array([bw, bh, bw, bh]), 0.5
            )
            box = (
                float(np.clip(u0 + jit[0], 0, w - 1)),
                float(np.clip(v0 + jit[1], 0, h - 1)),
                float(np.clip(u1 + jit[2], 0, w - 1)),
                float(np.clip(v1 + jit[3], 0, h - 1)),
            )
            trunc = u0 <= 1 or v0 <= 1 or u1 >= w - 2 or v1 >= h - 2
            conf = self.rng.normal(n.conf_mean, n.conf_sigma) - (0.1 if trunc else 0.0)
            conf *= min(1.0, 0.5 + cnt / 60.0)
            mask = None
            if n.use_mask:
                mask = m & (self.rng.random(m.shape) >= n.mask_dropout)
            dets.append(Detection(self.labels[k], float(np.clip(conf, 0.2, 0.99)), box, mask))
            truth.append(self.ids[k])
        if self.rng.random() < n.fp_rate and len(self.labels):
            bw, bh = self.rng.uniform(6, 24), self.rng.uniform(6, 24)
            u0, v0 = self.rng.uniform(0, w - bw), self.rng.uniform(0, h - bh)
            lab = self.labels[int(self.rng.integers(len(self.labels)))]
            dets.append(
                Detection(lab, float(self.rng.uniform(0.3, 0.6)), (u0, v0, u0 + bw, v0 + bh), None)
            )
            truth.append("fp")
        return dets, truth
