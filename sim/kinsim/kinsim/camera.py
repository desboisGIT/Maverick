"""Synthetic depth + semantic camera: the Gazebo forward RGB-D camera, by ray casting.

Mirrors sim/models/maverick_small/camera_mount.json and the real image path:

* Gazebo renders 320x240 float depth (clip 0.2-19.1 m, HFOV 1.274 rad), the camera driver turns it
  into u16 millimetres (0 = invalid outside the clip range);
* the mapping node integrates `depth[::stride, ::stride]` (stride 2 = every other pixel) with
  the intrinsics of CameraInfo (principal point at w/2, h/2);
* the memory node takes the pixel at the centre of each 2x2 block, `[1::2, 1::2]`, as 160x120
  depth and the segmentation image (class ids of the worldgen Label plugin) at the same pixels.

Only the pixels these consumers read are cast. The true ray of pixel index u goes through
(u + 0.5 - w/2) / f (pixel centre), while the consumers assume (u - cx) / fx: the half pixel
is kept.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from maverick_mapping.voxel_map import CameraIntrinsics, optical_rotation

from .scene import KScene, RaySet

ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class Mount:
    forward_m: float
    up_m: float
    hfov: float
    size: tuple[int, int]  # width, height
    clip: tuple[float, float]
    rate_hz: float

    @staticmethod
    def load(mount_dir: str = "maverick_small") -> Mount:
        d = json.loads((ROOT / "sim" / "models" / mount_dir / "camera_mount.json").read_text())
        return Mount(
            float(d["xyz_flu"][0]),
            float(d["xyz_flu"][2]),
            float(d["horizontal_fov"]),
            tuple(d["depth_size"]),
            tuple(d["clip"]),
            float(d["rate_hz"]),
        )


def camera_pose(body_xyz: np.ndarray, yaw: float, mount: Mount) -> tuple[np.ndarray, np.ndarray]:
    """Optical-frame position and rotation (optical -> map) of the level camera."""
    t = np.array(
        [
            body_xyz[0] + mount.forward_m * math.cos(yaw),
            body_xyz[1] + mount.forward_m * math.sin(yaw),
            body_xyz[2] + mount.up_m,
        ]
    )
    return t, optical_rotation(yaw, 0.0)


class DepthCamera:
    """Depth (mapping pixels) and semantic + depth (detector pixels) from a KScene."""

    def __init__(self, scene: KScene, mount: Mount, map_stride: int = 2, tile: int = 16) -> None:
        self.scene, self.mount, self.map_stride = scene, mount, map_stride
        w, h = mount.size
        self.f = w / (2.0 * math.tan(mount.hfov / 2.0))
        self.intr = CameraIntrinsics(w, h, self.f, self.f, w / 2.0, h / 2.0)  # CameraInfo K
        self.w, self.h = w, h
        # mapping pixels: indices 0, s, 2s ... (what `depth[::s, ::s]` reads)
        self.map_rays = self._rays(np.arange(0, w, map_stride), np.arange(0, h, map_stride), tile)
        # detector pixels: the centre of each 2x2 block, indices 1, 3, 5 ...
        self.det_rays = None
        self._tile = tile

    def _rays(self, us: np.ndarray, vs: np.ndarray, tile: int) -> RaySet:
        u, v = np.meshgrid(us, vs)
        d = np.stack(
            [
                (u + 0.5 - self.w / 2.0) / self.f,
                (v + 0.5 - self.h / 2.0) / self.f,
                np.ones(u.shape),
            ],
            axis=-1,
        )
        return RaySet(d, u.shape, (tile, tile))

    def _to_depth(self, t: np.ndarray) -> np.ndarray:
        """Camera driver: float metres -> mm integers; 0 outside the clip range -> NaN metres."""
        near, far = self.mount.clip
        ok = np.isfinite(t) & (t >= near - 1e-3) & (t <= far + 1e-3)
        d = np.full(t.shape, np.nan, np.float32)
        d[ok] = np.rint(t[ok] * 1000.0).astype(np.uint16) * np.float32(1e-3)
        return d

    def depth_for_mapping(self, cam_t: np.ndarray, cam_r: np.ndarray) -> np.ndarray:
        """Full-size (h, w) depth with only the pixels `[::stride, ::stride]` filled."""
        t, _ = self.scene.raycast(cam_t, cam_r, self.map_rays, self.mount.clip[1])
        s = self.map_stride
        full = np.full((self.h, self.w), np.nan, np.float32)
        full[::s, ::s] = self._to_depth(t).reshape(self.map_rays.shape)
        return full

    def semantic_and_depth(self, cam_t: np.ndarray, cam_r: np.ndarray):
        """(semantic labels uint8 (120, 160), depth (120, 160) float32) at the detector pixels."""
        if self.det_rays is None:
            self.det_rays = self._rays(np.arange(1, self.w, 2), np.arange(1, self.h, 2), self._tile)
        t, who = self.scene.raycast(cam_t, cam_r, self.det_rays, self.mount.clip[1])
        shape = self.det_rays.shape
        return (
            self.scene.labels_of(who).astype(np.uint8).reshape(shape),
            self._to_depth(t).reshape(shape),
        )
