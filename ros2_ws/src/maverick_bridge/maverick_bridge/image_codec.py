"""Lossless PNG coding of the depth and semantic images (numpy + Pillow, no ROS).

Depth is 16-bit millimetres (0 = invalid), semantic is 8-bit class ids. PNG's per-row filters plus
deflate make smooth depth small (planes and walls compress well) and are exactly reversible, so
the host sees the same numbers the driver measured. Used by the camera driver (encode) and the host
`camera_unpack` node (decode).
"""

from __future__ import annotations

import io

import numpy as np
from PIL import Image

PNG_LEVEL = 3  # compress_level: 1 is fastest, 9 smallest; 3 is within a few % of 9 on depth


def encode_depth_png(depth_mm: np.ndarray) -> bytes:
    if depth_mm.dtype != np.uint16 or depth_mm.ndim != 2:
        raise ValueError("depth must be a 2D uint16 array")
    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(depth_mm)).save(buf, "PNG", compress_level=PNG_LEVEL)
    return buf.getvalue()


def encode_semantic_png(ids: np.ndarray) -> bytes:
    if ids.dtype != np.uint8 or ids.ndim != 2:
        raise ValueError("semantic must be a 2D uint8 array")
    buf = io.BytesIO()
    Image.fromarray(np.ascontiguousarray(ids)).save(buf, "PNG", compress_level=PNG_LEVEL)
    return buf.getvalue()


def decode_png(data: bytes, dtype: np.dtype | type) -> np.ndarray:
    """PNG bytes -> 2D array of `dtype` (np.uint16 for depth, np.uint8 for semantic)."""
    img = Image.open(io.BytesIO(data))
    img.load()
    return np.asarray(img).astype(dtype, copy=False)


def decode_jpeg_rgb(data: bytes) -> np.ndarray:
    """JPEG bytes -> HxWx3 uint8 RGB."""
    img = Image.open(io.BytesIO(data)).convert("RGB")
    return np.asarray(img)
