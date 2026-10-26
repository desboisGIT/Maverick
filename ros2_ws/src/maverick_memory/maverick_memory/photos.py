"""Photo crops of detected objects (pure: PIL + the standard library, no ROS).

Memory keeps a reference (a file path) to the best crop of an object, so the agent can send it as
a photo ("here is the banana"). The crop comes from the RGB frame closest in time to the
detection; the box is normalised to the image like `detections_json` boxes.
"""

from __future__ import annotations

import io
from collections import deque
from pathlib import Path

PHOTO_LABELS = frozenset({"banana", "bread", "person"})


class FrameBuffer:
    """The last few JPEG frames by time, for 'the frame nearest to a detection stamp'."""

    def __init__(self, maxlen: int = 12) -> None:
        self._q: deque[tuple[float, bytes]] = deque(maxlen=maxlen)

    def add(self, t: float, jpeg: bytes) -> None:
        self._q.append((t, jpeg))

    def nearest(self, t: float, tol: float = 0.25) -> bytes | None:
        best = min(self._q, key=lambda e: abs(e[0] - t), default=None)
        return best[1] if best is not None and abs(best[0] - t) <= tol else None


def crop_jpeg(
    jpeg: bytes,
    box_norm: tuple[float, float, float, float],
    out_path: Path,
    pad: float = 0.35,
    min_side: int = 48,
) -> tuple[int, int] | None:
    """Write the padded crop of `box_norm` = (x, y, w, h) (0..1, top-left) as a JPEG.

    Returns the crop size in pixels, or None when the frame cannot be decoded.
    """
    from PIL import Image

    try:
        im = Image.open(io.BytesIO(jpeg)).convert("RGB")
    except (OSError, ValueError):
        return None
    w, h = im.size
    x, y, bw, bh = box_norm
    cx, cy = (x + bw / 2) * w, (y + bh / 2) * h
    half_w = max(bw * w * (0.5 + pad), min_side / 2)
    half_h = max(bh * h * (0.5 + pad), min_side / 2)
    x0, y0 = max(0, int(cx - half_w)), max(0, int(cy - half_h))
    x1, y1 = min(w, int(cx + half_w)), min(h, int(cy + half_h))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_path.with_suffix(".tmp")
    im.crop((x0, y0, x1, y1)).save(tmp, "JPEG", quality=90)
    tmp.replace(out_path)
    return x1 - x0, y1 - y0
