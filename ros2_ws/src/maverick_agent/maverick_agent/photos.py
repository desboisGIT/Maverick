"""Photo attachments for `transcript` events (docs/gateway-events.md). Pure Python, no ROS.

`send_photo(object_id)` sends the crop memory saved for that object; `current_view` sends the
latest camera frame. The attachment carries `object_id` and `label` (optional fields) so the
referee and the console can tell what the photo claims to show.
"""

from __future__ import annotations

import base64
from pathlib import Path

MAX_PHOTO_BYTES = 400_000  # a crop is a few KB; refuse anything absurd


def photo_attachment(
    jpeg: bytes, caption: str, object_id: str | None = None, label: str | None = None
) -> dict:
    att = {
        "kind": "photo",
        "mime": "image/jpeg",
        "data": base64.b64encode(jpeg).decode("ascii"),
        "caption": caption,
    }
    if object_id:
        att["object_id"] = object_id
    if label:
        att["label"] = label
    return att


def read_photo(path: str | Path) -> bytes | None:
    """The JPEG bytes at `path`, or None when missing, empty, too large or not a JPEG."""
    try:
        data = Path(path).read_bytes()
    except OSError:
        return None
    if not data or len(data) > MAX_PHOTO_BYTES or data[:2] != b"\xff\xd8":
        return None
    return data


def strip_attachment_data(event_data: dict) -> dict:
    """A copy of a transcript event with the base64 payload replaced by its length (for logs)."""
    out = dict(event_data)
    if out.get("attachments"):
        out["attachments"] = [
            {**{k: v for k, v in a.items() if k != "data"}, "data_len": len(a.get("data", ""))}
            for a in out["attachments"]
        ]
    return out
