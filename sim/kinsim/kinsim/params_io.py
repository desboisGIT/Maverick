"""Load / save KinParams; the calibrated set ships as calibrated.json next to this file."""

from __future__ import annotations

import json
from pathlib import Path

from .vehicle import KinParams

CALIBRATED = Path(__file__).with_name("calibrated.json")


def load_params(path: str | Path | None = None, map_stride: int | None = None) -> KinParams:
    p = Path(path) if path else CALIBRATED
    params = KinParams.from_dict(json.loads(p.read_text())) if p.exists() else KinParams()
    if map_stride is not None:
        params.map_stride = int(map_stride)
    return params


def save_params(params: KinParams, path: str | Path) -> None:
    Path(path).write_text(json.dumps(params.to_dict(), indent=2) + "\n")
