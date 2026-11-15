"""Put the drone stack's pure-Python packages, worldgen and the referee on sys.path.

The same trick as the packages' conftest.py files: nothing is installed, so kinsim runs on the
checked-out sources (and therefore always on the code under test).
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
_SRC = ROOT / "ros2_ws" / "src"

for _p in (
    _SRC / "maverick_mapping",
    _SRC / "maverick_memory",
    _SRC / "maverick_skills",
    _SRC / "maverick_flight",
    ROOT / "sim" / "worldgen",
    ROOT / "sim" / "referee",
    ROOT / "sim" / "kinsim",
):
    if _p.is_dir() and str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
