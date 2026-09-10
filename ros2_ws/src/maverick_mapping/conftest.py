"""Puts the package root and the worldgen library (sim/worldgen) on sys.path for the tests."""

import sys
from pathlib import Path

_here = Path(__file__).resolve().parent
for p in (_here, _here.parents[2] / "sim" / "worldgen"):
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
