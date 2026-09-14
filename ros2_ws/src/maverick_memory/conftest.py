"""Puts this package, maverick_mapping and the worldgen library on sys.path for the tests."""

import sys
from pathlib import Path

_here = Path(__file__).resolve().parent
for p in (_here, _here.parent / "maverick_mapping", _here.parents[2] / "sim" / "worldgen"):
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
