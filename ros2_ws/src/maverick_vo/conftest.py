"""Puts this package and the pure-Python bridge package on sys.path (tests run uninstalled)."""

import sys
from pathlib import Path

_here = Path(__file__).resolve().parent
for p in (_here, _here.parent / "maverick_bridge"):
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
