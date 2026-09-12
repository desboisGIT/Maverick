"""Put the package root on sys.path so tests import without install (importlib import mode)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
