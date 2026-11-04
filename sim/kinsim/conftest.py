"""Put the kinsim package root on sys.path so tests import without install."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
