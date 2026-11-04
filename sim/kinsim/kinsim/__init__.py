"""kinsim: a fast kinematic simulator that runs the real drone-stack pure modules in lockstep.

No ROS, no Gazebo: pure Python + numpy. See docs/kinsim.md for what it models, how it was
calibrated against Gazebo runs and when its numbers can be trusted (tuning and screening only,
never reported as S3 / S4 results).
"""

from __future__ import annotations

from . import paths as _paths  # noqa: F401  (puts the drone stack's pure packages on sys.path)

__version__ = "1"
