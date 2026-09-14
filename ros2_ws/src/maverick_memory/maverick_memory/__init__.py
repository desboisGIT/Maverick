"""Scene graph for the agent: objects lifted from detections, rooms from the 2D map, room types.

Pure Python + numpy, no ROS imports. Frames: the map frame is ROS REP 103 ENU (x east, y north,
z up), the same as maverick_mapping. Occupancy grids are int8 [ix, iy] with -1 unknown, 0 free,
100 occupied (see maverick_mapping.VoxelMap.project_2d).
"""

from .lifting import Detection, ObjectObservation, lift_detection
from .scene_graph import SceneGraph, SceneGraphConfig
from .store import ObjectStore, StoredObject

__all__ = [
    "Detection",
    "ObjectObservation",
    "ObjectStore",
    "SceneGraph",
    "SceneGraphConfig",
    "StoredObject",
    "lift_detection",
]
