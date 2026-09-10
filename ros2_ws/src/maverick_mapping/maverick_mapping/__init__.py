"""Voxel mapping, frontiers and exploration planning (pure numpy core, no ROS imports).

Frames: the map frame is ROS REP 103 ENU (x east, y north, z up, right-handed, yaw counter-
clockwise from +x) and equals the worldgen world frame. PX4 NED conversion lives only in frames.py.
Camera optical frame: x right, y down, z forward; a camera pose is (t, R) with R mapping
optical-frame vectors to the map frame. Depth images hold z-depth in metres, NaN or 0 for invalid.
"""

from .voxel_map import CameraIntrinsics, VoxelMap, optical_rotation

__all__ = ["CameraIntrinsics", "VoxelMap", "optical_rotation"]
