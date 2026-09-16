"""World (Gazebo, ENU) <-> drone map frame. The only place the transform lives.

The drone's map frame is ROS ENU with its origin at the PX4 local origin, i.e. the point where
the EKF started (the spawn point). PX4 local NED is earth-fixed (north = world +y), so the two
frames differ by a translation only, no rotation: the spawn yaw does not rotate the map.
[H] to be verified against ground truth in the first full-stack run.

`origin` is (x, y, z) of the spawn in the world frame: x, y from the manifest `spawn.pose`, z
the ground-truth height of the resting drone (0 if unknown).
"""

from __future__ import annotations

Vec3 = tuple[float, float, float]


def spawn_origin(manifest: dict, ground_z: float = 0.0) -> Vec3:
    pose = manifest["spawn"]["pose"]
    return (float(pose[0]), float(pose[1]), float(ground_z))


def world_to_map(p, origin: Vec3) -> Vec3:
    z = p[2] if len(p) > 2 else 0.0
    return (p[0] - origin[0], p[1] - origin[1], z - origin[2])


def map_to_world(p, origin: Vec3) -> Vec3:
    z = p[2] if len(p) > 2 else 0.0
    return (p[0] + origin[0], p[1] + origin[1], z + origin[2])


def model_pose_env(manifest: dict) -> str:
    """PX4_GZ_MODEL_POSE ("x,y,z,roll,pitch,yaw") that spawns the drone at the manifest spawn."""
    x, y, z, yaw = (float(v) for v in manifest["spawn"]["pose"])
    return f"{x:.4f},{y:.4f},{z:.4f},0,0,{yaw:.4f}"
