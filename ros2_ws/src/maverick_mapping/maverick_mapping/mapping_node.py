"""ROS 2 node `mapping_node`: depth + camera info + camera pose -> VoxelMap, 2D grid, frontiers.

Thin shell around maverick_mapping.VoxelMap (the pure core) and ros_conv / voxel_delta.

Topics
  in  /maverick/camera/depth   sensor_msgs/Image 16UC1 mm   (matched by stamp with the pose)
  in  /maverick/camera/info    sensor_msgs/CameraInfo       (intrinsics; latched or 1 Hz)
  in  /maverick/camera/pose    geometry_msgs/PoseStamped    (optical frame in the map frame)
  in  /maverick/map/voxels_request  std_msgs/Empty          (send a full reset snapshot)
  out /maverick/map/grid       nav_msgs/OccupancyGrid       1 Hz, latched, frame `map`
  out /maverick/map/frontiers  std_msgs/String JSON         1 Hz, latched: [{id, position, ...}]
  out /maverick/map/voxels     std_msgs/String JSON         <= 5 Hz: occupied-voxel deltas (see
                               voxel_delta.Delta.to_json); a `reset` snapshot at start, on request
                               and every `snapshot_period_s`

Parameters: resolution (0.1), max_range (5.0), max_rate_hz (5.0), stride (1), sync_tol_s (0.03),
grid_rate_hz (1.0), delta_rate_hz (4.0), band [0.8, 1.6] (2D projection band, m), altitude (1.2,
z of the frontier markers), min_cluster (3), snapshot_period_s (30), dump_path (/tmp/map_dump.npz).
Service /maverick/map/dump (std_srvs/Trigger) writes the voxel map to dump_path (map_io, for the
referee's scoring); the response message is the path.
"""

from __future__ import annotations

import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from . import topics as T
from .frontiers import detect_frontiers
from .map_io import save_map
from .ros_conv import (
    RateLimiter,
    SyncBuffer,
    depth_to_metres,
    grid_to_fields,
    info_to_intrinsics,
    intrinsics_for,
    pose_to_tr,
    stamp_to_sec,
)
from .voxel_delta import DeltaEncoder
from .voxel_map import VoxelMap

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
RELIABLE = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)


class MappingNode(Node):
    def __init__(self) -> None:
        super().__init__("mapping_node")
        p = self.declare_parameter
        self.vmap = VoxelMap(p("resolution", 0.1).value, max_range=float(p("max_range", 5.0).value))
        self.stride = int(p("stride", 1).value)
        self.band = tuple(float(v) for v in p("band", [0.8, 1.6]).value)
        self.altitude = float(p("altitude", 1.2).value)
        self.min_cluster = int(p("min_cluster", 3).value)
        self.snapshot_period = float(p("snapshot_period_s", 30.0).value)
        self.sync = SyncBuffer(["depth", "pose"], tol=float(p("sync_tol_s", 0.03).value))
        self.limiter = RateLimiter(float(p("max_rate_hz", 5.0).value), time.monotonic)
        self.encoder = DeltaEncoder()
        self.intr = None
        self.seq = 0
        self.last_stamp = 0.0
        self.last_snapshot = time.monotonic()
        self.integrate_ms = 0.0

        self.create_subscription(Image, T.CAM_DEPTH, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, T.CAM_POSE, self._on_pose, qos_profile_sensor_data)
        for q in (
            qos_profile_sensor_data,
        ):  # best-effort only: a latched sub makes the bridge request incompatible QoS
            self.create_subscription(CameraInfo, T.CAM_INFO, self._on_info, q)
        self.create_subscription(Empty, T.MAP_VOXELS_REQUEST, self._on_request, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, T.MAP_GRID, LATCHED)
        self.front_pub = self.create_publisher(String, T.MAP_FRONTIERS, LATCHED)
        self.vox_pub = self.create_publisher(String, T.MAP_VOXELS, RELIABLE)
        self.create_timer(1.0 / float(p("grid_rate_hz", 1.0).value), self._publish_grid)
        self.create_timer(1.0 / float(p("delta_rate_hz", 4.0).value), self._publish_delta)
        self._want_reset = True
        self.dump_path = str(p("dump_path", "/tmp/map_dump.npz").value)
        self.create_service(Trigger, T.MAP_DUMP, self._on_dump)
        self.get_logger().info("mapping_node up")

    # -- inputs ------------------------------------------------------------------------------
    def _on_info(self, msg: CameraInfo) -> None:
        self.intr = info_to_intrinsics(msg)

    def _on_request(self, _msg: Empty) -> None:
        self._want_reset = True

    def _on_dump(self, _req, resp):
        try:
            save_map(self.vmap, self.dump_path)
            resp.success, resp.message = True, self.dump_path
        except Exception as exc:  # noqa: BLE001 - report to the caller, keep the node alive
            resp.success, resp.message = False, repr(exc)
        return resp

    def _on_depth(self, msg: Image) -> None:
        self._feed("depth", msg)

    def _on_pose(self, msg) -> None:
        self._feed("pose", msg)

    def _feed(self, name: str, msg) -> None:
        for m in self.sync.push(name, stamp_to_sec(msg.header.stamp), msg):
            self._integrate(m["depth"], m["pose"])

    def _integrate(self, depth_msg: Image, pose_msg) -> None:
        if self.intr is None or not self.limiter.ready():
            return
        t0 = time.perf_counter()
        depth = depth_to_metres(depth_msg)
        intr = intrinsics_for(self.intr, depth.shape[1], depth.shape[0])
        t_wc, r_wc = pose_to_tr(pose_msg)
        self.vmap.integrate_depth(depth, intr, t_wc, r_wc, stride=self.stride)
        self.last_stamp = stamp_to_sec(depth_msg.header.stamp)
        self.integrate_ms = 1e3 * (time.perf_counter() - t0)

    # -- outputs -----------------------------------------------------------------------------
    def _publish_grid(self) -> None:
        if self.vmap.frames == 0:
            return
        grid, res, org = self.vmap.project_2d(*self.band)
        f = grid_to_fields(grid, res, org)
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = f["resolution"]
        msg.info.width, msg.info.height = f["width"], f["height"]
        msg.info.origin.position.x, msg.info.origin.position.y = f["origin_x"], f["origin_y"]
        msg.info.origin.orientation.w = 1.0
        msg.data = f["data"]
        self.grid_pub.publish(msg)
        fronts = detect_frontiers(grid, res, org, self.min_cluster)
        out = [
            {
                "id": f"f{i}",
                "position": [round(fr.centroid[0], 3), round(fr.centroid[1], 3), self.altitude],
                "size": int(fr.size),
                "gain": int(fr.unknown_gain),
            }
            for i, fr in enumerate(fronts)
        ]
        self.front_pub.publish(String(data=json.dumps(out)))

    def _publish_delta(self) -> None:
        if self.vmap.frames == 0:
            return
        now = time.monotonic()
        reset = self._want_reset or now - self.last_snapshot >= self.snapshot_period
        d = self.encoder.delta(self.vmap, reset=reset)
        if reset:
            self._want_reset = False
            self.last_snapshot = now
        if d.empty():
            return
        self.seq += 1
        payload = d.to_json(self.vmap.res, self.vmap._lattice, time.time(), self.seq)
        self.vox_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
