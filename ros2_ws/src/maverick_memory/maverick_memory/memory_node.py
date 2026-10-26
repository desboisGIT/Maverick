"""ROS 2 node `memory_node`: simulated detector -> SceneGraph (objects, rooms), queries.

Thin shell around maverick_memory.SceneGraph and SemanticDetector (pure cores).

Topics
  in  /maverick/camera/semantic  sensor_msgs/Image mono8   worldgen class ids (simulation only)
  in  /maverick/camera/depth     sensor_msgs/Image 16UC1 mm
  in  /maverick/camera/info      sensor_msgs/CameraInfo
  in  /maverick/camera/pose      geometry_msgs/PoseStamped
  in  /maverick/map/grid         nav_msgs/OccupancyGrid    (rooms are segmented from it, ~1 Hz)
  in  /maverick/memory/query     std_msgs/String JSON      {"id", "query": goal_for_type|find|
                                 room_of, ...} (see queries.answer_query)
  out /maverick/memory/answer    std_msgs/String JSON      reply, echoes "id"
  out /maverick/memory/detections std_msgs/String JSON     per frame: {stamp, boxes[]} (normalised)
  out /maverick/memory/rooms     std_msgs/String JSON      gateway `rooms` list, 1 Hz, latched
  out /maverick/memory/objects   std_msgs/String JSON      gateway `objects` list, 1 Hz, latched

Parameters: max_rate_hz (5.0), sync_tol_s (0.03), rooms_rate_hz (1.0), seed (0), miss_rate,
fp_rate, box_jitter, mask_dropout (detector noise; defaults = DetectorNoise()).
RGB (for crops) is not used yet.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from maverick_mapping import topics as T
from maverick_mapping.drone import drone_config
from maverick_mapping.ros_conv import (
    RateLimiter,
    SyncBuffer,
    depth_to_metres,
    image_to_array,
    info_to_intrinsics,
    intrinsics_for,
    occupancy_to_grid,
    pose_to_tr,
    stamp_to_sec,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from .lifting import lift_detection
from .photos import PHOTO_LABELS, FrameBuffer, crop_jpeg
from .queries import answer_query, detections_json
from .scene_graph import SceneGraph, SceneGraphConfig
from .semantic_detector import SemanticDetector
from .sim_detector import DetectorNoise

LATCHED = QoSProfile(
    depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
)
RELIABLE = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)


class MemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("memory_node")
        p = self.declare_parameter
        noise = DetectorNoise(
            miss_rate=float(p("miss_rate", 0.10).value),
            fp_rate=float(p("fp_rate", 0.02).value),
            box_jitter=float(p("box_jitter", 0.05).value),
            mask_dropout=float(p("mask_dropout", 0.05).value),
        )
        self.detector = SemanticDetector(noise, seed=int(p("seed", 0).value))
        drone = drone_config(str(p("drone", "").value) or None)  # "" -> MAVERICK_DRONE
        self.sg = SceneGraph(
            SceneGraphConfig(
                robot_clear=drone.goal_clear,
                connect_clear=drone.connect_clear,
                wall_clear=drone.wall_clear,
            )
        )
        self.photo_dir = Path(str(p("photo_dir", "/tmp/maverick_photos").value))
        self.photo_min_conf = float(p("photo_min_conf", 0.5).value)
        self.frames = FrameBuffer()
        self.sync = SyncBuffer(
            ["semantic", "depth", "pose"], tol=float(p("sync_tol_s", 0.03).value)
        )
        self.limiter = RateLimiter(float(p("max_rate_hz", 5.0).value), time.monotonic)
        self.intr = None
        self.robot_xy: tuple[float, float] | None = None
        self._grid_msg: OccupancyGrid | None = None
        self._grid_done = None
        self._t_now = 0.0

        q = qos_profile_sensor_data
        self.create_subscription(Image, T.CAM_SEMANTIC, lambda m: self._feed("semantic", m), q)
        self.create_subscription(Image, T.CAM_DEPTH, lambda m: self._feed("depth", m), q)
        self.create_subscription(PoseStamped, T.CAM_POSE, lambda m: self._feed("pose", m), q)
        self.create_subscription(CompressedImage, T.CAM_RGB, self._on_rgb, q)
        for qos in (q,):  # sensor-data QoS only (see mapping_node)
            self.create_subscription(CameraInfo, T.CAM_INFO, self._on_info, qos)
        self.create_subscription(OccupancyGrid, T.MAP_GRID, self._on_grid, LATCHED)
        self.create_subscription(String, T.MEM_QUERY, self._on_query, RELIABLE)
        self.ans_pub = self.create_publisher(String, T.MEM_ANSWER, RELIABLE)
        self.det_pub = self.create_publisher(String, T.MEM_DETECTIONS, RELIABLE)
        self.rooms_pub = self.create_publisher(String, T.MEM_ROOMS, LATCHED)
        self.objects_pub = self.create_publisher(String, T.MEM_OBJECTS, LATCHED)
        self.create_timer(1.0 / float(p("rooms_rate_hz", 1.0).value), self._update_rooms)
        self.get_logger().info("memory_node up")

    def _on_info(self, msg: CameraInfo) -> None:
        self.intr = info_to_intrinsics(msg)

    def _on_grid(self, msg: OccupancyGrid) -> None:
        self._grid_msg = msg

    def _on_rgb(self, msg: CompressedImage) -> None:
        self.frames.add(stamp_to_sec(msg.header.stamp), bytes(msg.data))

    def _feed(self, name: str, msg) -> None:
        for m in self.sync.push(name, stamp_to_sec(msg.header.stamp), msg):
            self._process(m["semantic"], m["depth"], m["pose"])

    def _process(self, sem_msg: Image, depth_msg: Image, pose_msg: PoseStamped) -> None:
        if self.intr is None or not self.limiter.ready():
            return
        depth = depth_to_metres(depth_msg)
        sem = image_to_array(sem_msg)
        if sem.shape != depth.shape:
            # depth is published at 2x the semantic image (320x240 vs 160x120): take every
            # f-th pixel (nearest sample) so both share one pixel grid
            fy, fx = depth.shape[0] // sem.shape[0], depth.shape[1] // sem.shape[1]
            if fy < 1 or fy != fx or depth.shape[0] != fy * sem.shape[0]:
                return
            depth = depth[fy // 2 :: fy, fx // 2 :: fx]
        intr = intrinsics_for(self.intr, depth.shape[1], depth.shape[0])
        t_wc, r_wc = pose_to_tr(pose_msg)
        stamp = stamp_to_sec(depth_msg.header.stamp)
        self.robot_xy = (float(t_wc[0]), float(t_wc[1]))
        self._t_now = stamp
        dets = self.detector.detect(sem, depth)
        obs = [lift_detection(d, depth, intr, t_wc, r_wc, t=stamp) for d in dets]
        self.sg.add_observations([o for o in obs if o is not None], stamp)
        self._save_photos(dets, obs, depth.shape[1], depth.shape[0], stamp)
        pos = [None if o is None else o.center for o in obs]
        payload = detections_json(dets, depth.shape[1], depth.shape[0], stamp, pos)
        payload["stamp_ns"] = int(depth_msg.header.stamp.sec) * 10**9 + int(
            depth_msg.header.stamp.nanosec
        )
        self.det_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def _save_photos(self, dets, obs, width: int, height: int, stamp: float) -> None:
        """Keep the most confident crop of each small object / person as its photo reference."""
        for d, o in zip(dets, obs, strict=True):
            if o is None or d.box is None or o.label not in PHOTO_LABELS:
                continue
            if d.confidence < self.photo_min_conf:
                continue
            obj = self.sg.store.match(o)
            if obj is None or d.confidence <= obj.extra.get("photo_conf", 0.0):
                continue
            jpeg = self.frames.nearest(stamp)
            if jpeg is None:
                continue
            u0, v0, u1, v1 = d.box
            box = (u0 / width, v0 / height, (u1 - u0 + 1) / width, (v1 - v0 + 1) / height)
            path = self.photo_dir / f"{obj.id}.jpg"
            if crop_jpeg(jpeg, box, path) is not None:
                obj.extra["photo"], obj.extra["photo_conf"] = str(path), float(d.confidence)

    def _update_rooms(self) -> None:
        if self._grid_msg is not None and self._grid_msg is not self._grid_done:
            grid, res, org = occupancy_to_grid(self._grid_msg)
            self._grid_done = self._grid_msg
            if grid.size:
                self.sg.update_rooms(grid, res, org)
        else:
            self.sg.retype()
        if self.robot_xy is not None:
            self.sg.visit(self.robot_xy)
        self.rooms_pub.publish(String(data=json.dumps(self.sg.gateway_rooms())))
        self.objects_pub.publish(String(data=json.dumps(self.sg.gateway_objects())))

    def _on_query(self, msg: String) -> None:
        try:
            q = json.loads(msg.data)
            ans = answer_query(self.sg, q, self.robot_xy)
        except (ValueError, TypeError, KeyError) as exc:
            ans = {"id": None, "ok": False, "reason": f"bad query: {exc}"}
        self.ans_pub.publish(String(data=json.dumps(ans)))


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = MemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
