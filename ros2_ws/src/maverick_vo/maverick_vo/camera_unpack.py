"""Host node `camera_unpack`: compressed camera topics from the link -> raw images for rtabmap.

  in  /maverick/camera/rgb/compressed   CompressedImage "jpeg"
  in  /maverick/camera/depth/png        CompressedImage "16UC1; png"
  in  /maverick/camera/semantic/png     CompressedImage "mono8; png"
  out /maverick/camera/rgb              Image rgb8       (same stamp and frame as the input)
  out /maverick/camera/depth            Image 16UC1 mm   (0 = invalid)
  out /maverick/camera/semantic         Image mono8      (class ids, 160x120, simulation only)

These raw topics exist only on the host (they are not in the Zenoh allow-lists), so the wifi link
carries the compressed forms only. mapping_node, memory_node and rgbd_odometry subscribe here.
"""

from __future__ import annotations

import sys

import numpy as np
import rclpy
from maverick_bridge.image_codec import decode_jpeg_rgb, decode_png
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def to_image(header, arr: np.ndarray, encoding: str) -> Image:
    arr = np.ascontiguousarray(arr)
    m = Image()
    m.header = header
    m.height, m.width = int(arr.shape[0]), int(arr.shape[1])
    m.encoding = encoding
    m.is_bigendian = 0
    m.step = int(arr.strides[0])
    m.data = arr.tobytes()
    return m


class CameraUnpack(Node):
    def __init__(self) -> None:
        super().__init__("camera_unpack")
        q = qos_profile_sensor_data
        self.pub_rgb = self.create_publisher(Image, "/maverick/camera/rgb", q)
        self.pub_depth = self.create_publisher(Image, "/maverick/camera/depth", q)
        self.pub_sem = self.create_publisher(Image, "/maverick/camera/semantic", q)
        self.create_subscription(CompressedImage, "/maverick/camera/rgb/compressed", self._rgb, q)
        self.create_subscription(CompressedImage, "/maverick/camera/depth/png", self._depth, q)
        self.create_subscription(CompressedImage, "/maverick/camera/semantic/png", self._sem, q)
        self.errors = 0

    def _guard(self, fn, msg) -> None:
        try:
            fn(msg)
        except Exception as exc:  # noqa: BLE001 - a corrupt frame must not stop the node
            self.errors += 1
            if self.errors <= 5:
                self.get_logger().warning(f"undecodable camera frame: {exc!r}")

    def _rgb(self, msg: CompressedImage) -> None:
        self._guard(
            lambda m: self.pub_rgb.publish(
                to_image(m.header, decode_jpeg_rgb(bytes(m.data)), "rgb8")
            ),
            msg,
        )

    def _depth(self, msg: CompressedImage) -> None:
        self._guard(
            lambda m: self.pub_depth.publish(
                to_image(m.header, decode_png(bytes(m.data), np.uint16), "16UC1")
            ),
            msg,
        )

    def _sem(self, msg: CompressedImage) -> None:
        self._guard(
            lambda m: self.pub_sem.publish(
                to_image(m.header, decode_png(bytes(m.data), np.uint8), "mono8")
            ),
            msg,
        )


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    node = CameraUnpack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
