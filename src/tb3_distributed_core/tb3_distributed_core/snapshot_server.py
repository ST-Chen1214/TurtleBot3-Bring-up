from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from tb3_distributed_interfaces.srv import GetSnapshot


class SnapshotServer(Node):
    """Keep the latest camera frame and return JPEG bytes on demand via ROS 2 service."""

    def __init__(self) -> None:
        super().__init__("snapshot_server")
        try:
            import cv2
            from cv_bridge import CvBridge
        except ImportError as exc:
            raise RuntimeError(
                "snapshot_server requires cv_bridge and OpenCV "
                "(ros-jazzy-cv-bridge, python3-opencv)"
            ) from exc

        self.cv2 = cv2
        self.bridge = CvBridge()
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("default_jpeg_quality", 85)
        self.declare_parameter("max_frame_age_s", 2.0)

        self.frame_lock = threading.Lock()
        self.latest_msg = None
        self.latest_ns = None

        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_service(GetSnapshot, "/camera/take_snapshot", self._snapshot)
        self.get_logger().info("Snapshot service ready: /camera/take_snapshot")

    def _on_image(self, msg: Image) -> None:
        with self.frame_lock:
            self.latest_msg = msg
            self.latest_ns = self.get_clock().now().nanoseconds

    def _snapshot(self, request: GetSnapshot.Request, response: GetSnapshot.Response):
        with self.frame_lock:
            msg = self.latest_msg
            stamp_ns = self.latest_ns

        if msg is None or stamp_ns is None:
            response.success = False
            response.message = "no camera frame has been received"
            response.content_type = ""
            return response

        age_s = max(0.0, (self.get_clock().now().nanoseconds - stamp_ns) / 1e9)
        max_age = float(self.get_parameter("max_frame_age_s").value)
        if age_s > max_age:
            response.success = False
            response.message = f"latest camera frame is stale ({age_s:.2f}s old)"
            response.content_type = ""
            return response

        quality = int(request.jpeg_quality) or int(
            self.get_parameter("default_jpeg_quality").value
        )
        quality = max(20, min(100, quality))
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, encoded = self.cv2.imencode(
                ".jpg",
                frame,
                [int(self.cv2.IMWRITE_JPEG_QUALITY), quality],
            )
            if not ok:
                raise RuntimeError("cv2.imencode returned false")
            response.success = True
            response.message = f"snapshot captured at JPEG quality {quality}"
            response.content_type = "image/jpeg"
            response.data = encoded.tobytes()
            return response
        except Exception as exc:
            response.success = False
            response.message = f"snapshot encode failed: {exc}"
            response.content_type = ""
            return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SnapshotServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
