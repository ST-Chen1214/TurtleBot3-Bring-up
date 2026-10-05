from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool


class SensorHealth(Node):
    """Publish lightweight sensor-liveness topics without forwarding camera frames."""

    def __init__(self) -> None:
        super().__init__("sensor_health")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("timeout_s", 1.5)
        self.declare_parameter("camera_expected", True)

        self.timeout_ns = int(float(self.get_parameter("timeout_s").value) * 1e9)
        self.camera_expected = bool(self.get_parameter("camera_expected").value)
        self.last_scan_ns = None
        self.last_image_ns = None

        self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        if self.camera_expected:
            self.create_subscription(
                Image,
                str(self.get_parameter("image_topic").value),
                self._on_image,
                qos_profile_sensor_data,
            )

        self.lidar_pub = self.create_publisher(Bool, "/health/lidar", 10)
        self.camera_pub = self.create_publisher(Bool, "/health/camera", 10)
        self.create_timer(0.5, self._tick)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_scan(self, _msg: LaserScan) -> None:
        self.last_scan_ns = self._now_ns()

    def _on_image(self, _msg: Image) -> None:
        self.last_image_ns = self._now_ns()

    def _alive(self, timestamp_ns) -> bool:
        if timestamp_ns is None:
            return False
        return 0 <= (self._now_ns() - timestamp_ns) <= self.timeout_ns

    def _tick(self) -> None:
        self.lidar_pub.publish(Bool(data=self._alive(self.last_scan_ns)))
        camera_alive = self._alive(self.last_image_ns) if self.camera_expected else False
        self.camera_pub.publish(Bool(data=camera_alive))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorHealth()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
