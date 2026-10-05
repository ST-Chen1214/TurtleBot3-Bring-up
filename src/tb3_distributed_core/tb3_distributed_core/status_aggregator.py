from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String

from tb3_distributed_interfaces.msg import RobotStatus


class StatusAggregator(Node):
    """Aggregate distributed ROS 2 state into one compact status message."""

    def __init__(self) -> None:
        super().__init__("status_aggregator")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("motor_timeout_s", 1.5)

        self.mode = "UNKNOWN"
        self.safety_blocked = True
        self.safety_reason = "STARTING"
        self.estop = False
        self.front_distance = float("nan")
        self.min_distance = float("nan")
        self.battery_percent = float("nan")
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.max_linear_speed = float("nan")
        self.lidar_alive = False
        self.camera_alive = False
        self.last_odom_ns = None

        self.create_subscription(String, "/robot/mode", self._on_mode, 10)
        self.create_subscription(Bool, "/safety/blocked", self._on_blocked, 10)
        self.create_subscription(String, "/safety/reason", self._on_reason, 10)
        self.create_subscription(Bool, "/safety/estop_state", self._on_estop, 10)
        self.create_subscription(Float32, "/safety/front_distance", self._on_front, 10)
        self.create_subscription(Float32, "/safety/min_distance", self._on_min, 10)
        self.create_subscription(Float32, "/robot/max_linear_speed", self._on_max_speed, 10)
        self.create_subscription(Bool, "/health/lidar", self._on_lidar, 10)
        self.create_subscription(Bool, "/health/camera", self._on_camera, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(BatteryState, "/battery_state", self._on_battery, 10)
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_cmd_vel, 10)

        self.pub = self.create_publisher(RobotStatus, "/robot/status", 10)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._publish)

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_mode(self, msg: String) -> None:
        self.mode = msg.data

    def _on_blocked(self, msg: Bool) -> None:
        self.safety_blocked = bool(msg.data)

    def _on_reason(self, msg: String) -> None:
        self.safety_reason = msg.data

    def _on_estop(self, msg: Bool) -> None:
        self.estop = bool(msg.data)

    def _on_front(self, msg: Float32) -> None:
        self.front_distance = float(msg.data)

    def _on_min(self, msg: Float32) -> None:
        self.min_distance = float(msg.data)

    def _on_max_speed(self, msg: Float32) -> None:
        self.max_linear_speed = float(msg.data)

    def _on_lidar(self, msg: Bool) -> None:
        self.lidar_alive = bool(msg.data)

    def _on_camera(self, msg: Bool) -> None:
        self.camera_alive = bool(msg.data)

    def _on_cmd_vel(self, msg: TwistStamped) -> None:
        self.linear_velocity = float(msg.twist.linear.x)
        self.angular_velocity = float(msg.twist.angular.z)

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom_ns = self._now_ns()
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _on_battery(self, msg: BatteryState) -> None:
        p = float(msg.percentage)
        # BatteryState.percentage should be [0,1]. Keep NaN when unavailable.
        if math.isfinite(p) and 0.0 <= p <= 1.0:
            self.battery_percent = 100.0 * p
        elif math.isfinite(p) and 1.0 < p <= 100.0:
            self.battery_percent = p
        else:
            self.battery_percent = float("nan")

    def _motor_alive(self) -> bool:
        if self.last_odom_ns is None:
            return False
        timeout_ns = int(float(self.get_parameter("motor_timeout_s").value) * 1e9)
        age = self._now_ns() - self.last_odom_ns
        return 0 <= age <= timeout_ns

    def _publish(self) -> None:
        msg = RobotStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        msg.safety_blocked = self.safety_blocked
        msg.safety_reason = self.safety_reason
        msg.estop_engaged = self.estop
        msg.front_distance_m = self.front_distance
        msg.min_distance_m = self.min_distance
        msg.battery_percent = self.battery_percent
        msg.linear_velocity_mps = self.linear_velocity
        msg.angular_velocity_rps = self.angular_velocity
        msg.x_m = self.x
        msg.y_m = self.y
        msg.yaw_rad = self.yaw
        msg.max_linear_speed_mps = self.max_linear_speed
        msg.lidar_alive = self.lidar_alive
        msg.camera_alive = self.camera_alive
        msg.motor_alive = self._motor_alive()
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
