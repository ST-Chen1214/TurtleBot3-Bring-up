from __future__ import annotations

import math
from dataclasses import replace

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool

from tb3_distributed_interfaces.srv import SetFloat

from .safety_logic import SafetyConfig, evaluate_safety, min_valid_range, sector_min_range


class SafetySupervisor(Node):
    """Fail-closed velocity gate between remote teleoperation and TurtleBot3.

    The operator command is *never* connected directly to the base controller.
    The only command published to /cmd_vel is the command approved here.
    """

    def __init__(self) -> None:
        super().__init__("safety_supervisor")

        self.declare_parameter("raw_cmd_topic", "/teleop/cmd_vel_raw")
        self.declare_parameter("safe_cmd_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("mode_topic", "/robot/mode")
        self.declare_parameter("stop_distance_m", 0.35)
        self.declare_parameter("front_sector_half_angle_deg", 15.0)
        self.declare_parameter("command_timeout_s", 0.50)
        self.declare_parameter("scan_timeout_s", 0.50)
        self.declare_parameter("fail_closed_on_scan_loss", True)
        self.declare_parameter("max_linear_mps", 0.22)
        self.declare_parameter("max_angular_rps", 2.84)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("initial_mode", "TELEOP")

        self.cfg = SafetyConfig(
            stop_distance_m=float(self.get_parameter("stop_distance_m").value),
            command_timeout_s=float(self.get_parameter("command_timeout_s").value),
            scan_timeout_s=float(self.get_parameter("scan_timeout_s").value),
            fail_closed_on_scan_loss=bool(
                self.get_parameter("fail_closed_on_scan_loss").value
            ),
            max_linear_mps=float(self.get_parameter("max_linear_mps").value),
            max_angular_rps=float(self.get_parameter("max_angular_rps").value),
        )
        self.front_half_angle = math.radians(
            float(self.get_parameter("front_sector_half_angle_deg").value)
        )

        raw_topic = str(self.get_parameter("raw_cmd_topic").value)
        safe_topic = str(self.get_parameter("safe_cmd_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        mode_topic = str(self.get_parameter("mode_topic").value)

        self.safe_pub = self.create_publisher(TwistStamped, safe_topic, 10)
        self.blocked_pub = self.create_publisher(Bool, "/safety/blocked", 10)
        self.reason_pub = self.create_publisher(String, "/safety/reason", 10)
        self.distance_pub = self.create_publisher(Float32, "/safety/min_distance", 10)
        self.front_distance_pub = self.create_publisher(
            Float32, "/safety/front_distance", 10
        )
        self.estop_state_pub = self.create_publisher(Bool, "/safety/estop_state", 10)
        self.max_speed_pub = self.create_publisher(Float32, "/robot/max_linear_speed", 10)

        self.create_subscription(TwistStamped, raw_topic, self._on_cmd, 10)
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Bool, "/safety/estop_cmd", self._on_estop_topic, 10)
        self.create_subscription(String, mode_topic, self._on_mode, 10)

        self.create_service(SetBool, "/safety/set_estop", self._srv_set_estop)
        self.create_service(SetFloat, "/safety/set_max_speed", self._srv_set_max_speed)

        self.last_cmd = TwistStamped()
        self.last_cmd_time_ns = None
        self.last_scan_time_ns = None
        self.min_distance_m = None
        self.front_distance_m = None
        self.estop_latched = False
        self.mode = str(self.get_parameter("initial_mode").value).upper()
        self.last_reason = None

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Safety supervisor ready: {raw_topic} -> {safe_topic}, "
            f"stop_distance={self.cfg.stop_distance_m:.2f} m, mode={self.mode}"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_cmd(self, msg: TwistStamped) -> None:
        self.last_cmd = msg
        self.last_cmd_time_ns = self._now_ns()

    def _on_scan(self, msg: LaserScan) -> None:
        self.min_distance_m = min_valid_range(msg.ranges, msg.range_min, msg.range_max)
        self.front_distance_m = sector_min_range(
            msg.ranges,
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            center_angle=0.0,
            half_width=self.front_half_angle,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )
        self.last_scan_time_ns = self._now_ns()

    def _on_estop_topic(self, msg: Bool) -> None:
        self._set_estop(bool(msg.data))

    def _on_mode(self, msg: String) -> None:
        self.mode = str(msg.data).strip().upper() or "IDLE"

    def _set_estop(self, engaged: bool) -> None:
        previous = self.estop_latched
        self.estop_latched = bool(engaged)
        if previous != self.estop_latched:
            state = "ENGAGED" if self.estop_latched else "CLEARED"
            self.get_logger().warning(f"E-stop {state}")

    def _srv_set_estop(self, request: SetBool.Request, response: SetBool.Response):
        self._set_estop(bool(request.data))
        response.success = True
        response.message = "E-stop engaged" if request.data else "E-stop cleared"
        return response

    def _srv_set_max_speed(self, request: SetFloat.Request, response: SetFloat.Response):
        requested = float(request.value)
        if not math.isfinite(requested) or requested <= 0.0:
            response.success = False
            response.message = "max speed must be positive and finite"
            return response
        # Never permit a network request to exceed the platform-configured limit.
        hard_limit = float(self.get_parameter("max_linear_mps").value)
        requested = min(requested, hard_limit)
        self.cfg = replace(self.cfg, max_linear_mps=requested)
        response.success = True
        response.message = f"max linear speed set to {requested:.3f} m/s"
        return response

    def _age_s(self, timestamp_ns):
        if timestamp_ns is None:
            return None
        # Protect against /clock resets in simulation.
        return max(0.0, (self._now_ns() - timestamp_ns) / 1e9)

    @staticmethod
    def _float_msg(value) -> Float32:
        msg = Float32()
        msg.data = (
            float(value)
            if value is not None and math.isfinite(float(value))
            else float("nan")
        )
        return msg

    def _tick(self) -> None:
        decision = evaluate_safety(
            linear_x=self.last_cmd.twist.linear.x,
            angular_z=self.last_cmd.twist.angular.z,
            command_age_s=self._age_s(self.last_cmd_time_ns),
            scan_age_s=self._age_s(self.last_scan_time_ns),
            min_distance_m=self.min_distance_m,
            estop_latched=self.estop_latched,
            config=self.cfg,
            mode=self.mode,
        )

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist.linear.x = decision.linear_x
        out.twist.angular.z = decision.angular_z
        self.safe_pub.publish(out)

        blocked = Bool(data=decision.blocked)
        reason = String(data=decision.reason)
        estop = Bool(data=self.estop_latched)
        max_speed = Float32(data=float(self.cfg.max_linear_mps))
        self.blocked_pub.publish(blocked)
        self.reason_pub.publish(reason)
        self.distance_pub.publish(self._float_msg(self.min_distance_m))
        self.front_distance_pub.publish(self._float_msg(self.front_distance_m))
        self.estop_state_pub.publish(estop)
        self.max_speed_pub.publish(max_speed)

        if decision.reason != self.last_reason:
            log = self.get_logger().warning if decision.blocked else self.get_logger().info
            log(f"Safety state: {decision.reason}")
            self.last_reason = decision.reason


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
