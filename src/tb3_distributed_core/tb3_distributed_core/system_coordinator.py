from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tb3_distributed_interfaces.srv import SetMode


class SystemCoordinator(Node):
    """Own the high-level robot operating mode.

    This node is an *application-level* coordinator, not a ROS 2 server/master.
    ROS 2 remains distributed across all Raspberry Pis.
    """

    VALID_MODES = {"TELEOP", "IDLE", "MAINTENANCE"}

    def __init__(self) -> None:
        super().__init__("system_coordinator")
        self.declare_parameter("initial_mode", "TELEOP")
        requested = str(self.get_parameter("initial_mode").value).upper()
        self.mode = requested if requested in self.VALID_MODES else "IDLE"

        self.mode_pub = self.create_publisher(String, "/robot/mode", 10)
        self.create_service(SetMode, "/robot/set_mode", self._set_mode)
        self.create_timer(0.5, self._publish_mode)
        self._publish_mode()
        self.get_logger().info(f"System coordinator ready; mode={self.mode}")

    def _publish_mode(self) -> None:
        self.mode_pub.publish(String(data=self.mode))

    def _set_mode(self, request: SetMode.Request, response: SetMode.Response):
        mode = str(request.mode).strip().upper()
        if mode not in self.VALID_MODES:
            response.success = False
            response.message = (
                f"invalid mode {mode!r}; valid={sorted(self.VALID_MODES)}"
            )
            return response
        self.mode = mode
        self._publish_mode()
        response.success = True
        response.message = f"mode set to {mode}"
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
