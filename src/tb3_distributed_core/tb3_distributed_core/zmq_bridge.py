from __future__ import annotations

import json
import rclpy
from rclpy.node import Node

from tb3_distributed_interfaces.msg import RobotStatus


from .status_codec import status_to_dict


class ZmqTelemetryBridge(Node):
    """Publish asynchronous robot telemetry to non-ROS applications via ZMQ PUB."""

    def __init__(self) -> None:
        super().__init__("zmq_telemetry_bridge")
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError("pyzmq is required: pip install pyzmq") from exc
        self.zmq = zmq

        self.declare_parameter("bind", "tcp://*:5555")
        self.declare_parameter("snd_hwm", 10)
        bind = str(self.get_parameter("bind").value)

        self.context = zmq.Context.instance()
        self.sock = self.context.socket(zmq.PUB)
        self.sock.setsockopt(zmq.SNDHWM, int(self.get_parameter("snd_hwm").value))
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.bind(bind)
        self.create_subscription(RobotStatus, "/robot/status", self._on_status, 10)
        self.get_logger().info(f"ZeroMQ telemetry PUB bound to {bind}")

    def _send(self, topic: str, payload) -> None:
        self.sock.send_multipart(
            [topic.encode("utf-8"), json.dumps(payload, separators=(",", ":")).encode("utf-8")]
        )

    def _on_status(self, msg: RobotStatus) -> None:
        data = status_to_dict(msg)
        # Full status plus selective topics let different operator-side apps
        # subscribe only to what they need.
        self._send("robot.status", data)
        self._send("robot.pose", data["pose"])
        self._send("robot.distance", data["distance"])
        self._send("robot.safety", data["safety"])
        self._send(
            "robot.health",
            {"battery_percent": data["battery_percent"], **data["health"]},
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZmqTelemetryBridge()
    try:
        rclpy.spin(node)
    finally:
        node.sock.close(linger=0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
