from __future__ import annotations

import socket
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node

from .command_protocol import parse_velocity_packet


class UdpCommandBridge(Node):
    """Low-latency latest-value teleoperation input.

    UDP is intentionally used only for continuously refreshed velocity/heartbeat
    data.  Sequence numbers reject stale/out-of-order datagrams.
    """

    def __init__(self) -> None:
        super().__init__("udp_command_bridge")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("port", 5005)
        self.declare_parameter("topic", "/teleop/cmd_vel_raw")
        self.declare_parameter("max_linear_mps", 0.22)
        self.declare_parameter("max_angular_rps", 2.84)

        self.max_linear = float(self.get_parameter("max_linear_mps").value)
        self.max_angular = float(self.get_parameter("max_angular_rps").value)
        topic = str(self.get_parameter("topic").value)
        bind_ip = str(self.get_parameter("bind_ip").value)
        port = int(self.get_parameter("port").value)

        self.pub = self.create_publisher(TwistStamped, topic, 10)
        self.last_seq: Dict[Tuple[str, int], int] = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, port))
        self.sock.setblocking(False)
        self.create_timer(0.005, self._poll)
        self.get_logger().info(f"UDP teleop listening on {bind_ip}:{port} -> {topic}")

    def _publish(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def _poll(self) -> None:
        for _ in range(64):
            try:
                payload, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                linear, angular, seq = parse_velocity_packet(
                    payload, self.max_linear, self.max_angular
                )
                if seq is not None:
                    previous = self.last_seq.get(addr)
                    if previous is not None and seq <= previous:
                        continue
                    self.last_seq[addr] = seq
                self._publish(linear, angular)
            except Exception as exc:
                self.get_logger().warning(f"Dropped invalid UDP command from {addr}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UdpCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
