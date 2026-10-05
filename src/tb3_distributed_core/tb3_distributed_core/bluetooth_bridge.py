from __future__ import annotations

import json
import socket
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from tb3_distributed_interfaces.msg import RobotStatus

from .status_codec import status_to_dict


class BluetoothMaintenanceBridge(Node):
    """Short-range commissioning/diagnostics channel over Bluetooth RFCOMM.

    This interface intentionally does not provide normal teleoperation.  It is
    for a technician standing near the robot when Wi-Fi is not yet configured
    or the primary network path is unavailable.
    """

    HELP = (
        "HELP | PING | GET_IP | GET_STATUS | TEST_LIDAR | TEST_CAMERA | "
        "TEST_MOTOR | ESTOP | CLEAR_ESTOP"
    )

    def __init__(self) -> None:
        super().__init__("bluetooth_maintenance_bridge")
        if not hasattr(socket, "AF_BLUETOOTH"):
            raise RuntimeError("Python on this host does not expose AF_BLUETOOTH")
        self.declare_parameter("channel", 1)
        channel = int(self.get_parameter("channel").value)

        self.status_lock = threading.Lock()
        self.latest_status: Optional[RobotStatus] = None
        self.create_subscription(RobotStatus, "/robot/status", self._on_status, 10)
        self.estop_pub = self.create_publisher(Bool, "/safety/estop_cmd", 10)

        self.stop_event = threading.Event()
        self.server_sock = None
        self.worker = threading.Thread(target=self._server, args=(channel,), daemon=True)
        self.worker.start()
        self.get_logger().info(
            f"Bluetooth RFCOMM maintenance server on channel {channel}"
        )

    def _on_status(self, msg: RobotStatus) -> None:
        with self.status_lock:
            self.latest_status = msg

    def _status(self):
        with self.status_lock:
            return self.latest_status

    @staticmethod
    def _local_ip() -> str:
        # No external traffic is sent; connect() only chooses a local interface.
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
        except OSError:
            return socket.gethostbyname(socket.gethostname())
        finally:
            s.close()

    def _handle(self, line: str) -> str:
        cmd = line.strip().upper()
        if cmd in {"", "HELP"}:
            return self.HELP
        if cmd == "PING":
            return "PONG"
        if cmd == "GET_IP":
            return self._local_ip()
        if cmd == "ESTOP":
            self.estop_pub.publish(Bool(data=True))
            return "OK ESTOP requested"
        if cmd == "CLEAR_ESTOP":
            self.estop_pub.publish(Bool(data=False))
            return "OK CLEAR_ESTOP requested"

        status = self._status()
        if status is None:
            return "ERROR robot status unavailable"
        data = status_to_dict(status)
        if cmd == "GET_STATUS":
            return json.dumps(data, separators=(",", ":"))
        if cmd == "TEST_LIDAR":
            return (
                f"LIDAR {'OK' if data['health']['lidar'] else 'FAIL'} "
                f"front={data['distance']['front_m']}m min={data['distance']['min_m']}m"
            )
        if cmd == "TEST_CAMERA":
            return f"CAMERA {'OK' if data['health']['camera'] else 'FAIL'}"
        if cmd == "TEST_MOTOR":
            return f"MOTOR/ODOM {'OK' if data['health']['motor'] else 'FAIL'}"
        return f"ERROR unknown command. {self.HELP}"

    def _server(self, channel: int) -> None:
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.server_sock = sock
        sock.bind((socket.BDADDR_ANY, channel))
        sock.listen(1)
        sock.settimeout(0.5)
        while not self.stop_event.is_set():
            try:
                conn, _addr = sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                conn.settimeout(0.5)
                buffer = b""
                try:
                    conn.sendall((self.HELP + "\n").encode())
                except OSError:
                    continue
                while not self.stop_event.is_set():
                    try:
                        chunk = conn.recv(1024)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        reply = self._handle(line.decode("utf-8", errors="ignore"))
                        try:
                            conn.sendall((reply + "\n").encode("utf-8"))
                        except OSError:
                            break

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.server_sock is not None:
            try:
                self.server_sock.close()
            except OSError:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BluetoothMaintenanceBridge()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
