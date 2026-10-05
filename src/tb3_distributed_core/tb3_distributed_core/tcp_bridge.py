from __future__ import annotations

import json
import queue
import socket
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from tb3_distributed_interfaces.srv import SetFloat, SetMode

from .command_protocol import make_reply, parse_reliable_command


@dataclass
class PendingCommand:
    command: Dict[str, Any]
    event: threading.Event = field(default_factory=threading.Event)
    response: Optional[Dict[str, Any]] = None


class TcpReliableCommandBridge(Node):
    """Reliable discrete command channel with application-level ACKs.

    TCP is deliberately *not* used for high-rate velocity control.  It maps
    newline-delimited JSON commands to ROS 2 services and sends a reply only
    after the target ROS service returns.
    """

    def __init__(self) -> None:
        super().__init__("tcp_reliable_command_bridge")
        self.declare_parameter("bind_ip", "0.0.0.0")
        self.declare_parameter("port", 5006)
        self.declare_parameter("command_timeout_s", 3.0)
        self.bind_ip = str(self.get_parameter("bind_ip").value)
        self.port = int(self.get_parameter("port").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)

        self.mode_client = self.create_client(SetMode, "/robot/set_mode")
        self.speed_client = self.create_client(SetFloat, "/safety/set_max_speed")
        self.estop_client = self.create_client(SetBool, "/safety/set_estop")

        self.incoming: queue.Queue[PendingCommand] = queue.Queue(maxsize=100)
        self.stop_event = threading.Event()
        self.server_sock: Optional[socket.socket] = None
        self.worker = threading.Thread(target=self._server, daemon=True)
        self.worker.start()
        self.create_timer(0.01, self._drain)
        self.get_logger().info(
            f"TCP reliable-command server listening on {self.bind_ip}:{self.port}"
        )

    def _server(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock = sock
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.bind_ip, self.port))
        sock.listen(8)
        sock.settimeout(0.5)
        while not self.stop_event.is_set():
            try:
                conn, addr = sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(
                target=self._client_session, args=(conn, addr), daemon=True
            ).start()

    def _client_session(self, conn: socket.socket, addr) -> None:
        with conn:
            conn.settimeout(0.5)
            buffer = b""
            while not self.stop_event.is_set():
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not data:
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    if not line.strip():
                        continue
                    request_id = None
                    try:
                        cmd = parse_reliable_command(line)
                        request_id = cmd.get("request_id")
                        pending = PendingCommand(cmd)
                        self.incoming.put(pending, timeout=0.2)
                        if not pending.event.wait(self.command_timeout_s):
                            reply = make_reply(
                                ok=False,
                                command=cmd["command"],
                                message="ROS command timed out",
                                request_id=request_id,
                            )
                        else:
                            reply = pending.response or make_reply(
                                ok=False,
                                command=cmd["command"],
                                message="empty ROS response",
                                request_id=request_id,
                            )
                    except Exception as exc:
                        reply = make_reply(
                            ok=False,
                            command="invalid",
                            message=str(exc),
                            request_id=request_id,
                        )
                    try:
                        conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))
                    except OSError:
                        return
        self.get_logger().debug(f"TCP client disconnected: {addr}")

    @staticmethod
    def _complete(pending: PendingCommand, reply: Dict[str, Any]) -> None:
        pending.response = reply
        pending.event.set()

    def _service_reply(self, pending: PendingCommand, command: str, future) -> None:
        request_id = pending.command.get("request_id")
        try:
            result = future.result()
            ok = bool(result.success)
            message = str(result.message)
            reply = make_reply(
                ok=ok,
                command=command,
                message=message,
                request_id=request_id,
            )
        except Exception as exc:
            reply = make_reply(
                ok=False,
                command=command,
                message=f"ROS service failed: {exc}",
                request_id=request_id,
            )
        self._complete(pending, reply)

    def _dispatch(self, pending: PendingCommand) -> None:
        cmd = pending.command
        name = cmd["command"]
        request_id = cmd.get("request_id")
        if name == "ping":
            self._complete(
                pending,
                make_reply(
                    ok=True,
                    command="ping",
                    message="pong",
                    request_id=request_id,
                ),
            )
            return

        if name == "set_mode":
            client = self.mode_client
            if not client.service_is_ready():
                return self._complete(
                    pending,
                    make_reply(
                        ok=False,
                        command=name,
                        message="/robot/set_mode service unavailable",
                        request_id=request_id,
                    ),
                )
            req = SetMode.Request()
            req.mode = cmd["mode"]
            future = client.call_async(req)
        elif name == "set_max_speed":
            client = self.speed_client
            if not client.service_is_ready():
                return self._complete(
                    pending,
                    make_reply(
                        ok=False,
                        command=name,
                        message="/safety/set_max_speed service unavailable",
                        request_id=request_id,
                    ),
                )
            req = SetFloat.Request()
            req.value = float(cmd["value"])
            future = client.call_async(req)
        elif name == "set_estop":
            client = self.estop_client
            if not client.service_is_ready():
                return self._complete(
                    pending,
                    make_reply(
                        ok=False,
                        command=name,
                        message="/safety/set_estop service unavailable",
                        request_id=request_id,
                    ),
                )
            req = SetBool.Request()
            req.data = bool(cmd["engaged"])
            future = client.call_async(req)
        else:
            return self._complete(
                pending,
                make_reply(
                    ok=False,
                    command=name,
                    message="unsupported command",
                    request_id=request_id,
                ),
            )

        future.add_done_callback(
            lambda f, p=pending, n=name: self._service_reply(p, n, f)
        )

    def _drain(self) -> None:
        for _ in range(32):
            try:
                pending = self.incoming.get_nowait()
            except queue.Empty:
                break
            self._dispatch(pending)

    def shutdown(self) -> None:
        self.stop_event.set()
        if self.server_sock is not None:
            try:
                self.server_sock.close()
            except OSError:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpReliableCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
