from __future__ import annotations

import queue
import threading
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node

from tb3_distributed_interfaces.msg import RobotStatus
from tb3_distributed_interfaces.srv import GetSnapshot

from .status_codec import status_to_dict


@dataclass
class SnapshotRequest:
    quality: int
    event: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    message: str = ""
    content_type: str = ""
    data: bytes = b""


class RestApiBridge(Node):
    """HTTP resource API for snapshots and on-demand robot state.

    REST is deliberately kept out of the fast control loop.  It exposes
    resources that naturally fit request/response semantics.
    """

    def __init__(self) -> None:
        super().__init__("rest_api_bridge")
        try:
            from fastapi import FastAPI, HTTPException, Response
            import uvicorn
        except ImportError as exc:
            raise RuntimeError("fastapi and uvicorn are required") from exc

        self.HTTPException = HTTPException
        self.Response = Response
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8000)
        self.declare_parameter("snapshot_timeout_s", 4.0)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)

        self.status_lock = threading.Lock()
        self.latest_status: Optional[RobotStatus] = None
        self.create_subscription(RobotStatus, "/robot/status", self._on_status, 10)

        self.snapshot_client = self.create_client(GetSnapshot, "/camera/take_snapshot")
        self.snapshot_queue: queue.Queue[SnapshotRequest] = queue.Queue(maxsize=8)
        self.create_timer(0.02, self._drain_snapshot_requests)

        app = FastAPI(
            title="TurtleBot3 Operator Resource API",
            description="On-demand resources only: status and camera snapshots.",
        )

        @app.get("/api/health")
        def health():
            return {
                "ok": True,
                "status_available": self._status_copy() is not None,
                "snapshot_service_available": self.snapshot_client.service_is_ready(),
            }

        @app.get("/api/status")
        def status():
            msg = self._status_copy()
            if msg is None:
                raise HTTPException(status_code=503, detail="robot status not available yet")
            return status_to_dict(msg)

        @app.get("/api/snapshot")
        def snapshot(quality: int = 85):
            quality = max(20, min(100, int(quality)))
            request = SnapshotRequest(quality=quality)
            try:
                self.snapshot_queue.put(request, timeout=0.2)
            except queue.Full:
                raise HTTPException(status_code=503, detail="snapshot queue is busy")
            timeout = float(self.get_parameter("snapshot_timeout_s").value)
            if not request.event.wait(timeout):
                raise HTTPException(status_code=504, detail="snapshot request timed out")
            if not request.success:
                raise HTTPException(status_code=503, detail=request.message)
            return Response(
                content=request.data,
                media_type=request.content_type or "image/jpeg",
                headers={"X-Robot-Message": request.message[:200]},
            )

        @app.get("/api/config")
        def config():
            return {
                "roles": {
                    "udp": "real-time velocity + heartbeat",
                    "tcp": "reliable discrete commands + ACK",
                    "zeromq": "asynchronous telemetry PUB/SUB",
                    "rest": "on-demand snapshots/status resources",
                    "bluetooth": "local commissioning/diagnostics",
                }
            }

        self.thread = threading.Thread(
            target=lambda: uvicorn.run(app, host=host, port=port, log_level="warning"),
            daemon=True,
        )
        self.thread.start()
        self.get_logger().info(f"REST resource API listening on http://{host}:{port}")

    def _on_status(self, msg: RobotStatus) -> None:
        with self.status_lock:
            self.latest_status = msg

    def _status_copy(self) -> Optional[RobotStatus]:
        with self.status_lock:
            return self.latest_status

    def _snapshot_done(self, request: SnapshotRequest, future) -> None:
        try:
            result = future.result()
            request.success = bool(result.success)
            request.message = str(result.message)
            request.content_type = str(result.content_type)
            request.data = bytes(result.data)
        except Exception as exc:
            request.success = False
            request.message = f"snapshot ROS service failed: {exc}"
        request.event.set()

    def _drain_snapshot_requests(self) -> None:
        for _ in range(4):
            try:
                pending = self.snapshot_queue.get_nowait()
            except queue.Empty:
                break
            if not self.snapshot_client.service_is_ready():
                pending.success = False
                pending.message = "/camera/take_snapshot service unavailable"
                pending.event.set()
                continue
            req = GetSnapshot.Request()
            req.jpeg_quality = int(pending.quality)
            future = self.snapshot_client.call_async(req)
            future.add_done_callback(
                lambda f, p=pending: self._snapshot_done(p, f)
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RestApiBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
