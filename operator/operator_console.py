#!/usr/bin/env python3
"""Full operator-side console for the distributed TurtleBot3 project.

No ROS 2 installation is required on the operator computer.

Channels used here are intentionally separated by semantics:
  UDP      -> real-time velocity + heartbeat
  TCP      -> reliable discrete commands + application ACK
  ZeroMQ   -> asynchronous telemetry subscription
  REST     -> on-demand status resources and camera snapshots
Bluetooth is a separate local-maintenance path; see bluetooth_maintenance_client.py.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path


def read_key_windows():
    import msvcrt
    while True:
        if msvcrt.kbhit():
            return msvcrt.getwch().lower()
        time.sleep(0.01)


def read_key_posix():
    import select
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.05)
            if ready:
                return sys.stdin.read(1).lower()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def read_key():
    return read_key_windows() if sys.platform.startswith("win") else read_key_posix()


class UdpVelocitySender:
    def __init__(self, host: str, port: int, rate_hz: float):
        self.host = host
        self.port = port
        self.period = 1.0 / max(1.0, rate_hz)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.lock = threading.Lock()
        self.linear = 0.0
        self.angular = 0.0
        self.seq = 0
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def set(self, linear: float, angular: float) -> None:
        with self.lock:
            self.linear = float(linear)
            self.angular = float(angular)

    def _packet(self) -> bytes:
        with self.lock:
            linear, angular = self.linear, self.angular
        payload = {
            "linear_x": linear,
            "angular_z": angular,
            "seq": self.seq,
            "wall_time": time.time(),
        }
        self.seq += 1
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    def send_once(self) -> None:
        self.sock.sendto(self._packet(), (self.host, self.port))

    def _loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.send_once()
            except OSError as exc:
                print(f"\n[UDP] network error: {exc}")
            self.stop_event.wait(self.period)

    def close(self) -> None:
        self.set(0.0, 0.0)
        try:
            for _ in range(3):
                self.send_once()
                time.sleep(0.03)
        except OSError:
            pass
        self.stop_event.set()
        self.sock.close()


class TcpReliableClient:
    def __init__(self, host: str, port: int, timeout: float = 4.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.file = None
        self.lock = threading.Lock()
        self.request_id = 0

    def _connect(self) -> None:
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        sock.settimeout(self.timeout)
        self.sock = sock
        self.file = sock.makefile("rwb", buffering=0)

    def command(self, command: str, **kwargs):
        with self.lock:
            if self.sock is None:
                self._connect()
            self.request_id += 1
            payload = {"command": command, "request_id": self.request_id, **kwargs}
            line = (json.dumps(payload, separators=(",", ":")) + "\n").encode()
            try:
                self.file.write(line)
                raw = self.file.readline()
                if not raw:
                    raise OSError("TCP server closed connection")
                return json.loads(raw.decode("utf-8"))
            except Exception:
                self.close()
                raise

    def close(self) -> None:
        if self.file is not None:
            try:
                self.file.close()
            except OSError:
                pass
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.file = None
        self.sock = None


class ZmqTelemetryClient:
    def __init__(self, host: str, port: int):
        try:
            import zmq
        except ImportError as exc:
            raise RuntimeError("operator telemetry needs pyzmq: pip install pyzmq") from exc
        self.zmq = zmq
        self.context = zmq.Context.instance()
        self.sock = self.context.socket(zmq.SUB)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "robot.status")
        self.sock.setsockopt(zmq.RCVHWM, 4)
        self.sock.connect(f"tcp://{host}:{port}")
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.last_rx_wall = None
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        poller = self.zmq.Poller()
        poller.register(self.sock, self.zmq.POLLIN)
        while not self.stop_event.is_set():
            events = dict(poller.poll(200))
            if self.sock not in events:
                continue
            try:
                topic, payload = self.sock.recv_multipart()
                data = json.loads(payload.decode("utf-8"))
                with self.lock:
                    self.latest = data
                    self.last_rx_wall = time.time()
            except Exception:
                continue

    def get(self):
        with self.lock:
            return self.latest, self.last_rx_wall

    def close(self):
        self.stop_event.set()
        self.sock.close(linger=0)


class RestResourceClient:
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.base = f"http://{host}:{port}"
        self.timeout = timeout

    def get_json(self, path: str):
        with urllib.request.urlopen(self.base + path, timeout=self.timeout) as response:
            return json.loads(response.read().decode("utf-8"))

    def snapshot(self, output_dir: Path, quality: int = 85) -> Path:
        url = f"{self.base}/api/snapshot?quality={int(quality)}"
        with urllib.request.urlopen(url, timeout=self.timeout) as response:
            data = response.read()
        output_dir.mkdir(parents=True, exist_ok=True)
        filename = time.strftime("snapshot_%Y%m%d_%H%M%S.jpg")
        path = output_dir / filename
        path.write_bytes(data)
        return path


def compact_status(data) -> str:
    if not data:
        return "telemetry unavailable"
    d = data.get("distance", {})
    s = data.get("safety", {})
    h = data.get("health", {})
    p = data.get("pose", {})
    return (
        f"mode={data.get('mode')} safety={s.get('reason')} "
        f"front={d.get('front_m')}m battery={data.get('battery_percent')}% "
        f"pose=({p.get('x_m'):.2f},{p.get('y_m'):.2f},{p.get('yaw_rad'):.2f}) "
        f"health[L/C/M]={int(bool(h.get('lidar')))}/{int(bool(h.get('camera')))}/{int(bool(h.get('motor')))}"
    )


def pretty(obj):
    print(json.dumps(obj, indent=2, sort_keys=True))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True, help="Communication Pi IP address")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--tcp-port", type=int, default=5006)
    parser.add_argument("--zmq-port", type=int, default=5555)
    parser.add_argument("--rest-port", type=int, default=8000)
    parser.add_argument("--linear", type=float, default=0.12)
    parser.add_argument("--angular", type=float, default=1.0)
    parser.add_argument("--udp-rate", type=float, default=20.0)
    parser.add_argument("--snapshot-dir", default=str(Path(__file__).with_name("snapshots")))
    args = parser.parse_args()

    udp = UdpVelocitySender(args.host, args.udp_port, args.udp_rate)
    tcp = TcpReliableClient(args.host, args.tcp_port)
    rest = RestResourceClient(args.host, args.rest_port)
    telemetry = None
    try:
        telemetry = ZmqTelemetryClient(args.host, args.zmq_port)
    except Exception as exc:
        print(f"[ZeroMQ disabled] {exc}")

    mode = "TELEOP"
    speeds = {"1": 0.08, "2": 0.14, "3": 0.22}
    print(
        "\n=== TurtleBot3 Operator Console ===\n"
        "UDP motion: W/S/A/D, SPACE=stop\n"
        "TCP reliable: E=E-stop, C=clear E-stop, M=toggle TELEOP/IDLE, 1/2/3=max speed\n"
        "ZeroMQ: T=show latest continuous telemetry\n"
        "REST: P=take photo, I=get full status\n"
        "Q=quit\n"
    )

    try:
        while True:
            key = read_key()
            if key == "w":
                udp.set(args.linear, 0.0)
            elif key == "s":
                udp.set(-args.linear, 0.0)
            elif key == "a":
                udp.set(0.0, args.angular)
            elif key == "d":
                udp.set(0.0, -args.angular)
            elif key in {" ", "x"}:
                udp.set(0.0, 0.0)
            elif key == "e":
                udp.set(0.0, 0.0)
                pretty(tcp.command("set_estop", engaged=True))
            elif key == "c":
                pretty(tcp.command("set_estop", engaged=False))
            elif key == "m":
                mode = "IDLE" if mode == "TELEOP" else "TELEOP"
                pretty(tcp.command("set_mode", mode=mode))
            elif key in speeds:
                pretty(tcp.command("set_max_speed", value=speeds[key]))
            elif key == "t":
                if telemetry is None:
                    print("ZeroMQ telemetry disabled")
                else:
                    data, rx = telemetry.get()
                    age = None if rx is None else time.time() - rx
                    print(f"[ZMQ age={age}] {compact_status(data)}")
            elif key == "i":
                try:
                    pretty(rest.get_json("/api/status"))
                except urllib.error.URLError as exc:
                    print(f"[REST] status failed: {exc}")
            elif key == "p":
                try:
                    path = rest.snapshot(Path(args.snapshot_dir))
                    print(f"[REST] snapshot saved: {path}")
                except urllib.error.URLError as exc:
                    print(f"[REST] snapshot failed: {exc}")
            elif key == "q":
                break
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\noperator error: {exc}")
    finally:
        udp.close()
        tcp.close()
        if telemetry is not None:
            telemetry.close()
        print("Stopped; final zero-velocity packets sent.")


if __name__ == "__main__":
    main()
