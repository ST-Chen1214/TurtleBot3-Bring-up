"""Wire-protocol helpers shared by the external communication bridges.

The operator computer does not need ROS 2.  It talks to the Communication Pi
using small JSON messages.  Once a message reaches the Communication Pi it is
translated into ROS 2 topics/services.
"""
from __future__ import annotations

import json
import math
from typing import Any, Dict, Optional, Tuple


def _load_json(payload: bytes | str) -> Dict[str, Any]:
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    obj = json.loads(payload)
    if not isinstance(obj, dict):
        raise ValueError("message must be a JSON object")
    return obj


def parse_velocity_packet(
    payload: bytes | str,
    max_linear: float,
    max_angular: float,
) -> Tuple[float, float, Optional[int]]:
    """Parse the UDP latest-value velocity packet.

    Expected format::

        {"linear_x": 0.12, "angular_z": 0.0, "seq": 42}

    ``seq`` is optional for compatibility but strongly recommended.  The UDP
    bridge uses it to reject stale/out-of-order datagrams.
    """
    obj = _load_json(payload)
    linear = float(obj.get("linear_x", 0.0))
    angular = float(obj.get("angular_z", 0.0))
    if not math.isfinite(linear) or not math.isfinite(angular):
        raise ValueError("velocity values must be finite")
    linear = max(-float(max_linear), min(float(max_linear), linear))
    angular = max(-float(max_angular), min(float(max_angular), angular))
    seq_obj = obj.get("seq")
    seq = int(seq_obj) if seq_obj is not None else None
    return linear, angular, seq


def parse_reliable_command(payload: bytes | str) -> Dict[str, Any]:
    """Validate one newline-delimited JSON command used by the TCP channel."""
    obj = _load_json(payload)
    command = str(obj.get("command", "")).strip().lower()
    if command not in {"ping", "set_mode", "set_max_speed", "set_estop"}:
        raise ValueError(f"unsupported TCP command: {command!r}")
    obj["command"] = command

    if command == "set_mode":
        mode = str(obj.get("mode", "")).strip().upper()
        if not mode:
            raise ValueError("set_mode requires 'mode'")
        obj["mode"] = mode
    elif command == "set_max_speed":
        value = float(obj.get("value"))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("set_max_speed requires a positive finite 'value'")
        obj["value"] = value
    elif command == "set_estop":
        if "engaged" not in obj:
            raise ValueError("set_estop requires 'engaged'")
        obj["engaged"] = bool(obj["engaged"])

    return obj


def make_reply(*, ok: bool, command: str, message: str, request_id: Any = None, **extra: Any) -> Dict[str, Any]:
    reply: Dict[str, Any] = {
        "ok": bool(ok),
        "command": str(command),
        "message": str(message),
    }
    if request_id is not None:
        reply["request_id"] = request_id
    reply.update(extra)
    return reply


# Backward-compatible name used by older code/tests.
def parse_command(payload: bytes, max_linear: float, max_angular: float) -> Tuple[float, float]:
    linear, angular, _ = parse_velocity_packet(payload, max_linear, max_angular)
    return linear, angular
