from __future__ import annotations

import math


def finite_or_none(value: float):
    value = float(value)
    return value if math.isfinite(value) else None


def status_to_dict(msg):
    return {
        "timestamp": {"sec": int(msg.stamp.sec), "nanosec": int(msg.stamp.nanosec)},
        "mode": msg.mode,
        "safety": {
            "blocked": bool(msg.safety_blocked),
            "reason": msg.safety_reason,
            "estop": bool(msg.estop_engaged),
        },
        "distance": {
            "front_m": finite_or_none(msg.front_distance_m),
            "min_m": finite_or_none(msg.min_distance_m),
        },
        "battery_percent": finite_or_none(msg.battery_percent),
        "velocity": {
            "linear_mps": float(msg.linear_velocity_mps),
            "angular_rps": float(msg.angular_velocity_rps),
        },
        "pose": {
            "x_m": float(msg.x_m),
            "y_m": float(msg.y_m),
            "yaw_rad": float(msg.yaw_rad),
        },
        "max_linear_speed_mps": finite_or_none(msg.max_linear_speed_mps),
        "health": {
            "lidar": bool(msg.lidar_alive),
            "camera": bool(msg.camera_alive),
            "motor": bool(msg.motor_alive),
        },
    }
