"""Pure safety-decision logic, intentionally independent of ROS 2."""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi
from typing import Iterable, Optional, Sequence


@dataclass(frozen=True)
class SafetyConfig:
    stop_distance_m: float = 0.35
    command_timeout_s: float = 0.50
    scan_timeout_s: float = 0.50
    fail_closed_on_scan_loss: bool = True
    max_linear_mps: float = 0.22
    max_angular_rps: float = 2.84


@dataclass(frozen=True)
class SafetyDecision:
    linear_x: float
    angular_z: float
    blocked: bool
    reason: str


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def min_valid_range(
    ranges: Iterable[float],
    range_min: float = 0.0,
    range_max: float = float("inf"),
) -> Optional[float]:
    valid = [
        float(r)
        for r in ranges
        if isfinite(float(r)) and float(r) >= range_min and float(r) <= range_max
    ]
    return min(valid) if valid else None


def sector_min_range(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    center_angle: float = 0.0,
    half_width: float = pi / 12.0,
    range_min: float = 0.0,
    range_max: float = float("inf"),
) -> Optional[float]:
    """Return the minimum valid range in an angular sector.

    Angles are normalized to [-pi, pi].  This helper is used to report a
    human-friendly "front distance" without changing the global safety rule.
    """
    if not ranges or angle_increment == 0.0:
        return None

    def wrap(a: float) -> float:
        while a > pi:
            a -= 2.0 * pi
        while a < -pi:
            a += 2.0 * pi
        return a

    vals = []
    for i, raw in enumerate(ranges):
        r = float(raw)
        if not isfinite(r) or r < range_min or r > range_max:
            continue
        angle = angle_min + i * angle_increment
        if abs(wrap(angle - center_angle)) <= half_width:
            vals.append(r)
    return min(vals) if vals else None


def evaluate_safety(
    *,
    linear_x: float,
    angular_z: float,
    command_age_s: Optional[float],
    scan_age_s: Optional[float],
    min_distance_m: Optional[float],
    estop_latched: bool,
    config: SafetyConfig,
    mode: str = "TELEOP",
) -> SafetyDecision:
    """Return the velocity command allowed to reach the base controller."""
    if estop_latched:
        return SafetyDecision(0.0, 0.0, True, "ESTOP")

    if str(mode).upper() != "TELEOP":
        return SafetyDecision(0.0, 0.0, True, f"MODE_{str(mode).upper()}")

    if command_age_s is None or command_age_s > config.command_timeout_s:
        return SafetyDecision(0.0, 0.0, True, "COMMAND_TIMEOUT")

    if scan_age_s is None or scan_age_s > config.scan_timeout_s:
        if config.fail_closed_on_scan_loss:
            return SafetyDecision(0.0, 0.0, True, "SCAN_TIMEOUT")

    if min_distance_m is None:
        if config.fail_closed_on_scan_loss:
            return SafetyDecision(0.0, 0.0, True, "NO_VALID_SCAN")
    elif min_distance_m < config.stop_distance_m:
        # Intentionally conservative: *any* obstacle inside the protected
        # radius blocks operator motion, matching the project requirement.
        return SafetyDecision(0.0, 0.0, True, "OBSTACLE_TOO_CLOSE")

    return SafetyDecision(
        clamp(float(linear_x), -config.max_linear_mps, config.max_linear_mps),
        clamp(float(angular_z), -config.max_angular_rps, config.max_angular_rps),
        False,
        "CLEAR",
    )
