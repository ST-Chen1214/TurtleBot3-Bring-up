import importlib.util
import sys
from math import pi
from pathlib import Path

MODULE_PATH = (
    Path(__file__).parents[1]
    / "src"
    / "tb3_distributed_core"
    / "tb3_distributed_core"
    / "safety_logic.py"
)
spec = importlib.util.spec_from_file_location("safety_logic", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)

SafetyConfig = mod.SafetyConfig
evaluate_safety = mod.evaluate_safety
min_valid_range = mod.min_valid_range
sector_min_range = mod.sector_min_range


def decision(**overrides):
    args = dict(
        linear_x=0.1,
        angular_z=0.2,
        command_age_s=0.1,
        scan_age_s=0.1,
        min_distance_m=1.0,
        estop_latched=False,
        config=SafetyConfig(),
        mode="TELEOP",
    )
    args.update(overrides)
    return evaluate_safety(**args)


def test_clear_command_passes():
    d = decision()
    assert not d.blocked
    assert d.reason == "CLEAR"


def test_obstacle_overrides_operator():
    d = decision(
        linear_x=0.1,
        angular_z=0.0,
        min_distance_m=0.20,
        config=SafetyConfig(stop_distance_m=0.35),
    )
    assert d.blocked and d.linear_x == 0.0 and d.angular_z == 0.0
    assert d.reason == "OBSTACLE_TOO_CLOSE"


def test_command_timeout_stops():
    d = decision(command_age_s=1.0, config=SafetyConfig(command_timeout_s=0.5))
    assert d.blocked and d.reason == "COMMAND_TIMEOUT"


def test_scan_timeout_fail_closed():
    d = decision(scan_age_s=1.0, config=SafetyConfig(scan_timeout_s=0.5))
    assert d.blocked and d.reason == "SCAN_TIMEOUT"


def test_estop_has_priority():
    d = decision(estop_latched=True, mode="IDLE")
    assert d.blocked and d.reason == "ESTOP"


def test_non_teleop_mode_blocks_motion():
    d = decision(mode="IDLE")
    assert d.blocked and d.reason == "MODE_IDLE"


def test_min_valid_range_filters_nan_inf_and_out_of_range():
    assert min_valid_range([float("nan"), float("inf"), 0.01, 0.8, 1.2], 0.05, 2.0) == 0.8


def test_front_sector_distance():
    # Five rays at -90, -45, 0, +45, +90 degrees; only the middle ray is in +/-20 deg.
    ranges = [2.0, 1.5, 0.42, 1.0, 0.8]
    result = sector_min_range(
        ranges,
        angle_min=-pi / 2,
        angle_increment=pi / 4,
        half_width=pi / 9,
        range_min=0.05,
        range_max=3.5,
    )
    assert result == 0.42
