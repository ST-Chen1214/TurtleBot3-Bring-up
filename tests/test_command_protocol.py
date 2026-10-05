import importlib.util
import sys
from pathlib import Path

MODULE_PATH = (
    Path(__file__).parents[1]
    / "src"
    / "tb3_distributed_core"
    / "tb3_distributed_core"
    / "command_protocol.py"
)
spec = importlib.util.spec_from_file_location("command_protocol", MODULE_PATH)
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
spec.loader.exec_module(mod)


def test_udp_velocity_packet_is_clamped_and_keeps_sequence():
    linear, angular, seq = mod.parse_velocity_packet(
        b'{"linear_x":9,"angular_z":-9,"seq":17}', 0.22, 2.84
    )
    assert linear == 0.22
    assert angular == -2.84
    assert seq == 17


def test_tcp_reliable_command_validation():
    cmd = mod.parse_reliable_command(
        '{"command":"set_mode","mode":"teleop","request_id":4}'
    )
    assert cmd["command"] == "set_mode"
    assert cmd["mode"] == "TELEOP"
    assert cmd["request_id"] == 4


def test_tcp_rejects_velocity_as_reliable_command():
    try:
        mod.parse_reliable_command('{"command":"forward"}')
    except ValueError as exc:
        assert "unsupported" in str(exc)
    else:
        raise AssertionError("forward should not be a TCP reliable command")
