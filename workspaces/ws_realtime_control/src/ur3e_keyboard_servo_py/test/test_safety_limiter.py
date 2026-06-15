import pytest

from ur3e_keyboard_servo_py.key_mapping import KeyAction, KeyCommand
from ur3e_keyboard_servo_py.safety_limiter import SafetyLimiter


def make_limiter():
    return SafetyLimiter(linear_speed_mps=0.02, key_timeout_sec=0.20)


def test_motion_is_limited_to_configured_xy_speed():
    limiter = make_limiter()

    twist = limiter.apply_key_command(KeyCommand(KeyAction.MOVE, x=10.0, y=-10.0), 1.0)

    assert twist.linear_x == pytest.approx(0.02)
    assert twist.linear_y == pytest.approx(-0.02)
    assert twist.linear_z == 0.0


def test_z_and_rotation_remain_zero_for_motion_command():
    limiter = make_limiter()

    twist = limiter.apply_key_command(KeyCommand(KeyAction.MOVE, x=1.0), 1.0)

    assert twist.linear_z == 0.0
    assert twist.angular_x == 0.0
    assert twist.angular_y == 0.0
    assert twist.angular_z == 0.0


def test_stop_and_quit_return_zero_command():
    limiter = make_limiter()
    limiter.apply_key_command(KeyCommand(KeyAction.MOVE, x=1.0), 1.0)

    assert limiter.apply_key_command(KeyCommand(KeyAction.STOP), 1.1).is_zero

    limiter.apply_key_command(KeyCommand(KeyAction.MOVE, x=1.0), 2.0)
    assert limiter.apply_key_command(KeyCommand(KeyAction.QUIT), 2.1).is_zero


def test_timeout_returns_zero_command():
    limiter = make_limiter()
    limiter.apply_key_command(KeyCommand(KeyAction.MOVE, x=1.0), 1.0)

    assert not limiter.current_command(1.10).is_zero
    assert limiter.current_command(1.21).is_zero


def test_invalid_limits_are_rejected():
    with pytest.raises(ValueError):
        SafetyLimiter(linear_speed_mps=-0.01, key_timeout_sec=0.20)

    with pytest.raises(ValueError):
        SafetyLimiter(linear_speed_mps=0.02, key_timeout_sec=-0.01)
