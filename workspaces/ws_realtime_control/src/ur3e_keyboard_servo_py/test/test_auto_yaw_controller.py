import math

import pytest

from ur3e_keyboard_servo_py.auto_yaw_controller import (
    AutoYawController,
    Quaternion,
    desired_reverse_motion_yaw,
    normalize_angle,
    yaw_from_quaternion,
)
from ur3e_keyboard_servo_py.safety_limiter import TwistCommand


def quaternion_from_yaw(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


@pytest.mark.parametrize('yaw', [0.0, 0.5, -0.75, math.pi])
def test_yaw_from_quaternion(yaw):
    assert yaw_from_quaternion(quaternion_from_yaw(yaw)) == pytest.approx(yaw)


@pytest.mark.parametrize(
    ('linear_x', 'linear_y', 'expected_yaw'),
    [
        (1.0, 0.0, math.pi),
        (0.0, 1.0, -math.pi / 2.0),
        (-1.0, 0.0, 0.0),
        (0.0, -1.0, math.pi / 2.0),
    ],
)
def test_desired_yaw_faces_reverse_motion_direction(
    linear_x,
    linear_y,
    expected_yaw,
):
    assert desired_reverse_motion_yaw(linear_x, linear_y) == pytest.approx(
        expected_yaw
    )


def test_normalize_angle_wraps_to_pi_range():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-3.0 * math.pi) == pytest.approx(-math.pi)
    assert normalize_angle(2.5 * math.pi) == pytest.approx(math.pi / 2.0)


def test_auto_yaw_applies_gain_and_limit():
    controller = AutoYawController(
        yaw_gain=1.5,
        max_angular_speed_radps=0.60,
        min_linear_speed_mps=0.02,
    )

    command = controller.apply(
        TwistCommand(linear_x=0.20),
        current_tool_yaw=0.0,
    )

    assert command.angular_z == pytest.approx(0.60)


def test_auto_yaw_keeps_existing_linear_velocity():
    controller = AutoYawController(
        yaw_gain=1.5,
        max_angular_speed_radps=0.60,
        min_linear_speed_mps=0.02,
    )

    command = controller.apply(
        TwistCommand(linear_x=0.10, linear_y=0.10),
        current_tool_yaw=math.pi,
    )

    assert command.linear_x == pytest.approx(0.10)
    assert command.linear_y == pytest.approx(0.10)
    assert command.linear_z == 0.0
    assert command.angular_x == 0.0
    assert command.angular_y == 0.0


def test_auto_yaw_returns_zero_angular_when_speed_is_too_low():
    controller = AutoYawController(
        yaw_gain=1.5,
        max_angular_speed_radps=0.60,
        min_linear_speed_mps=0.02,
    )

    command = controller.apply(
        TwistCommand(linear_x=0.01),
        current_tool_yaw=0.0,
    )

    assert command.angular_z == 0.0


def test_auto_yaw_leaves_command_unchanged_without_tf():
    controller = AutoYawController(
        yaw_gain=1.5,
        max_angular_speed_radps=0.60,
        min_linear_speed_mps=0.02,
    )
    original = TwistCommand(linear_x=0.20)

    command = controller.apply(original, current_tool_yaw=None)

    assert command == original


def test_invalid_auto_yaw_parameters_are_rejected():
    with pytest.raises(ValueError):
        AutoYawController(
            yaw_gain=0.0,
            max_angular_speed_radps=0.60,
            min_linear_speed_mps=0.02,
        )
