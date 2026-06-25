import math

import pytest

from ur3e_pen_writing_control_py.pen_math import (
    PaperBounds,
    PlanarVelocity,
    SmoothPlanarVelocity,
    VirtualPenState,
    desired_reverse_motion_yaw,
    pen_axis_vector,
)


@pytest.mark.parametrize(
    ("linear_x", "linear_y", "expected_yaw"),
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


def test_pen_axis_tilts_toward_reverse_motion_yaw():
    axis = pen_axis_vector(
        reverse_yaw=math.pi,
        tilt_rad=math.radians(20.0),
        pen_length=0.14,
    )

    assert axis[0] < 0.0
    assert axis[1] == pytest.approx(0.0, abs=1e-12)
    assert axis[2] > 0.0
    assert math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) == pytest.approx(0.14)


def test_virtual_pen_holds_yaw_when_speed_is_too_low():
    state = VirtualPenState(
        initial_tip_x=0.0,
        initial_tip_y=0.0,
        initial_yaw=1.25,
        paper_bounds=PaperBounds(width=0.2, height=0.2),
        yaw_hold_speed_mps=0.01,
    )

    pose = state.update(PlanarVelocity(x=0.001, y=0.0), 1.0)

    assert pose.yaw == pytest.approx(1.25)


def test_virtual_pen_updates_yaw_when_speed_is_high_enough():
    state = VirtualPenState(
        initial_tip_x=0.0,
        initial_tip_y=0.0,
        initial_yaw=1.25,
        paper_bounds=PaperBounds(width=0.2, height=0.2),
        yaw_hold_speed_mps=0.01,
    )

    pose = state.update(PlanarVelocity(x=0.02, y=0.0), 1.0)

    assert pose.yaw == pytest.approx(math.pi)


def test_virtual_pen_clamps_to_paper_bounds():
    state = VirtualPenState(
        initial_tip_x=0.0,
        initial_tip_y=0.0,
        initial_yaw=0.0,
        paper_bounds=PaperBounds(width=0.2, height=0.1),
        yaw_hold_speed_mps=0.0,
    )

    pose = state.update(PlanarVelocity(x=1.0, y=1.0), 1.0)

    assert pose.tip_x == pytest.approx(0.1)
    assert pose.tip_y == pytest.approx(0.05)


def test_smooth_velocity_caps_diagonal_speed():
    velocity = SmoothPlanarVelocity(
        max_speed_mps=0.08,
        acceleration_mps2=100.0,
        deceleration_mps2=100.0,
    )

    command = velocity.update(1.0, 1.0, 1.0)

    assert math.hypot(command.x, command.y) == pytest.approx(0.08)
    assert command.x == pytest.approx(command.y)
