import math

import pytest

from ur3e_pen_writing_control_py.pen_math import (
    PaperBounds,
    PlanarVelocity,
    planar_turn_speed_scale,
    SmoothPlanarVelocity,
    VirtualPenState,
    desired_pen_tail_yaw,
    move_toward,
    pen_axis_vector,
)


@pytest.mark.parametrize(
    ("linear_x", "linear_y", "expected_yaw"),
    [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, math.pi / 2.0),
        (-1.0, 0.0, math.pi),
        (0.0, -1.0, -math.pi / 2.0),
    ],
)
def test_desired_yaw_puts_pen_tail_ahead_of_motion(
    linear_x,
    linear_y,
    expected_yaw,
):
    assert desired_pen_tail_yaw(linear_x, linear_y) == pytest.approx(
        expected_yaw
    )


def test_pen_axis_tilts_tail_toward_motion_yaw():
    axis = pen_axis_vector(
        tail_yaw=0.0,
        tilt_rad=math.radians(20.0),
        pen_length=0.14,
    )

    assert axis[0] > 0.0
    assert axis[1] == pytest.approx(0.0, abs=1e-12)
    assert axis[2] > 0.0
    assert math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2) == pytest.approx(0.14)


def test_pen_axis_is_vertical_when_tilt_is_zero():
    axis = pen_axis_vector(
        tail_yaw=0.0,
        tilt_rad=0.0,
        pen_length=0.14,
    )

    assert axis[0] == pytest.approx(0.0)
    assert axis[1] == pytest.approx(0.0)
    assert axis[2] == pytest.approx(0.14)


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

    assert pose.yaw == pytest.approx(0.0)


def test_virtual_pen_starts_upright():
    state = VirtualPenState(
        initial_tip_x=0.0,
        initial_tip_y=0.0,
        initial_yaw=0.0,
        paper_bounds=PaperBounds(width=0.2, height=0.2),
        yaw_hold_speed_mps=0.01,
        target_tilt_rad=math.radians(20.0),
        tilt_activate_speed_mps=0.01,
        tilt_rate_radps=math.radians(45.0),
        untilt_rate_radps=math.radians(60.0),
    )

    assert state.pose.tilt_rad == pytest.approx(0.0)


def test_virtual_pen_tilts_gradually_while_moving():
    state = VirtualPenState(
        initial_tip_x=0.0,
        initial_tip_y=0.0,
        initial_yaw=0.0,
        paper_bounds=PaperBounds(width=0.2, height=0.2),
        yaw_hold_speed_mps=0.01,
        target_tilt_rad=math.radians(20.0),
        tilt_activate_speed_mps=0.01,
        tilt_rate_radps=math.radians(45.0),
        untilt_rate_radps=math.radians(60.0),
    )

    pose = state.update(PlanarVelocity(x=0.02, y=0.0), 0.2)

    assert pose.tilt_rad == pytest.approx(math.radians(9.0))

    pose = state.update(PlanarVelocity(x=0.02, y=0.0), 1.0)

    assert pose.tilt_rad == pytest.approx(math.radians(20.0))


def test_virtual_pen_returns_upright_after_stopping():
    state = VirtualPenState(
        initial_tip_x=0.0,
        initial_tip_y=0.0,
        initial_yaw=0.0,
        paper_bounds=PaperBounds(width=0.2, height=0.2),
        yaw_hold_speed_mps=0.01,
        target_tilt_rad=math.radians(20.0),
        tilt_activate_speed_mps=0.01,
        tilt_rate_radps=math.radians(45.0),
        untilt_rate_radps=math.radians(60.0),
    )

    state.update(PlanarVelocity(x=0.02, y=0.0), 1.0)
    pose = state.update(PlanarVelocity(), 0.2)

    assert pose.tilt_rad == pytest.approx(math.radians(8.0))

    pose = state.update(PlanarVelocity(), 1.0)

    assert pose.tilt_rad == pytest.approx(0.0)


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


def test_turn_speed_scale_stays_full_for_small_heading_change():
    scale = planar_turn_speed_scale(
        current_velocity=PlanarVelocity(x=0.03, y=0.0),
        target_x=1.0,
        target_y=0.1,
        active_speed_mps=0.01,
        soft_turn_deg=45.0,
        hard_turn_deg=135.0,
        min_scale=0.35,
    )

    assert scale == pytest.approx(1.0)


def test_turn_speed_scale_reaches_min_for_reverse_command():
    scale = planar_turn_speed_scale(
        current_velocity=PlanarVelocity(x=0.03, y=0.0),
        target_x=-1.0,
        target_y=0.0,
        active_speed_mps=0.01,
        soft_turn_deg=45.0,
        hard_turn_deg=135.0,
        min_scale=0.35,
    )

    assert scale == pytest.approx(0.35)


def test_turn_speed_scale_blends_for_mid_angle_change():
    scale = planar_turn_speed_scale(
        current_velocity=PlanarVelocity(x=0.03, y=0.0),
        target_x=0.0,
        target_y=1.0,
        active_speed_mps=0.01,
        soft_turn_deg=45.0,
        hard_turn_deg=135.0,
        min_scale=0.35,
    )

    assert scale == pytest.approx(0.675)


def test_turn_speed_scale_does_not_slow_from_near_stop():
    scale = planar_turn_speed_scale(
        current_velocity=PlanarVelocity(x=0.001, y=0.0),
        target_x=-1.0,
        target_y=0.0,
        active_speed_mps=0.01,
        soft_turn_deg=45.0,
        hard_turn_deg=135.0,
        min_scale=0.35,
    )

    assert scale == pytest.approx(1.0)


def test_move_toward_limits_step_without_overshoot():
    assert move_toward(0.0, 1.0, 0.25) == pytest.approx(0.25)
    assert move_toward(0.9, 1.0, 0.25) == pytest.approx(1.0)
    assert move_toward(1.0, 0.0, 0.25) == pytest.approx(0.75)
    assert move_toward(0.1, 0.0, 0.25) == pytest.approx(0.0)
