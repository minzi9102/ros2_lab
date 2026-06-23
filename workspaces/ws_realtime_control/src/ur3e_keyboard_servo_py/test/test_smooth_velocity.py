import pytest

from ur3e_keyboard_servo_py.smooth_velocity import SmoothVelocityController


def make_controller():
    return SmoothVelocityController(
        linear_speed_mps=0.20,
        acceleration_mps2=0.50,
        deceleration_mps2=0.80,
    )


def test_accelerates_to_target_in_about_point_four_seconds():
    controller = make_controller()
    samples = [controller.update(1.0, 0.0, 0.01).linear_x for _ in range(40)]

    assert samples == sorted(samples)
    assert samples[-1] == pytest.approx(0.20)
    assert max(samples) <= 0.20


def test_release_decelerates_to_zero_in_about_point_two_five_seconds():
    controller = make_controller()
    for _ in range(40):
        controller.update(1.0, 0.0, 0.01)

    samples = [controller.update(0.0, 0.0, 0.01).linear_x for _ in range(25)]

    assert samples == sorted(samples, reverse=True)
    assert samples[-1] == pytest.approx(0.0)


def test_direction_reversal_passes_through_zero_before_accelerating():
    controller = make_controller()
    for _ in range(20):
        controller.update(1.0, 0.0, 0.01)

    samples = []
    for _ in range(20):
        samples.append(controller.update(-1.0, 0.0, 0.01).linear_x)

    zero_index = next(index for index, value in enumerate(samples) if value == 0.0)
    assert all(value >= 0.0 for value in samples[:zero_index])
    assert any(value < 0.0 for value in samples[zero_index + 1:])


def test_axis_change_stops_old_axis_before_accelerating_new_axis():
    controller = make_controller()
    for _ in range(20):
        controller.update(1.0, 0.0, 0.01)

    while controller.current_command().linear_x > 0.0:
        command = controller.update(0.0, 1.0, 0.01)
        assert command.linear_y == 0.0

    command = controller.update(0.0, 1.0, 0.01)
    assert command.linear_x == 0.0
    assert command.linear_y > 0.0


def test_immediate_stop_bypasses_deceleration():
    controller = make_controller()
    controller.update(1.0, 0.0, 0.10)

    assert controller.stop_immediately().is_zero


def test_invalid_motion_profile_is_rejected():
    with pytest.raises(ValueError):
        SmoothVelocityController(
            linear_speed_mps=0.0,
            acceleration_mps2=0.5,
            deceleration_mps2=0.8,
        )
