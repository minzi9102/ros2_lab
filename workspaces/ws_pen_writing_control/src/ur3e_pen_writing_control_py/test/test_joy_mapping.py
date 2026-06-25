import math

import pytest

from ur3e_pen_writing_control_py.joy_mapping import JoyMapper


def make_mapper():
    return JoyMapper(deadzone=0.08)


def test_deadzone_returns_zero():
    control = make_mapper().map([0.02, -0.03], [0, 0])

    assert control.target_x == 0.0
    assert control.target_y == 0.0


@pytest.mark.parametrize(
    ("axes", "expected_x", "expected_y"),
    [
        ([0.0, -1.0], 1.0, 0.0),
        ([0.0, 1.0], -1.0, 0.0),
        ([-1.0, 0.0], 0.0, 1.0),
        ([1.0, 0.0], 0.0, -1.0),
    ],
)
def test_left_stick_maps_to_xy_plane(axes, expected_x, expected_y):
    control = make_mapper().map(axes, [0, 0])

    assert control.target_x == pytest.approx(expected_x)
    assert control.target_y == pytest.approx(expected_y)


def test_diagonal_is_normalized_to_unit_vector():
    control = make_mapper().map([-1.0, -1.0], [0, 0])

    assert control.target_x == pytest.approx(1.0 / math.sqrt(2.0))
    assert control.target_y == pytest.approx(1.0 / math.sqrt(2.0))
    assert math.hypot(control.target_x, control.target_y) == pytest.approx(1.0)


def test_a_button_requests_immediate_stop():
    control = make_mapper().map([0.0, -1.0], [1, 0])

    assert control.emergency_stop
    assert not control.quit_requested
    assert control.target_x == 0.0
    assert control.target_y == 0.0


def test_b_button_requests_stop_and_quit():
    control = make_mapper().map([0.0, -1.0], [0, 1])

    assert control.emergency_stop
    assert control.quit_requested
    assert control.target_x == 0.0
    assert control.target_y == 0.0
