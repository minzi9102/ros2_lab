import math

import pytest

from ur3e_pen2_control_py.joy_mapping import JoyMapper


def test_joy_mapping_preserves_existing_axis_convention_and_normalizes():
    intent = JoyMapper(0.08).map([1.0, -1.0], [])

    assert intent.x == pytest.approx(1.0 / math.sqrt(2.0))
    assert intent.y == pytest.approx(-1.0 / math.sqrt(2.0))


def test_joy_mapping_deadzone_and_short_messages_are_idle():
    mapper = JoyMapper(0.08)

    assert mapper.map([0.01, 0.01], []).x == 0.0
    assert mapper.map([], []).y == 0.0


def test_a_is_emergency_stop_and_b_also_requests_quit():
    mapper = JoyMapper(0.08)

    assert mapper.map([], [1, 0]).emergency_stop
    quit_intent = mapper.map([], [0, 1])
    assert quit_intent.emergency_stop
    assert quit_intent.quit_requested
