from ur3e_pen_writing_control_py.controller_switch_once_node import (
    controller_states_match,
    non_empty_strings,
)
from ur3e_pen_writing_control_py.pen_real_tracking_benchmark_node import (
    alignment_row_ready,
    should_return_home,
)
from ur3e_pen_writing_control_py.pen_tracking_benchmark_node import (
    LONG_MINUS_Y_PLUS_XY_PROFILE,
    benchmark_phases,
    joy_message_for_target,
)


def test_real_benchmark_reuses_complete_eight_direction_sequence():
    scored = [phase.label for phase in benchmark_phases() if phase.scored]

    assert scored == [
        "plus_x",
        "minus_x",
        "plus_y",
        "minus_y",
        "plus_xy",
        "minus_xy",
        "plus_x_minus_y",
        "minus_x_plus_y",
    ]


def test_real_benchmark_can_select_long_direction_profile():
    scored = [
        phase.label
        for phase in benchmark_phases(LONG_MINUS_Y_PLUS_XY_PROFILE)
        if phase.scored
    ]

    assert scored == ["minus_y", "plus_xy"]


def test_real_benchmark_return_home_policy():
    assert should_return_home("completed")
    assert should_return_home("performance_fail")
    assert should_return_home("operator_b")
    assert not should_return_home("safety_abort")
    assert not should_return_home("infrastructure_error")
    assert not should_return_home("keyboard_interrupt")


def test_real_alignment_ready_requires_low_error_and_not_settling():
    ready_row = {
        "position_m": 0.004,
        "z_axis_deg": 2.0,
        "virtual_pen_settling": 0.0,
    }
    settling_row = {
        "position_m": 0.004,
        "z_axis_deg": 2.0,
        "virtual_pen_settling": 1.0,
    }
    high_error_row = {
        "position_m": 0.020,
        "z_axis_deg": 2.0,
        "virtual_pen_settling": 0.0,
    }

    assert alignment_row_ready(ready_row)
    assert not alignment_row_ready(settling_row)
    assert not alignment_row_ready(high_error_row)


def test_freeze_joy_uses_a_button_without_b():
    message = joy_message_for_target(0.0, 0.0, emergency_stop=True)

    assert list(message.axes) == [-0.0, -0.0]
    assert list(message.buttons) == [1, 0]


def test_controller_switch_requires_expected_active_and_inactive_states():
    assert controller_states_match(
        {
            "forward_position_controller": "active",
            "scaled_joint_trajectory_controller": "inactive",
        },
        activate=["forward_position_controller"],
        deactivate=["scaled_joint_trajectory_controller"],
    )
    assert not controller_states_match(
        {
            "forward_position_controller": "active",
            "scaled_joint_trajectory_controller": "active",
        },
        activate=["forward_position_controller"],
        deactivate=["scaled_joint_trajectory_controller"],
    )


def test_controller_switch_ignores_empty_string_array_default():
    assert non_empty_strings(["", "forward_position_controller"]) == [
        "forward_position_controller"
    ]
