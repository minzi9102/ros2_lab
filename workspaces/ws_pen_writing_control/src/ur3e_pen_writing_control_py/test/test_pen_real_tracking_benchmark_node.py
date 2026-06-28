from ur3e_pen_writing_control_py.controller_switch_once_node import (
    controller_states_match,
)
from ur3e_pen_writing_control_py.pen_real_tracking_benchmark_node import (
    should_return_home,
)
from ur3e_pen_writing_control_py.pen_tracking_benchmark_node import (
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


def test_real_benchmark_return_home_policy():
    assert should_return_home("completed")
    assert should_return_home("performance_fail")
    assert should_return_home("operator_b")
    assert not should_return_home("safety_abort")
    assert not should_return_home("infrastructure_error")
    assert not should_return_home("keyboard_interrupt")


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
