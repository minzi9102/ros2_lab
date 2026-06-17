from ur3e_keyboard_servo_py.keyboard_servo_node import (
    REQUIRED_REAL_CONFIRMATION,
    is_motion_confirmation_valid,
)


def test_confirmation_is_not_required_for_sim_defaults():
    assert is_motion_confirmation_valid(
        require_confirmation=False,
        human_confirmation='',
    )


def test_real_motion_confirmation_requires_exact_phrase():
    assert is_motion_confirmation_valid(
        require_confirmation=True,
        human_confirmation=REQUIRED_REAL_CONFIRMATION,
    )

    assert not is_motion_confirmation_valid(
        require_confirmation=True,
        human_confirmation='',
    )

    assert not is_motion_confirmation_valid(
        require_confirmation=True,
        human_confirmation='I_CONFIRM_REAL_ROBOT_MOTION ',
    )


def test_custom_confirmation_phrase_can_be_checked():
    assert is_motion_confirmation_valid(
        require_confirmation=True,
        human_confirmation='CONFIRM',
        required_confirmation_text='CONFIRM',
    )

    assert not is_motion_confirmation_valid(
        require_confirmation=True,
        human_confirmation='CONFIRM',
        required_confirmation_text='DIFFERENT',
    )
