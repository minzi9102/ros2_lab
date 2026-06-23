from ur3e_keyboard_servo_py.evdev_key_reader import EvdevKeyEvent, KeyEventValue
from ur3e_keyboard_servo_py.pressed_key_state import PressedKeyState


def event(key_name, value):
    return EvdevKeyEvent(key_name, value)


def test_key_down_holds_direction_until_key_up():
    state = PressedKeyState()

    state.apply(event('KEY_W', KeyEventValue.DOWN))
    assert state.target_axes() == (1.0, 0.0)
    assert state.target_axes() == (1.0, 0.0)

    state.apply(event('KEY_W', KeyEventValue.UP))
    assert state.target_axes() == (0.0, 0.0)


def test_repeat_event_does_not_change_pressed_state():
    state = PressedKeyState()
    state.apply(event('KEY_A', KeyEventValue.DOWN))

    state.apply(event('KEY_A', KeyEventValue.REPEAT))

    assert state.target_axes() == (0.0, 1.0)


def test_duplicate_keys_for_same_direction_do_not_conflict():
    state = PressedKeyState()
    state.apply(event('KEY_W', KeyEventValue.DOWN))
    state.apply(event('KEY_UP', KeyEventValue.DOWN))

    assert state.target_axes() == (1.0, 0.0)

    state.apply(event('KEY_W', KeyEventValue.UP))
    assert state.target_axes() == (1.0, 0.0)


def test_multiple_logical_directions_stop_motion():
    state = PressedKeyState()
    state.apply(event('KEY_W', KeyEventValue.DOWN))
    state.apply(event('KEY_A', KeyEventValue.DOWN))
    assert state.target_axes() == (0.0, 0.0)

    state.clear()
    state.apply(event('KEY_W', KeyEventValue.DOWN))
    state.apply(event('KEY_S', KeyEventValue.DOWN))
    assert state.target_axes() == (0.0, 0.0)


def test_space_and_q_clear_motion_and_request_immediate_stop():
    state = PressedKeyState()
    state.apply(event('KEY_D', KeyEventValue.DOWN))

    stop = state.apply(event('KEY_SPACE', KeyEventValue.DOWN))
    assert stop.emergency_stop
    assert not stop.quit_requested
    assert state.target_axes() == (0.0, 0.0)

    state.apply(event('KEY_W', KeyEventValue.DOWN))
    quit_decision = state.apply(event('KEY_Q', KeyEventValue.DOWN))
    assert quit_decision.emergency_stop
    assert quit_decision.quit_requested
    assert state.target_axes() == (0.0, 0.0)
