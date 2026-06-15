from ur3e_keyboard_servo_py.key_mapping import (
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_UP,
    KeyAction,
    map_key,
)


def test_arrow_keys_map_to_expected_xy_directions():
    assert map_key(KEY_UP).x > 0.0
    assert map_key(KEY_DOWN).x < 0.0
    assert map_key(KEY_LEFT).y > 0.0
    assert map_key(KEY_RIGHT).y < 0.0


def test_wasd_keys_match_arrow_directions():
    assert map_key('w') == map_key(KEY_UP)
    assert map_key('s') == map_key(KEY_DOWN)
    assert map_key('a') == map_key(KEY_LEFT)
    assert map_key('d') == map_key(KEY_RIGHT)


def test_space_stops_and_q_quits():
    assert map_key(' ').action == KeyAction.STOP
    assert map_key('q').action == KeyAction.QUIT


def test_unknown_or_empty_key_is_ignored():
    assert map_key(None).action == KeyAction.IGNORE
    assert map_key('').action == KeyAction.IGNORE
    assert map_key('x').action == KeyAction.IGNORE
