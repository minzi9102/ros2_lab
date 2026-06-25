from ur3e_pen_writing_control_py.joy_mapping import JoyControl
from ur3e_pen_writing_control_py.pen_fakehardware_servo_node import (
    has_planar_motion_intent,
)


def test_neutral_joy_control_has_no_planar_motion_intent():
    assert not has_planar_motion_intent(JoyControl())


def test_nonzero_joy_control_has_planar_motion_intent():
    assert has_planar_motion_intent(JoyControl(target_x=1.0))
    assert has_planar_motion_intent(JoyControl(target_y=-1.0))
