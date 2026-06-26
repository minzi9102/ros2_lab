import pytest

from ur3e_pen_writing_control_py.joy_mapping import JoyControl
from ur3e_pen_writing_control_py.pen_fakehardware_servo_node import (
    has_planar_motion_intent,
    is_servo_status_fresh,
    paper_origin_from_current_tool0,
    should_publish_pose_command,
)
from ur3e_pen_writing_control_py.pose_math import Point3, Quaternion


def test_neutral_joy_control_has_no_planar_motion_intent():
    assert not has_planar_motion_intent(JoyControl())


def test_nonzero_joy_control_has_planar_motion_intent():
    assert has_planar_motion_intent(JoyControl(target_x=1.0))
    assert has_planar_motion_intent(JoyControl(target_y=-1.0))


def test_paper_origin_uses_current_tool_xy_but_fixed_configured_z():
    paper_origin, estimated_tip_z = paper_origin_from_current_tool0(
        current_tool_position=Point3(x=0.4, y=0.2, z=0.5),
        current_tool_orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        tool0_to_pen_tip=Point3(x=0.0, y=0.0, z=-0.14),
        initial_tip_x=0.03,
        initial_tip_y=-0.02,
        fixed_paper_z=0.12,
    )

    assert paper_origin.x == pytest.approx(0.37)
    assert paper_origin.y == pytest.approx(0.22)
    assert paper_origin.z == pytest.approx(0.12)
    assert estimated_tip_z == pytest.approx(0.36)


def test_servo_status_fresh_requires_seen_recent_status():
    assert is_servo_status_fresh(
        status_seen=True,
        last_status_time=9.5,
        now_sec=10.0,
        timeout_sec=1.0,
    )
    assert not is_servo_status_fresh(
        status_seen=False,
        last_status_time=10.0,
        now_sec=10.0,
        timeout_sec=1.0,
    )
    assert not is_servo_status_fresh(
        status_seen=True,
        last_status_time=8.0,
        now_sec=10.0,
        timeout_sec=1.0,
    )


def test_pose_command_publishes_only_while_armed_and_motion_is_requested():
    assert should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=True,
        servo_health_fault=False,
    )
    assert not should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=False,
        servo_health_fault=False,
    )
    assert not should_publish_pose_command(
        pose_command_armed=False,
        has_motion_intent=True,
        servo_health_fault=False,
    )
    assert not should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=True,
        servo_health_fault=True,
    )
