import math

import pytest

from ur3e_pen_writing_control_py.joy_mapping import JoyControl
from ur3e_pen_writing_control_py.pen_math import PlanarVelocity
from ur3e_pen_writing_control_py.pen_fakehardware_servo_node import (
    has_planar_motion_intent,
    is_servo_status_fresh,
    is_tool_pose_aligned,
    is_virtual_pen_settling,
    paper_origin_from_current_tool0,
    should_publish_pose_command,
    tool_alignment_error,
)
from ur3e_pen_writing_control_py.pose_math import Point3, PoseTarget, Quaternion


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


def test_pose_command_publishes_only_after_arm():
    assert not should_publish_pose_command(
        pose_command_armed=False,
        has_motion_intent=True,
        virtual_pen_settling=True,
        tool_pose_aligned=False,
        servo_health_fault=False,
    )


def test_pose_command_publishes_while_motion_is_requested():
    assert should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=True,
        virtual_pen_settling=False,
        tool_pose_aligned=True,
        servo_health_fault=False,
    )


def test_pose_command_publishes_while_virtual_pen_is_settling_after_release():
    assert should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=False,
        virtual_pen_settling=True,
        tool_pose_aligned=True,
        servo_health_fault=False,
    )


def test_pose_command_publishes_until_tool_pose_is_aligned():
    assert should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=False,
        virtual_pen_settling=False,
        tool_pose_aligned=False,
        servo_health_fault=False,
    )


def test_pose_command_stops_after_virtual_pen_settles_and_tool_pose_aligns():
    assert not should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=False,
        virtual_pen_settling=False,
        tool_pose_aligned=True,
        servo_health_fault=False,
    )


def test_pose_command_stops_when_servo_health_faults():
    assert not should_publish_pose_command(
        pose_command_armed=True,
        has_motion_intent=True,
        virtual_pen_settling=True,
        tool_pose_aligned=False,
        servo_health_fault=True,
    )


def test_pose_command_stops_after_a_button_freezes_following():
    assert not should_publish_pose_command(
        pose_command_armed=False,
        has_motion_intent=False,
        virtual_pen_settling=True,
        tool_pose_aligned=False,
        servo_health_fault=False,
    )


def test_virtual_pen_settling_detects_speed_or_tilt():
    assert is_virtual_pen_settling(
        velocity=PlanarVelocity(x=0.003, y=0.0),
        tilt_rad=0.0,
        speed_tolerance_mps=0.002,
        tilt_tolerance_rad=math.radians(0.5),
    )
    assert is_virtual_pen_settling(
        velocity=PlanarVelocity(),
        tilt_rad=math.radians(1.0),
        speed_tolerance_mps=0.002,
        tilt_tolerance_rad=math.radians(0.5),
    )
    assert not is_virtual_pen_settling(
        velocity=PlanarVelocity(x=0.001, y=0.0),
        tilt_rad=math.radians(0.25),
        speed_tolerance_mps=0.002,
        tilt_tolerance_rad=math.radians(0.5),
    )


def test_tool_pose_alignment_checks_position_and_tool_z_axis():
    current = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    aligned_target = PoseTarget(
        position=Point3(x=0.003, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    tilted_target = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(
            x=0.0,
            y=math.sin(math.radians(5.0) / 2.0),
            z=0.0,
            w=math.cos(math.radians(5.0) / 2.0),
        ),
    )

    assert is_tool_pose_aligned(
        current_tool_pose=current,
        target_tool_pose=aligned_target,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=math.radians(3.0),
    )
    assert not is_tool_pose_aligned(
        current_tool_pose=current,
        target_tool_pose=tilted_target,
        position_tolerance_m=0.005,
        orientation_tolerance_rad=math.radians(3.0),
    )
    assert tool_alignment_error(
        current_tool_pose=current,
        target_tool_pose=tilted_target,
    ).z_axis_rad == pytest.approx(math.radians(5.0))
