import csv
import math

import pytest

from ur3e_pen_writing_control_py.joy_mapping import JoyControl
from ur3e_pen_writing_control_py.pen_math import PenPose2D, PlanarVelocity
from ur3e_pen_writing_control_py.pen_fakehardware_servo_node import (
    AlignmentErrorCsvLogger,
    ToolAlignmentError,
    fixed_vertical_pen_orientation,
    has_planar_motion_intent,
    initial_active_servo_command_mode,
    is_session_timed_out,
    is_servo_status_fresh,
    is_tool_pose_aligned,
    is_virtual_pen_settling,
    paper_origin_from_current_tool0,
    pose_mode_became_ready,
    pose_axis_points,
    should_publish_pose_command,
    should_switch_linear_only_to_twist,
    target_orientation_for_command,
    tool_alignment_error,
    tool_tail_to_tip_points,
    tool_tip_point_from_tool_pose,
    twist_feedforward_command,
)
from ur3e_pen_writing_control_py.pose_math import (
    Point3,
    PoseTarget,
    Quaternion,
    tool_pose_from_pen_tip_pose,
)


def test_neutral_joy_control_has_no_planar_motion_intent():
    assert not has_planar_motion_intent(JoyControl())


def test_nonzero_joy_control_has_planar_motion_intent():
    assert has_planar_motion_intent(JoyControl(target_x=1.0))
    assert has_planar_motion_intent(JoyControl(target_y=-1.0))


def test_twist_feedforward_combines_target_derivative_and_pose_correction():
    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    quarter_turn = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(math.pi / 4.0),
        w=math.cos(math.pi / 4.0),
    )
    previous = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=identity,
    )
    target = PoseTarget(
        position=Point3(x=0.01, y=0.0, z=0.0),
        orientation=quarter_turn,
    )
    current = PoseTarget(
        position=Point3(x=0.0, y=-0.01, z=0.0),
        orientation=identity,
    )

    command = twist_feedforward_command(
        previous_target=previous,
        target=target,
        current=current,
        dt_sec=0.1,
        position_gain=2.0,
        orientation_gain=2.0,
        linear_correction_limit_mps=0.01,
        angular_correction_limit_radps=0.2,
    )

    correction = math.sqrt(0.5) * 0.01
    assert command.linear == pytest.approx((0.1 + correction, correction, 0.0))
    assert command.angular == pytest.approx((0.0, 0.0, math.pi / 0.2 + 0.2))


def test_twist_feedforward_treats_opposite_quaternion_sign_as_same_orientation():
    target = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=-1.0),
    )
    current = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )

    command = twist_feedforward_command(
        previous_target=None,
        target=target,
        current=current,
        dt_sec=0.0,
        position_gain=2.0,
        orientation_gain=2.0,
        linear_correction_limit_mps=0.03,
        angular_correction_limit_radps=0.3,
    )

    assert command.linear == (0.0, 0.0, 0.0)
    assert command.angular == (0.0, 0.0, 0.0)


def test_linear_only_twist_keeps_linear_command_and_zeros_angular_command():
    identity = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    target = PoseTarget(
        position=Point3(x=0.01, y=0.02, z=0.0),
        orientation=Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(math.pi / 8.0),
            w=math.cos(math.pi / 8.0),
        ),
    )
    current = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=identity,
    )
    kwargs = {
        "previous_target": current,
        "target": target,
        "current": current,
        "dt_sec": 0.1,
        "position_gain": 2.0,
        "orientation_gain": 2.0,
        "linear_correction_limit_mps": 0.03,
        "angular_correction_limit_radps": 0.3,
    }

    full_command = twist_feedforward_command(**kwargs)
    linear_only_command = twist_feedforward_command(
        **kwargs,
        angular_enabled=False,
    )

    assert linear_only_command.linear == pytest.approx(full_command.linear)
    assert linear_only_command.angular == (0.0, 0.0, 0.0)


def test_linear_only_starts_in_pose_and_switches_only_after_alignment():
    assert initial_active_servo_command_mode("twist_linear_only") == "pose"
    assert initial_active_servo_command_mode("twist_feedforward") == (
        "twist_feedforward"
    )
    ready = {
        "configured_mode": "twist_linear_only",
        "active_mode": "pose",
        "command_armed": True,
        "has_motion_intent": False,
        "virtual_pen_settling": False,
        "tool_pose_aligned": True,
    }

    assert should_switch_linear_only_to_twist(**ready)
    assert not should_switch_linear_only_to_twist(
        **{**ready, "has_motion_intent": True}
    )
    assert not should_switch_linear_only_to_twist(
        **{**ready, "virtual_pen_settling": True}
    )


def test_linear_only_uses_frozen_target_orientation():
    dynamic = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    frozen = Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)

    assert target_orientation_for_command(
        configured_mode="twist_linear_only",
        diagnostic_orientation_mode="dynamic",
        fixed_vertical_orientation=dynamic,
        frozen_orientation=frozen,
        dynamic_orientation=dynamic,
    ) == frozen
    assert target_orientation_for_command(
        configured_mode="twist_feedforward",
        diagnostic_orientation_mode="dynamic",
        fixed_vertical_orientation=dynamic,
        frozen_orientation=frozen,
        dynamic_orientation=dynamic,
    ) == dynamic


def test_fixed_vertical_orientation_mode_overrides_dynamic_orientation():
    dynamic = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    fixed = fixed_vertical_pen_orientation(pen_length=0.14)

    assert target_orientation_for_command(
        configured_mode="pose",
        diagnostic_orientation_mode="fixed_vertical",
        fixed_vertical_orientation=fixed,
        frozen_orientation=None,
        dynamic_orientation=dynamic,
    ) == fixed


def test_fixed_vertical_orientation_is_constant_and_upright():
    orientation_a = fixed_vertical_pen_orientation(pen_length=0.14)
    orientation_b = fixed_vertical_pen_orientation(pen_length=0.20)

    assert orientation_a == orientation_b
    assert orientation_a.x == pytest.approx(1.0)
    assert orientation_a.y == pytest.approx(0.0)
    assert orientation_a.z == pytest.approx(0.0)
    assert orientation_a.w == pytest.approx(0.0)


def test_fixed_vertical_orientation_still_allows_tip_xy_motion():
    orientation = fixed_vertical_pen_orientation(pen_length=0.14)
    paper_origin = Point3(x=0.4, y=0.1, z=0.2)
    tool0_to_pen_tip = Point3(x=0.0, y=0.0, z=0.14)
    target_a = tool_pose_from_pen_tip_pose(
        pen_pose=PenPose2D(
            tip_x=0.00,
            tip_y=0.00,
            yaw=0.0,
            tilt_rad=math.radians(20.0),
        ),
        paper_origin=paper_origin,
        pen_length=0.14,
        tool0_to_pen_tip_xyz=tool0_to_pen_tip,
        orientation_override=orientation,
    )
    target_b = tool_pose_from_pen_tip_pose(
        pen_pose=PenPose2D(
            tip_x=0.03,
            tip_y=-0.02,
            yaw=math.pi / 3.0,
            tilt_rad=math.radians(35.0),
        ),
        paper_origin=paper_origin,
        pen_length=0.14,
        tool0_to_pen_tip_xyz=tool0_to_pen_tip,
        orientation_override=orientation,
    )

    assert target_a.orientation == orientation
    assert target_b.orientation == orientation
    assert target_a.position != target_b.position


def test_paper_origin_uses_current_tool_xy_but_fixed_configured_z():
    paper_origin, estimated_tip_z = paper_origin_from_current_tool0(
        current_tool_position=Point3(x=0.4, y=0.2, z=0.5),
        current_tool_orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        tool0_to_pen_tip=Point3(x=0.0, y=0.0, z=0.14),
        initial_tip_x=0.03,
        initial_tip_y=-0.02,
        fixed_paper_z=0.12,
    )

    assert paper_origin.x == pytest.approx(0.37)
    assert paper_origin.y == pytest.approx(0.22)
    assert paper_origin.z == pytest.approx(0.12)
    assert estimated_tip_z == pytest.approx(0.64)


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


def test_session_timeout_is_disabled_when_duration_is_zero():
    assert not is_session_timed_out(
        max_session_duration_sec=0.0,
        session_started_at_sec=10.0,
        now_sec=1000.0,
    )


def test_session_timeout_triggers_after_configured_duration():
    assert not is_session_timed_out(
        max_session_duration_sec=30.0,
        session_started_at_sec=10.0,
        now_sec=39.99,
    )
    assert is_session_timed_out(
        max_session_duration_sec=30.0,
        session_started_at_sec=10.0,
        now_sec=40.0,
    )


def test_pose_command_publishes_only_after_arm():
    assert not should_publish_pose_command(
        pose_command_armed=False,
        has_motion_intent=True,
        virtual_pen_settling=True,
        tool_pose_aligned=False,
        servo_health_fault=False,
    )


def test_pose_mode_became_ready_only_on_first_successful_transition():
    assert pose_mode_became_ready(was_ready=False, is_ready=True)
    assert not pose_mode_became_ready(was_ready=False, is_ready=False)
    assert not pose_mode_became_ready(was_ready=True, is_ready=True)


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
    assert is_virtual_pen_settling(
        velocity=PlanarVelocity(),
        tilt_rad=0.0,
        speed_tolerance_mps=0.002,
        tilt_tolerance_rad=math.radians(0.5),
        orientation_error_rad=math.radians(1.0),
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


def test_tool_alignment_error_separates_position_z_axis_and_full_orientation():
    current = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    translated = PoseTarget(
        position=Point3(x=0.003, y=0.004, z=0.0),
        orientation=current.orientation,
    )
    z_twisted = PoseTarget(
        position=current.position,
        orientation=Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(math.pi / 4.0),
            w=math.cos(math.pi / 4.0),
        ),
    )

    translated_error = tool_alignment_error(
        current_tool_pose=current,
        target_tool_pose=translated,
    )
    assert translated_error.position_m == pytest.approx(0.005)
    assert translated_error.z_axis_rad == pytest.approx(0.0)
    assert translated_error.full_quaternion_rad == pytest.approx(0.0)

    twisted_error = tool_alignment_error(
        current_tool_pose=current,
        target_tool_pose=z_twisted,
    )
    assert twisted_error.position_m == pytest.approx(0.0)
    assert twisted_error.z_axis_rad == pytest.approx(0.0)
    assert twisted_error.full_quaternion_rad == pytest.approx(math.pi / 2.0)


def test_tool_alignment_error_treats_opposite_quaternion_sign_as_same_pose():
    current = PoseTarget(
        position=Point3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9),
    )
    target = PoseTarget(
        position=current.position,
        orientation=Quaternion(x=-0.1, y=-0.2, z=-0.3, w=-0.9),
    )

    assert tool_alignment_error(
        current_tool_pose=current,
        target_tool_pose=target,
    ).full_quaternion_rad == pytest.approx(0.0)


def test_alignment_error_csv_starts_on_first_pose_and_samples_at_20_hz(tmp_path):
    log_path = tmp_path / "tool_alignment_error.csv"
    logger = AlignmentErrorCsvLogger(path=str(log_path), sample_rate_hz=20.0)
    error = ToolAlignmentError(
        position_m=0.005,
        z_axis_rad=math.radians(2.0),
        full_quaternion_rad=math.radians(15.0),
    )

    assert not logger.record(
        now_sec=10.0,
        start_requested=False,
        error=error,
        pose_command_armed=False,
        pose_command_published=False,
        has_motion_intent=False,
        virtual_pen_settling=False,
    )
    assert not log_path.exists()

    assert logger.record(
        now_sec=11.0,
        start_requested=True,
        error=error,
        pose_command_armed=True,
        pose_command_published=True,
        has_motion_intent=True,
        virtual_pen_settling=False,
    )
    assert not logger.record(
        now_sec=11.02,
        start_requested=False,
        error=error,
        pose_command_armed=False,
        pose_command_published=False,
        has_motion_intent=False,
        virtual_pen_settling=False,
    )
    assert logger.record(
        now_sec=11.05,
        start_requested=False,
        error=None,
        pose_command_armed=False,
        pose_command_published=False,
        has_motion_intent=False,
        virtual_pen_settling=False,
    )
    logger.close()

    with log_path.open(newline="", encoding="utf-8") as log_file:
        rows = list(csv.DictReader(log_file))
    assert len(rows) == 2
    assert rows[0]["elapsed_sec"] == "0.000000000"
    assert rows[0]["position_m"] == "0.005000000"
    assert rows[0]["z_axis_deg"] == "2.000000"
    assert rows[0]["full_quaternion_deg"] == "15.000000"
    assert rows[0]["pose_command_published"] == "1"
    assert rows[1]["elapsed_sec"] == "0.050000000"
    assert rows[1]["position_m"] == "nan"
    assert rows[1]["pose_command_armed"] == "0"
    assert rows[1]["pose_command_published"] == "0"


def test_tool_tip_point_uses_tool0_to_pen_tip_positive_z_direction():
    tool_pose = PoseTarget(
        position=Point3(x=0.1, y=0.2, z=0.3),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )

    tip = tool_tip_point_from_tool_pose(
        tool_pose=tool_pose,
        tool0_to_pen_tip=Point3(x=0.0, y=0.0, z=0.14),
    )

    assert tip.x == pytest.approx(0.1)
    assert tip.y == pytest.approx(0.2)
    assert tip.z == pytest.approx(0.44)


def test_tool_tail_to_tip_points_make_arrow_match_virtual_pen_axis():
    tool_pose = PoseTarget(
        position=Point3(x=0.1, y=0.2, z=0.3),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )

    start, end = tool_tail_to_tip_points(
        tool_pose=tool_pose,
        tool0_to_pen_tip=Point3(x=0.0, y=0.0, z=0.14),
    )

    assert start.z == pytest.approx(0.3)
    assert end.z == pytest.approx(0.44)


def test_pose_axis_points_rotate_and_enlarge_display_axes():
    pose = PoseTarget(
        position=Point3(x=0.1, y=0.2, z=0.3),
        orientation=Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(math.pi / 4.0),
            w=math.cos(math.pi / 4.0),
        ),
    )

    start, end = pose_axis_points(
        pose=pose,
        local_axis=Point3(x=1.0, y=0.0, z=0.0),
        axis_length_m=0.08,
    )

    assert start == pose.position
    assert end.x == pytest.approx(0.1)
    assert end.y == pytest.approx(0.28)
    assert end.z == pytest.approx(0.3)
