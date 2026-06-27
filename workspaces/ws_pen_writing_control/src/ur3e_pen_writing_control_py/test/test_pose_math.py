import math

import pytest

from ur3e_pen_writing_control_py.pen_math import PenPose2D, pen_axis_vector
from ur3e_pen_writing_control_py.pose_math import (
    ContinuousPenOrientation,
    Point3,
    pose_target_from_pen_pose,
    quaternion_dot,
    rotate_vector,
    tool_pose_from_pen_tip_pose,
    transform_point,
    vector_angle,
)


def test_pose_target_position_adds_paper_origin_and_tip_xy():
    target = pose_target_from_pen_pose(
        pen_pose=PenPose2D(
            tip_x=0.03,
            tip_y=-0.02,
            yaw=0.0,
            tilt_rad=0.0,
        ),
        paper_origin=Point3(x=0.4, y=0.1, z=0.2),
        pen_length=0.14,
    )

    assert target.position.x == pytest.approx(0.43)
    assert target.position.y == pytest.approx(0.08)
    assert target.position.z == pytest.approx(0.2)


def test_pose_target_keeps_tool_z_vertical_when_pen_is_upright():
    target = pose_target_from_pen_pose(
        pen_pose=PenPose2D(
            tip_x=0.0,
            tip_y=0.0,
            yaw=math.pi / 2.0,
            tilt_rad=0.0,
        ),
        paper_origin=Point3(x=0.0, y=0.0, z=0.0),
        pen_length=0.14,
    )

    rotated_z = rotate_vector(target.orientation, (0.0, 0.0, 1.0))

    assert rotated_z[0] == pytest.approx(0.0, abs=1e-12)
    assert rotated_z[1] == pytest.approx(0.0, abs=1e-12)
    assert rotated_z[2] == pytest.approx(-1.0)


def test_pose_target_aligns_tool_z_with_virtual_pen_tail_to_tip_axis():
    pen_pose = PenPose2D(
        tip_x=0.0,
        tip_y=0.0,
        yaw=math.pi / 2.0,
        tilt_rad=math.radians(20.0),
    )

    target = pose_target_from_pen_pose(
        pen_pose=pen_pose,
        paper_origin=Point3(x=0.0, y=0.0, z=0.0),
        pen_length=0.14,
    )

    expected_axis = pen_axis_vector(
        tail_yaw=pen_pose.yaw,
        tilt_rad=pen_pose.tilt_rad,
        pen_length=0.14,
    )
    rotated_z = rotate_vector(target.orientation, (0.0, 0.0, 1.0))

    assert rotated_z[0] == pytest.approx(-expected_axis[0] / 0.14)
    assert rotated_z[1] == pytest.approx(-expected_axis[1] / 0.14)
    assert rotated_z[2] == pytest.approx(-expected_axis[2] / 0.14)


def test_pose_target_quaternion_is_normalized():
    target = pose_target_from_pen_pose(
        pen_pose=PenPose2D(
            tip_x=0.0,
            tip_y=0.0,
            yaw=-math.pi / 4.0,
            tilt_rad=math.radians(20.0),
        ),
        paper_origin=Point3(x=0.0, y=0.0, z=0.0),
        pen_length=0.14,
    )

    q = target.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)

    assert norm == pytest.approx(1.0)


def test_tool_pose_places_tool_origin_above_upright_pen_tip_with_positive_z_tip_offset():
    target = tool_pose_from_pen_tip_pose(
        pen_pose=PenPose2D(
            tip_x=0.03,
            tip_y=-0.02,
            yaw=0.0,
            tilt_rad=0.0,
        ),
        paper_origin=Point3(x=0.4, y=0.1, z=0.2),
        pen_length=0.14,
        tool0_to_pen_tip_xyz=Point3(x=0.0, y=0.0, z=0.14),
    )

    assert target.position.x == pytest.approx(0.43)
    assert target.position.y == pytest.approx(0.08)
    assert target.position.z == pytest.approx(0.34)


def test_tool_pose_z_axis_aligns_with_tilted_pen_tail_to_tip_axis():
    pen_pose = PenPose2D(
        tip_x=0.0,
        tip_y=0.0,
        yaw=math.pi / 2.0,
        tilt_rad=math.radians(20.0),
    )

    target = tool_pose_from_pen_tip_pose(
        pen_pose=pen_pose,
        paper_origin=Point3(x=0.0, y=0.0, z=0.0),
        pen_length=0.14,
        tool0_to_pen_tip_xyz=Point3(x=0.0, y=0.0, z=0.14),
    )

    expected_axis = pen_axis_vector(
        tail_yaw=pen_pose.yaw,
        tilt_rad=pen_pose.tilt_rad,
        pen_length=0.14,
    )
    rotated_z = rotate_vector(target.orientation, (0.0, 0.0, 1.0))

    assert rotated_z[0] == pytest.approx(-expected_axis[0] / 0.14)
    assert rotated_z[1] == pytest.approx(-expected_axis[1] / 0.14)
    assert rotated_z[2] == pytest.approx(-expected_axis[2] / 0.14)


def test_tool_pose_offset_reconstructs_pen_tip_world_position():
    pen_pose = PenPose2D(
        tip_x=-0.04,
        tip_y=0.05,
        yaw=-math.pi / 4.0,
        tilt_rad=math.radians(20.0),
    )
    paper_origin = Point3(x=0.35, y=0.12, z=0.21)
    tool0_to_pen_tip = Point3(x=0.0, y=0.0, z=0.14)

    target = tool_pose_from_pen_tip_pose(
        pen_pose=pen_pose,
        paper_origin=paper_origin,
        pen_length=0.14,
        tool0_to_pen_tip_xyz=tool0_to_pen_tip,
    )
    reconstructed_tip = transform_point(target, tool0_to_pen_tip)

    assert reconstructed_tip.x == pytest.approx(0.31)
    assert reconstructed_tip.y == pytest.approx(0.17)
    assert reconstructed_tip.z == pytest.approx(0.21)


def test_continuous_orientation_does_not_spin_xy_when_upright_yaw_reverses():
    orientation = ContinuousPenOrientation(
        initial_pen_pose=PenPose2D(
            tip_x=0.0,
            tip_y=0.0,
            yaw=0.0,
            tilt_rad=0.0,
        ),
        pen_length=0.14,
        max_axis_angular_speed_radps=math.radians(55.0),
    )
    initial_frame = orientation.frame

    orientation.update(
        PenPose2D(
            tip_x=0.0,
            tip_y=0.0,
            yaw=math.pi,
            tilt_rad=0.0,
        ),
        dt_sec=0.1,
    )

    assert orientation.frame.x_axis == pytest.approx(initial_frame.x_axis)
    assert orientation.frame.y_axis == pytest.approx(initial_frame.y_axis)
    assert orientation.frame.z_axis == pytest.approx(initial_frame.z_axis)


def test_continuous_orientation_limits_pen_axis_angular_speed():
    initial_pose = PenPose2D(
        tip_x=0.0,
        tip_y=0.0,
        yaw=0.0,
        tilt_rad=math.radians(20.0),
    )
    orientation = ContinuousPenOrientation(
        initial_pen_pose=initial_pose,
        pen_length=0.14,
        max_axis_angular_speed_radps=math.radians(10.0),
    )
    initial_z = orientation.frame.z_axis

    orientation.update(
        PenPose2D(
            tip_x=0.0,
            tip_y=0.0,
            yaw=math.pi,
            tilt_rad=math.radians(20.0),
        ),
        dt_sec=1.0,
    )

    assert vector_angle(initial_z, orientation.frame.z_axis) == pytest.approx(
        math.radians(10.0)
    )
    assert orientation.axis_error_rad == pytest.approx(math.radians(30.0))


def test_continuous_orientation_keeps_orthonormal_frame_and_quaternion_sign():
    orientation = ContinuousPenOrientation(
        initial_pen_pose=PenPose2D(
            tip_x=0.0,
            tip_y=0.0,
            yaw=0.0,
            tilt_rad=0.0,
        ),
        pen_length=0.14,
        max_axis_angular_speed_radps=math.radians(55.0),
    )
    previous_quaternion = orientation.orientation

    for yaw in (math.pi / 2.0, math.pi, -math.pi / 2.0, 0.0):
        quaternion = orientation.update(
            PenPose2D(
                tip_x=0.0,
                tip_y=0.0,
                yaw=yaw,
                tilt_rad=math.radians(20.0),
            ),
            dt_sec=0.1,
        )
        frame = orientation.frame

        x_norm = sum(value * value for value in frame.x_axis)
        y_norm = sum(value * value for value in frame.y_axis)
        z_norm = sum(value * value for value in frame.z_axis)
        xy_dot = sum(a * b for a, b in zip(frame.x_axis, frame.y_axis))
        xz_dot = sum(a * b for a, b in zip(frame.x_axis, frame.z_axis))
        yz_dot = sum(a * b for a, b in zip(frame.y_axis, frame.z_axis))

        assert x_norm == pytest.approx(1.0)
        assert y_norm == pytest.approx(1.0)
        assert z_norm == pytest.approx(1.0)
        assert xy_dot == pytest.approx(0.0, abs=1e-12)
        assert xz_dot == pytest.approx(0.0, abs=1e-12)
        assert yz_dot == pytest.approx(0.0, abs=1e-12)
        assert quaternion_dot(previous_quaternion, quaternion) >= 0.0
        previous_quaternion = quaternion


def test_pose_target_accepts_continuous_orientation_override():
    orientation = ContinuousPenOrientation(
        initial_pen_pose=PenPose2D(
            tip_x=0.0,
            tip_y=0.0,
            yaw=0.0,
            tilt_rad=0.0,
        ),
        pen_length=0.14,
        max_axis_angular_speed_radps=math.radians(55.0),
    )

    target = pose_target_from_pen_pose(
        pen_pose=PenPose2D(
            tip_x=0.03,
            tip_y=-0.02,
            yaw=math.pi,
            tilt_rad=math.radians(20.0),
        ),
        paper_origin=Point3(x=0.4, y=0.1, z=0.2),
        pen_length=0.14,
        orientation_override=orientation.orientation,
    )

    assert target.orientation == orientation.orientation
    assert target.position.x == pytest.approx(0.43)
    assert target.position.y == pytest.approx(0.08)
