import math

import pytest

from ur3e_pen_writing_control_py.pen_math import PenPose2D, pen_axis_vector
from ur3e_pen_writing_control_py.pose_math import (
    Point3,
    pose_target_from_pen_pose,
    rotate_vector,
    tool_pose_from_pen_tip_pose,
    transform_point,
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
    assert rotated_z[2] == pytest.approx(1.0)


def test_pose_target_aligns_tool_z_with_virtual_pen_axis():
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

    assert rotated_z[0] == pytest.approx(expected_axis[0] / 0.14)
    assert rotated_z[1] == pytest.approx(expected_axis[1] / 0.14)
    assert rotated_z[2] == pytest.approx(expected_axis[2] / 0.14)


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


def test_tool_pose_places_tool_origin_above_upright_pen_tip():
    target = tool_pose_from_pen_tip_pose(
        pen_pose=PenPose2D(
            tip_x=0.03,
            tip_y=-0.02,
            yaw=0.0,
            tilt_rad=0.0,
        ),
        paper_origin=Point3(x=0.4, y=0.1, z=0.2),
        pen_length=0.14,
        tool0_to_pen_tip_xyz=Point3(x=0.0, y=0.0, z=-0.14),
    )

    assert target.position.x == pytest.approx(0.43)
    assert target.position.y == pytest.approx(0.08)
    assert target.position.z == pytest.approx(0.34)


def test_tool_pose_z_axis_aligns_with_tilted_pen_axis():
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
        tool0_to_pen_tip_xyz=Point3(x=0.0, y=0.0, z=-0.14),
    )

    expected_axis = pen_axis_vector(
        tail_yaw=pen_pose.yaw,
        tilt_rad=pen_pose.tilt_rad,
        pen_length=0.14,
    )
    rotated_z = rotate_vector(target.orientation, (0.0, 0.0, 1.0))

    assert rotated_z[0] == pytest.approx(expected_axis[0] / 0.14)
    assert rotated_z[1] == pytest.approx(expected_axis[1] / 0.14)
    assert rotated_z[2] == pytest.approx(expected_axis[2] / 0.14)


def test_tool_pose_offset_reconstructs_pen_tip_world_position():
    pen_pose = PenPose2D(
        tip_x=-0.04,
        tip_y=0.05,
        yaw=-math.pi / 4.0,
        tilt_rad=math.radians(20.0),
    )
    paper_origin = Point3(x=0.35, y=0.12, z=0.21)
    tool0_to_pen_tip = Point3(x=0.0, y=0.0, z=-0.14)

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
