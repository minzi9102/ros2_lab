import pytest

from ur3e_pen_writing_control_py.pen_tip_plane_monitor_node import (
    actual_pen_tip_from_tool_pose,
    normalize_vector,
    signed_distance_to_plane,
)
from ur3e_pen_writing_control_py.pose_math import Point3, PoseTarget, Quaternion


def test_point_on_paper_plane_has_zero_signed_distance():
    distance = signed_distance_to_plane(
        point=Point3(x=0.2, y=-0.1, z=0.12),
        center=Point3(x=0.0, y=0.0, z=0.12),
        normal=Point3(x=0.0, y=0.0, z=1.0),
    )

    assert distance == pytest.approx(0.0)


def test_point_below_paper_plane_has_negative_signed_distance():
    distance = signed_distance_to_plane(
        point=Point3(x=0.2, y=-0.1, z=0.115),
        center=Point3(x=0.0, y=0.0, z=0.12),
        normal=Point3(x=0.0, y=0.0, z=1.0),
    )

    assert distance == pytest.approx(-0.005)


def test_plane_normal_is_normalized_for_signed_distance():
    distance = signed_distance_to_plane(
        point=Point3(x=0.0, y=0.0, z=0.13),
        center=Point3(x=0.0, y=0.0, z=0.12),
        normal=Point3(x=0.0, y=0.0, z=10.0),
    )

    assert distance == pytest.approx(0.01)


def test_zero_plane_normal_is_rejected():
    with pytest.raises(ValueError, match="paper_normal_xyz"):
        normalize_vector(Point3(x=0.0, y=0.0, z=0.0))


def test_actual_pen_tip_uses_tool_pose_and_tcp_offset():
    tool_pose = PoseTarget(
        position=Point3(x=0.1, y=0.2, z=0.3),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    tip = actual_pen_tip_from_tool_pose(
        tool_pose=tool_pose,
        tool0_to_pen_tip=Point3(x=0.001, y=0.031, z=0.174),
    )

    assert tip.x == pytest.approx(0.101)
    assert tip.y == pytest.approx(0.231)
    assert tip.z == pytest.approx(0.474)
