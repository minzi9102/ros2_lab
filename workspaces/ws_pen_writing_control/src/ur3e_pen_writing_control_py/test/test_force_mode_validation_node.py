import math

import pytest

from ur3e_pen_writing_control_py.force_mode_validation_node import (
    force_delta_norm,
    position_distance,
    PROFILES,
    projected_displacement,
    retracted_pose,
)
from ur3e_pen_writing_control_py.pose_math import Point3, PoseTarget, Quaternion


def test_force_profiles_keep_hard_safety_limits():
    assert PROFILES["zero"].target_force_n == 0.0
    assert PROFILES["direction"].target_force_n == 0.5
    assert PROFILES["contact"].target_force_n == 2.0
    assert max(profile.duration_sec for profile in PROFILES.values()) <= 5.0
    assert (
        max(profile.max_displacement_m for profile in PROFILES.values()) <= 0.005
    )


def test_projected_displacement_uses_captured_tool_axis():
    start = Point3(0.1, 0.2, 0.3)
    current = Point3(0.102, 0.203, 0.304)

    assert projected_displacement(
        start, current, (1.0, 0.0, 0.0)
    ) == pytest.approx(0.002)
    assert projected_displacement(
        start, current, (0.0, 0.0, 1.0)
    ) == pytest.approx(0.004)


def test_force_limit_uses_full_force_delta():
    assert force_delta_norm((1.0, 2.0, 3.0), (4.0, 6.0, 3.0)) == pytest.approx(5.0)


def test_retraction_moves_opposite_captured_tool_axis():
    current = PoseTarget(
        position=Point3(0.4, 0.2, 0.1),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
    axis = (math.sqrt(0.5), 0.0, math.sqrt(0.5))
    target = retracted_pose(current, axis, 0.003)

    assert projected_displacement(
        current.position, target.position, axis
    ) == pytest.approx(-0.003)
    assert position_distance(current.position, target.position) == pytest.approx(0.003)
    assert target.orientation == current.orientation
