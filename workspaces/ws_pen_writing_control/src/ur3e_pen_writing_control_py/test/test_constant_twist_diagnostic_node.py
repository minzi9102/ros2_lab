import math

from ur3e_pen_writing_control_py.constant_twist_diagnostic_node import (
    PROFILES,
    distance,
    rotation_angle,
)


def test_constant_twist_profiles_match_diagnostic_plan():
    assert PROFILES["pure_x"] == ((0.03, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert PROFILES["pure_y"] == ((0.0, 0.03, 0.0), (0.0, 0.0, 0.0))
    assert PROFILES["pure_yaw"] == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.3))


def test_distance_and_rotation_angle_helpers():
    assert distance((0.0, 0.0, 0.0), (0.03, 0.04, 0.0)) == 0.05

    identity = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    yaw_90 = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    assert math.isclose(rotation_angle(identity, yaw_90), math.pi / 2.0)
