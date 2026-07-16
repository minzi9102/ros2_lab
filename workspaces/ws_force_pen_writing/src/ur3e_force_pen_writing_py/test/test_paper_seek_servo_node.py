import math

from ur3e_force_pen_writing_py.geometry import Point3, Quaternion, transform_point
from ur3e_force_pen_writing_py.paper_seek_servo_node import (
    contact_force_from_baseline,
    lowpass_force_z,
    next_paper_seek_offset,
    paper_seek_baseline_stats,
    paper_seek_controller_error,
    paper_seek_dynamic_threshold,
    paper_seek_tf_progressed,
    paper_seek_tool_pose_target,
)


def test_seek_target_locks_tip_xy_and_orientation():
    orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    offset = Point3(0.00079, -0.00076, 0.15172)
    target = paper_seek_tool_pose_target(
        captured_tip_xy=(0.42, -0.03),
        target_tip_z=0.1,
        captured_tool_orientation=orientation,
        tool0_to_pen_tip=offset,
    )

    tip = transform_point(target, offset)
    assert math.isclose(tip.x, 0.42)
    assert math.isclose(tip.y, -0.03)
    assert math.isclose(tip.z, 0.1)
    assert target.orientation == orientation


def test_seek_force_baseline_filter_and_dynamic_threshold():
    assert lowpass_force_z(
        previous_fz_n=99.0, sample_fz_n=1.0, alpha=0.1, initialized=False
    ) == 1.0
    assert math.isclose(
        contact_force_from_baseline(
            filtered_fz_n=0.8, baseline_fz_n=0.2, force_axis_sign=1.0
        ),
        0.6,
    )
    mean, stddev = paper_seek_baseline_stats([0.8, 1.0, 1.2])
    assert math.isclose(mean, 1.0)
    assert paper_seek_dynamic_threshold(
        minimum_threshold_n=0.5,
        baseline_standard_deviation_n=stddev,
        sigma_multiplier=6.0,
    ) > 0.5


def test_seek_descent_and_controller_guards():
    assert next_paper_seek_offset(
        current_offset_m=-0.001, down_speed_mps=0.0005, dt_sec=1.0
    ) == -0.0015
    assert paper_seek_tf_progressed(
        previous_descent_m=0.001, actual_descent_m=0.001051
    )
    assert not paper_seek_tf_progressed(
        previous_descent_m=0.001, actual_descent_m=0.001049
    )
    assert paper_seek_controller_error(
        {
            "joint_trajectory_controller": "active",
            "passthrough_trajectory_controller": "inactive",
            "force_mode_controller": "inactive",
        }
    ) is None
    assert "joint_trajectory_controller" in paper_seek_controller_error({})
    assert "force_mode_controller" in paper_seek_controller_error(
        {
            "joint_trajectory_controller": "active",
            "force_mode_controller": "active",
        }
    )
