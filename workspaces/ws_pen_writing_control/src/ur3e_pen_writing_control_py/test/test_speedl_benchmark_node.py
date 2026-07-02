import math
from pathlib import Path

from ur3e_pen_writing_control_py.pose_math import Point3
from ur3e_pen_writing_control_py.speedl_benchmark_node import (
    DEFAULT_PREHOME,
    PhaseTimes,
    Sample,
    phase_at,
    prehome_script,
    speedl_script,
    summarize_samples,
    target_distance,
)


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_ursim_speedl_benchmark.launch.py"
)


def test_speedl_scripts_have_expected_motion_and_stop():
    script = speedl_script(speed=0.06, acceleration=0.2, duration=5.0)

    assert "speedl([0.060000, 0, 0, 0, 0, 0], 0.200000, 5.000000)" in script
    assert "stopl(0.200000)" in script
    assert "movej([" in prehome_script(DEFAULT_PREHOME)


def test_phase_times_and_virtual_target_integration():
    times = PhaseTimes()

    assert times.total == 12.0
    assert phase_at(1.999, times) == "initial_settle"
    assert phase_at(2.0, times) == "speedl_plus_x"
    assert phase_at(7.0, times) == "final_settle"
    assert target_distance(1.0, 0.06, times) == 0.0
    assert math.isclose(target_distance(4.0, 0.06, times), 0.12)
    assert math.isclose(target_distance(20.0, 0.06, times), 0.30)


def test_summary_uses_middle_steady_window_and_classifies_normal_motion():
    times = PhaseTimes()
    samples = []
    for index in range(121):
        elapsed = index * 0.1
        distance = target_distance(elapsed, 0.06, times)
        point = Point3(distance, 0.0, 0.0)
        samples.append(
            Sample(
                elapsed=elapsed,
                phase=phase_at(elapsed, times),
                target=point,
                actual=point,
                target_speed=0.06 if 2.0 <= elapsed < 7.0 else 0.0,
                actual_speed=0.06 if 2.0 <= elapsed < 7.0 else 0.0,
                joint_velocities=(0.1,) * 6,
            )
        )

    summary = summarize_samples(samples, times=times)

    assert summary["verdict"] == "downstream_normal"
    assert math.isclose(summary["steady_motion_window"]["path_ratio"], 1.0)
    assert math.isclose(
        summary["steady_motion_window"]["target_path_length_m"], 0.24
    )
    assert all(
        math.isclose(value, 0.1)
        for value in summary["joint_max_abs_velocity_radps"].values()
    )


def test_launch_disables_motion_controller_and_starts_rviz_markers():
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"activate_joint_controller": "false"' in source
    assert '"launch_rviz", default_value="true"' in source
    assert "stage2_fakehardware_pen_servo.rviz" in source
    assert "scoped=True" in source
    assert "speedl_benchmark_node" in source
    assert "stop_external_control" in source
