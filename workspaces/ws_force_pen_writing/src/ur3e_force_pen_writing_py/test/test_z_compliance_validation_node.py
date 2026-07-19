import inspect
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import PointStamped, Pose, WrenchStamped
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import ur3e_force_pen_writing_py.z_compliance_validation_node as validation_module
from ur3e_force_pen_writing_py.geometry import Point3
from ur3e_force_pen_writing_py.z_compliance_validation_node import (
    anchored_tip_strokes,
    baseline_compensated_force_target,
    contact_lost,
    contact_force_window_is_stable,
    contact_motion_distances,
    controller_delta,
    controllers_match,
    duration_seconds,
    execution_completed_too_early,
    estimate_contact_run_sec,
    FORCE,
    force_mode_request,
    line_motion_reversed,
    MOTION_CONTROLLERS,
    PASSTHROUGH,
    path_contact_acquire_minimum,
    pen_axis_in_base_and_tilt,
    polyline_tracking,
    relative_normal_force,
    retime_passthrough_trajectory,
    retract_distance_is_stable,
    RunStopped,
    stroke_execution_distance,
    tip_path_distance,
    tool_waypoints_for_tip_targets,
    validate_joint_trajectory,
    validate_contact_strokes,
    ZComplianceValidationNode,
)


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _configure_contact_capacity(node):
    node.max_contact_execution_distance_m = 0.2
    node.max_contact_run_sec = 180.0
    node.line_speed_mps = 0.004
    node.air_speed_mps = 0.01
    node.contact_clearance_m = 0.002
    node.retract_distance_m = 0.003
    node.max_z_speed_mps = 0.0005
    node.baseline_settle_sec = 0.5
    node.baseline_duration_sec = 1.0
    node.contact_settle_sec = 1.0


def _trajectory(*samples: tuple[float, tuple[float, ...]]) -> JointTrajectory:
    trajectory = JointTrajectory(joint_names=list(JOINT_NAMES))
    for stamp, positions in samples:
        point = JointTrajectoryPoint(positions=list(positions))
        point.time_from_start.sec = int(stamp)
        point.time_from_start.nanosec = int((stamp % 1.0) * 1e9)
        trajectory.points.append(point)
    return trajectory


def test_wrench_projection_preserves_sign_and_applies_low_pass_filter():
    node = object.__new__(ZComplianceValidationNode)
    node.base_frame = "base_link"
    node.tool_frame = "tool0"
    node._lock = threading.Lock()
    node._filter_initialized = False
    node._filtered_projected_force_n = 0.0

    wrench = WrenchStamped()
    wrench.header.frame_id = node.base_frame
    wrench.wrench.force.z = -2.0
    node._on_wrench(wrench)
    assert node._raw_projected_force_n == -2.0
    assert node._filtered_projected_force_n == -2.0

    wrench.wrench.force.z = 2.0
    node._on_wrench(wrench)
    assert node._raw_projected_force_n == 2.0
    assert node._filtered_projected_force_n == pytest.approx(-1.6)
    assert relative_normal_force(
        projected_force_n=1.1, baseline_force_n=0.3
    ) == pytest.approx(0.8)


def test_controller_delta_only_switches_states_that_need_changing():
    states = {
        "joint_trajectory_controller": "active",
        "scaled_joint_trajectory_controller": "inactive",
        PASSTHROUGH: "inactive",
        FORCE: "active",
    }

    delta = controller_delta(
        states,
        activate=(PASSTHROUGH, FORCE),
        deactivate=MOTION_CONTROLLERS,
    )
    assert delta.activate == (PASSTHROUGH,)
    assert delta.deactivate == ("joint_trajectory_controller",)
    assert controllers_match(
        {PASSTHROUGH: "active", FORCE: "active"},
        active=(PASSTHROUGH, FORCE),
        inactive=MOTION_CONTROLLERS,
    )


def test_pen_axis_tilt_is_measured_absolutely_from_base_down():
    axis_base, tilt = pen_axis_in_base_and_tilt(
        validation_module.Quaternion(1.0, 0.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    assert axis_base == pytest.approx((0.0, 0.0, -1.0))
    assert tilt == pytest.approx(0.0)

    three_degrees = math.radians(3.0)
    orientation = validation_module.Quaternion(
        math.sin((math.pi - three_degrees) / 2.0),
        0.0,
        0.0,
        math.cos((math.pi - three_degrees) / 2.0),
    )
    _, tilt = pen_axis_in_base_and_tilt(orientation, (0.0, 0.0, 2.0))
    assert math.degrees(tilt) == pytest.approx(3.0)


def test_pen_axis_tilt_rejects_invalid_axis_and_upward_pen():
    with pytest.raises(ValueError, match="finite and nonzero"):
        pen_axis_in_base_and_tilt(
            validation_module.Quaternion(0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
        )
    _, tilt = pen_axis_in_base_and_tilt(
        validation_module.Quaternion(0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 1.0),
    )
    assert math.degrees(tilt) == pytest.approx(180.0)


def test_absolute_pen_tilt_precheck_rejects_before_controller_switch():
    node = object.__new__(ZComplianceValidationNode)
    node.pen_axis_tool_xyz = (0.0, 0.0, 1.0)
    node.max_pen_tilt_rad = math.radians(1.0)
    pose = Pose()
    three_degrees = math.radians(3.0)
    pose.orientation.x = math.sin((math.pi - three_degrees) / 2.0)
    pose.orientation.w = math.cos((math.pi - three_degrees) / 2.0)
    node._current_tool_pose_stamped = lambda: SimpleNamespace(pose=pose)

    with pytest.raises(RunStopped, match="tilt=3.000deg limit=1.000deg"):
        node._assert_pen_axis_tilt()

    run_source = inspect.getsource(ZComplianceValidationNode._run_profile)
    assert run_source.index("self._precheck()") < run_source.index(
        "self._switch_to_passthrough_force()"
    )

def test_cartesian_plan_rejects_incomplete_fraction_before_execution():
    source = inspect.getsource(ZComplianceValidationNode._plan_tip_targets)

    assert "result.fraction < 0.999" in source
    assert source.index("result.fraction < 0.999") < source.index(
        "validate_joint_trajectory"
    )


def test_trajectory_requires_increasing_timestamps_and_bounded_joint_steps():
    zeros = (0.0,) * 6
    safe_step = (0.1,) + (0.0,) * 5
    jump = (0.21,) + (0.0,) * 5

    assert (
        validate_joint_trajectory(_trajectory((0.1, zeros), (1.0, safe_step)))
        is None
    )
    assert "first time_from_start must be positive" in validate_joint_trajectory(
        _trajectory((0.0, zeros), (1.0, safe_step))
    )
    assert "strictly increasing" in validate_joint_trajectory(
        _trajectory((0.1, zeros), (0.1, safe_step))
    )
    assert "joint-space jump" in validate_joint_trajectory(
        _trajectory((0.1, zeros), (1.0, jump))
    )
    incomplete_velocity = _trajectory((0.1, zeros), (1.0, safe_step))
    incomplete_velocity.points[0].velocities = [0.0]
    assert "incomplete joint velocities" in validate_joint_trajectory(
        incomplete_velocity
    )
    incomplete_acceleration = _trajectory((0.1, zeros), (1.0, safe_step))
    incomplete_acceleration.points[0].accelerations = [0.0]
    assert "incomplete joint accelerations" in validate_joint_trajectory(
        incomplete_acceleration
    )


def test_passthrough_retiming_enforces_distance_over_speed_and_preserves_path():
    zeros = (0.0,) * 6
    middle = (0.05,) + (0.0,) * 5
    safe_step = (0.1,) + (0.0,) * 5
    trajectory = _trajectory((0.0, zeros), (0.2, middle), (0.5, safe_step))
    trajectory.points[1].velocities = [1.0] * 6
    trajectory.points[1].accelerations = [2.0] * 6
    trajectory.points[1].effort = [3.0] * 6

    duration, scale = retime_passthrough_trajectory(
        trajectory, distance_m=0.01, speed_mps=0.002
    )

    assert duration == pytest.approx(5.0)
    assert scale == pytest.approx(10.0)
    assert [
        duration_seconds(point.time_from_start) for point in trajectory.points
    ] == pytest.approx([0.1, 2.1, 5.1])
    assert tuple(trajectory.points[0].positions) == zeros
    assert tuple(trajectory.points[1].positions) == middle
    assert tuple(trajectory.points[2].positions) == safe_step
    assert trajectory.points[1].velocities == pytest.approx([0.1] * 6)
    assert trajectory.points[1].accelerations == pytest.approx([0.02] * 6)
    assert trajectory.points[1].effort == pytest.approx([3.0] * 6)
    assert not trajectory.points[0].velocities
    assert not trajectory.points[0].accelerations
    assert validate_joint_trajectory(trajectory) is None


@pytest.mark.parametrize(
    ("distance_m", "expected_duration"), ((0.01, 5.0), (0.003, 1.5))
)
def test_passthrough_retiming_matches_line_and_retract_duration(
    distance_m, expected_duration
):
    trajectory = _trajectory((0.0, (0.0,) * 6), (0.5, (0.1,) + (0.0,) * 5))

    duration, _ = retime_passthrough_trajectory(
        trajectory, distance_m=distance_m, speed_mps=0.002
    )

    first = duration_seconds(trajectory.points[0].time_from_start)
    final = duration_seconds(trajectory.points[-1].time_from_start)
    assert duration == pytest.approx(expected_duration)
    assert final - first == pytest.approx(expected_duration)


def test_contact_retiming_uses_one_goal_with_slow_entry_and_smooth_ramp():
    samples = tuple(
        (index * 0.25, (index * 0.025,) + (0.0,) * 5)
        for index in range(5)
    )
    trajectory = _trajectory(*samples)
    for point in trajectory.points:
        point.velocities = [1.0] * 6
        point.accelerations = [2.0] * 6

    duration, _ = retime_passthrough_trajectory(
        trajectory,
        distance_m=0.006,
        speed_mps=0.003,
        entry_speed_mps=0.002,
        entry_distance_m=0.0015,
        entry_ramp_distance_m=0.0015,
    )

    assert duration == pytest.approx(2.35)
    assert [
        duration_seconds(point.time_from_start) for point in trajectory.points
    ] == pytest.approx([0.1, 0.85, 1.45, 1.95, 2.45])
    assert all(point.velocities for point in trajectory.points)
    assert all(not point.accelerations for point in trajectory.points)
    assert validate_joint_trajectory(trajectory) is None


def test_passthrough_retiming_rejects_invalid_inputs_and_early_completion():
    trajectory = _trajectory((0.0, (0.0,) * 6), (1.0, (0.1,) + (0.0,) * 5))

    with pytest.raises(ValueError, match="distance and speed must be positive"):
        retime_passthrough_trajectory(
            trajectory, distance_m=0.01, speed_mps=0.0
        )
    with pytest.raises(ValueError, match="distance and speed must be positive"):
        retime_passthrough_trajectory(
            trajectory, distance_m=0.0, speed_mps=0.002
        )
    with pytest.raises(ValueError, match="duration must be positive"):
        retime_passthrough_trajectory(
            _trajectory((0.0, (0.0,) * 6), (0.0, (0.1,) + (0.0,) * 5)),
            distance_m=0.01,
            speed_mps=0.002,
        )
    assert execution_completed_too_early(
        elapsed_sec=0.7, commanded_motion_sec=5.0
    )
    assert not execution_completed_too_early(
        elapsed_sec=4.5, commanded_motion_sec=5.0
    )


def test_line_reverse_watchdog_tolerates_noise_and_rejects_backtracking():
    assert not line_motion_reversed(
        progress_m=0.0019, furthest_progress_m=0.0020
    )
    assert line_motion_reversed(
        progress_m=0.001899, furthest_progress_m=0.0020
    )


def test_contact_path_requires_force_margin_above_steady_lower_bound():
    assert path_contact_acquire_minimum(
        target_force_n=0.8, steady_force_min_n=0.5
    ) == pytest.approx(0.7)
    assert path_contact_acquire_minimum(
        target_force_n=0.55, steady_force_min_n=0.5
    ) == pytest.approx(0.5)


def test_contact_path_force_window_requires_mean_and_coverage():
    assert contact_force_window_is_stable(
        [0.60, 0.72, 0.78, 0.74, 0.70],
        minimum_mean_n=0.7,
        steady_min_n=0.5,
        steady_max_n=1.1,
    )
    assert not contact_force_window_is_stable(
        [0.60, 0.62, 0.64, 0.66, 0.68],
        minimum_mean_n=0.7,
        steady_min_n=0.5,
        steady_max_n=1.1,
    )
    assert not contact_force_window_is_stable(
        [0.80] * 8 + [0.20] * 2,
        minimum_mean_n=0.7,
        steady_min_n=0.5,
        steady_max_n=1.1,
    )


def test_contact_motion_profile_preserves_a_steady_tail():
    assert contact_motion_distances(0.010) == pytest.approx(
        (0.0015, 0.0015, 0.003)
    )
    assert contact_motion_distances(0.002) == pytest.approx((0.002, 0.0, 0.001))
    with pytest.raises(ValueError, match="must be positive"):
        contact_motion_distances(0.0)


def test_force_target_compensates_bounded_relative_baseline():
    assert baseline_compensated_force_target(
        relative_target_n=0.8, baseline_force_n=0.239
    ) == pytest.approx(1.039)
    assert baseline_compensated_force_target(
        relative_target_n=0.8, baseline_force_n=-0.172
    ) == pytest.approx(0.628)
    with pytest.raises(ValueError, match="compensation limit"):
        baseline_compensated_force_target(
            relative_target_n=0.8, baseline_force_n=0.301
        )
    with pytest.raises(ValueError, match="must be positive"):
        baseline_compensated_force_target(
            relative_target_n=0.2, baseline_force_n=-0.2
        )


def test_line_reverse_watchdog_cancels_goal_before_aborting():
    source = inspect.getsource(ZComplianceValidationNode._execute_trajectory)
    watchdog = source.index("if line_motion_reversed")
    cancel = source.index("handle.cancel_goal_async()", watchdog)
    abort = source.index("reversed beyond 0.1mm", watchdog)

    assert watchdog < cancel < abort


def test_polyline_tracking_reports_progress_and_lateral_error_across_v_corner():
    path = [
        Point3(0.0, 0.0, 0.0),
        Point3(0.003, -0.004, 0.0),
        Point3(0.006, 0.0, 0.0),
    ]

    first = polyline_tracking(Point3(0.0015, -0.002, 0.2), path)
    second = polyline_tracking(Point3(0.0045, -0.0018, -0.1), path)

    assert first.progress_m == pytest.approx(0.0025)
    assert first.lateral_error_m == pytest.approx(0.0)
    assert second.progress_m > 0.005
    assert second.lateral_error_m == pytest.approx(0.00012)
    assert second.total_length_m == pytest.approx(0.01)


def test_contact_path_allows_multiple_strokes_with_first_complexity_tier():
    valid = [
        [(0.0, 0.0), (0.020, 0.0)],
        [(0.0, 0.010), (0.030, 0.010)],
    ]

    assert validate_contact_strokes(valid, speed_mps=0.002) == valid
    with pytest.raises(ValueError, match="exceeds 75mm"):
        validate_contact_strokes(
            [[(0.0, 0.0), (0.075001, 0.0)]], speed_mps=0.002
        )
    with pytest.raises(ValueError, match="total contact path length exceeds 120mm"):
        validate_contact_strokes(
            [
                [(0.0, 0.0), (0.061, 0.0)],
                [(0.0, 0.010), (0.061, 0.010)],
            ],
            speed_mps=0.002,
        )
    with pytest.raises(ValueError, match="stroke count exceeds 12"):
        validate_contact_strokes(
            [[(0.0, 0.0), (0.001, 0.0)]] * 13, speed_mps=0.002
        )
    with pytest.raises(ValueError, match="writing time exceeds 60s"):
        validate_contact_strokes(valid, speed_mps=0.0005)


def test_contact_run_estimate_includes_writing_air_and_per_stroke_overhead():
    estimate = estimate_contact_run_sec(
        pen_down_length_m=0.08,
        execution_distance_m=0.12,
        stroke_count=4,
        contact_speed_mps=0.004,
        air_speed_mps=0.01,
        contact_clearance_m=0.002,
        retract_distance_m=0.003,
        max_z_speed_mps=0.0005,
        baseline_settle_sec=0.5,
        baseline_duration_sec=1.0,
        contact_settle_sec=1.0,
    )

    assert estimate == pytest.approx(61.9)


def test_contact_path_capacity_checks_precede_controller_switch():
    source = inspect.getsource(ZComplianceValidationNode._run_contact_path)

    switch = source.index("self._switch_to_passthrough_force()")
    assert source.index("execution distance exceeds") < switch
    assert source.index("estimated contact run time exceeds") < switch


def test_contact_run_deadline_aborts_active_character():
    node = object.__new__(ZComplianceValidationNode)
    node._abort_event = threading.Event()
    node._contact_run_deadline = time.monotonic() - 1.0
    node.max_contact_run_sec = 180.0

    with pytest.raises(RunStopped, match="contact run time exceeded 180s"):
        node._raise_if_stopped()


def test_handwriting_strokes_are_centered_on_anchor_at_fixed_air_gap():
    strokes = anchored_tip_strokes(
        [[(-0.005, 0.005), (0.005, -0.005)]],
        anchor=Point3(0.4, 0.1, 0.2),
        tip_z=0.203,
    )

    assert (strokes[0][0].x, strokes[0][0].y, strokes[0][0].z) == pytest.approx(
        (0.395, 0.105, 0.203)
    )
    assert (strokes[0][1].x, strokes[0][1].y, strokes[0][1].z) == pytest.approx(
        (0.405, 0.095, 0.203)
    )


def test_tip_targets_translate_tool_pose_without_rotating_it():
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (0.3, 0.2, 0.1)
    pose.orientation.x, pose.orientation.w = (0.2, 0.98)
    start = Point3(0.4, 0.1, 0.05)
    targets = [Point3(0.405, 0.095, 0.053), Point3(0.41, 0.1, 0.053)]

    waypoints = tool_waypoints_for_tip_targets(pose, start, targets)

    assert tip_path_distance(start, targets) == pytest.approx(
        (0.005**2 + 0.005**2 + 0.003**2) ** 0.5
        + (0.005**2 + 0.005**2) ** 0.5
    )
    assert (waypoints[0].position.x, waypoints[0].position.y, waypoints[0].position.z) == pytest.approx(
        (0.305, 0.195, 0.103)
    )
    assert waypoints[0].orientation == pose.orientation
    assert waypoints[1].orientation == pose.orientation


def test_air_execution_distance_includes_pen_up_transitions():
    strokes = [
        [Point3(0.001, 0.0, 0.003), Point3(0.002, 0.0, 0.003)],
        [Point3(0.002, 0.003, 0.003), Point3(0.004, 0.003, 0.003)],
    ]

    distance = stroke_execution_distance(Point3(0.0, 0.0, 0.0), strokes)

    assert distance == pytest.approx(
        math.sqrt(0.001**2 + 0.003**2) + 0.001 + 0.003 + 0.002
    )


def test_tip_waypoints_can_lock_one_orientation_across_segments():
    pose = Pose()
    pose.orientation.w = 1.0
    fixed = Pose().orientation
    fixed.z, fixed.w = (0.2, 0.98)

    waypoints = tool_waypoints_for_tip_targets(
        pose,
        Point3(0.0, 0.0, 0.0),
        [Point3(0.001, 0.0, 0.0)],
        fixed,
    )

    assert waypoints[0].orientation == fixed


def test_air_path_profile_skips_force_mode_and_monitors_live_data():
    run_source = inspect.getsource(ZComplianceValidationNode._run_profile)
    execute_source = inspect.getsource(ZComplianceValidationNode._execute_trajectory)

    assert 'profile not in ("switch_hold", "path_air")' in run_source
    assert 'elif profile == "path_air"' in run_source
    assert 'allow_retract=profile not in ("switch_hold", "path_air")' in run_source
    assert "elif monitor_live" in execute_source
    assert "self._assert_live_data()" in execute_source


def test_contact_and_air_motion_use_independent_speeds():
    air_source = inspect.getsource(ZComplianceValidationNode._execute_air_tip_targets)
    contact_source = inspect.getsource(ZComplianceValidationNode._write_contact_path)
    lift_source = inspect.getsource(ZComplianceValidationNode._lift_between_contact_strokes)
    cleanup_source = inspect.getsource(ZComplianceValidationNode._safe_cleanup)

    assert "speed_mps=self.air_speed_mps" in air_source
    assert "distance / self.air_speed_mps" in air_source
    assert "speed_mps=self.line_speed_mps" in contact_source
    assert "distance / self.line_speed_mps" in contact_source
    assert "speed_mps=self.air_speed_mps" in lift_source
    assert "speed_mps=self.air_speed_mps" in cleanup_source


def test_contact_writing_reuses_acquisition_gate_for_single_goal_entry():
    run_source = inspect.getsource(ZComplianceValidationNode._run_profile)
    path_source = inspect.getsource(ZComplianceValidationNode._run_contact_path)
    write_source = inspect.getsource(ZComplianceValidationNode._write_contact_path)
    execute_source = inspect.getsource(ZComplianceValidationNode._execute_trajectory)

    assert run_source.index("self._acquire_contact(start_tip)") < run_source.index(
        "self._write_line()"
    )
    assert path_source.index("self._acquire_contact(") < path_source.index(
        "self._write_contact_path"
    )
    assert "_confirm_quiet_contact" not in run_source
    assert "_confirm_quiet_contact" not in path_source
    assert "contact_motion=True" in write_source
    assert "steady_force_start_progress_m=steady_start_m" in write_source
    assert "CONTACT_ENTRY" in write_source
    assert "CONTACT_STEADY_WRITE" in execute_source
    assert "contact entry stayed below steady force for 0.3s" in execute_source
    assert "self._assert_tip_endpoint(endpoint, monitor_contact=True)" in write_source
    assert "self._assert_tip_endpoint(endpoint, monitor_contact=True)" in inspect.getsource(
        ZComplianceValidationNode._write_line
    )


def test_air_endpoint_waits_for_transient_tracking_lag(monkeypatch):
    monkeypatch.setattr(validation_module, "PATH_ENDPOINT_SETTLE_TIMEOUT_SEC", 0.08)
    monkeypatch.setattr(validation_module, "PATH_ENDPOINT_STABLE_WINDOW_SEC", 0.02)
    node = object.__new__(ZComplianceValidationNode)
    target = Point3(0.0, 0.0, 0.0)
    checks = []
    messages = []
    samples = iter(
        [Point3(0.000555, 0.0, 0.0), Point3(0.0, 0.0, 0.0)]
    )
    current = Point3(0.0, 0.0, 0.0)
    node._assert_live_data = lambda: checks.append(True)
    node._current_tip = lambda: next(samples, current)
    node.get_logger = lambda: SimpleNamespace(info=messages.append)

    node._assert_tip_endpoint(target)

    assert len(checks) >= 3
    assert messages and "initial_error=0.000555m" in messages[0]


def test_air_endpoint_aborts_when_error_never_settles(monkeypatch):
    monkeypatch.setattr(validation_module, "PATH_ENDPOINT_SETTLE_TIMEOUT_SEC", 0.03)
    monkeypatch.setattr(validation_module, "PATH_ENDPOINT_STABLE_WINDOW_SEC", 0.01)
    node = object.__new__(ZComplianceValidationNode)
    node._assert_live_data = lambda: None
    node._current_tip = lambda: Point3(0.000555, 0.0, 0.0)

    with pytest.raises(RunStopped, match="after settling: 0.000555m"):
        node._assert_tip_endpoint(Point3(0.0, 0.0, 0.0))


def test_air_endpoint_settle_wait_propagates_live_safety_failure():
    node = object.__new__(ZComplianceValidationNode)
    node._assert_live_data = lambda: (_ for _ in ()).throw(
        RunStopped("joint state timed out")
    )
    node._current_tip = lambda: pytest.fail("endpoint read after safety failure")

    with pytest.raises(RunStopped, match="joint state timed out"):
        node._assert_tip_endpoint(Point3(0.0, 0.0, 0.0))


def test_contact_endpoint_wait_keeps_force_and_path_safety_active(monkeypatch):
    monkeypatch.setattr(validation_module, "PATH_ENDPOINT_SETTLE_TIMEOUT_SEC", 0.08)
    monkeypatch.setattr(validation_module, "PATH_ENDPOINT_STABLE_WINDOW_SEC", 0.02)
    node = object.__new__(ZComplianceValidationNode)
    target = Point3(0.0, 0.0, 0.0)
    calls = []
    samples = iter([Point3(0.000798, 0.0, 0.0), target])
    node.lost_contact_force_n = 0.2
    node.lost_contact_duration_sec = 0.3
    node._assert_live_data = lambda **kwargs: (
        calls.append(("live", kwargs)) or 0.8
    )
    node._assert_contact_z_offset = lambda: calls.append(("z", {}))
    node._tracking_errors = lambda _point: (0.0, 0.0)
    node._current_tip = lambda: next(samples, target)
    node.get_logger = lambda: SimpleNamespace(
        info=lambda message: calls.append(("log", message))
    )

    node._assert_tip_endpoint(target, monitor_contact=True)

    assert all(
        kwargs == {"check_contact_force": True}
        for kind, kwargs in calls
        if kind == "live"
    )
    assert any(kind == "z" for kind, _value in calls)
    assert any(
        kind == "log" and "Contact path endpoint settled" in message
        for kind, message in calls
    )


def test_contact_endpoint_wait_propagates_force_safety_failure():
    node = object.__new__(ZComplianceValidationNode)
    node._assert_live_data = lambda **_kwargs: (_ for _ in ()).throw(
        RunStopped("filtered force limit exceeded")
    )
    node._current_tip = lambda: pytest.fail("endpoint read after force failure")

    with pytest.raises(RunStopped, match="filtered force limit exceeded"):
        node._assert_tip_endpoint(Point3(0.0, 0.0, 0.0), monitor_contact=True)


def test_contact_path_refreshes_air_baseline_before_each_stroke():
    node = object.__new__(ZComplianceValidationNode)
    _configure_contact_capacity(node)
    strokes = [
        [Point3(0.0, 0.0, 0.003), Point3(0.005, 0.0, 0.003)],
        [Point3(0.0, 0.005, 0.003), Point3(0.005, 0.005, 0.003)],
    ]
    orientation = Pose().orientation
    orientation.w = 1.0
    events = []
    node.target_force_n = 0.8
    node.steady_force_min_n = 0.5
    node._compile_contact_strokes = lambda: strokes
    node._current_tool_pose_stamped = lambda: SimpleNamespace(
        pose=SimpleNamespace(orientation=orientation)
    )
    node._current_tip = lambda: Point3(0.0, 0.0, 0.003)
    node._switch_to_passthrough_force = lambda: events.append("switch")
    node._send_hold_current_joints = lambda: events.append("hold")
    node._publish_status = lambda *_args: None
    node._execute_air_tip_targets = lambda *_args: events.append("air_move")
    node._prepare_force_baseline = lambda: events.append("baseline")
    node._start_force_mode = lambda _force: events.append("force_start")
    node._acquire_contact = lambda _start, **kwargs: events.append(
        f"contact:{kwargs['minimum_mean_force_n']:.1f}"
    )
    node._write_contact_path = lambda *_args: events.append("write")
    node._lift_between_contact_strokes = lambda: events.append("pen_up")

    node._run_contact_path()

    assert events == [
        "switch",
        "hold",
        "air_move",
        "baseline",
        "force_start",
        "contact:0.7",
        "write",
        "pen_up",
        "air_move",
        "baseline",
        "force_start",
        "contact:0.7",
        "write",
    ]


def test_contact_path_uses_shorter_clearance_without_reducing_final_retract():
    compile_source = inspect.getsource(ZComplianceValidationNode._compile_contact_strokes)
    lift_source = inspect.getsource(ZComplianceValidationNode._lift_between_contact_strokes)
    cleanup_source = inspect.getsource(ZComplianceValidationNode._safe_cleanup)

    assert "paper.z + self.contact_clearance_m" in compile_source
    assert "delta_z=self.contact_clearance_m" in lift_source
    assert "expected_distance_m=self.contact_clearance_m" in lift_source
    assert "delta_z=self.retract_distance_m" in cleanup_source


def test_pen_up_between_strokes_holds_stops_force_then_retracts():
    node = object.__new__(ZComplianceValidationNode)
    events = []
    node._active_stroke_index = 1
    node._stroke_count = 2
    node._force_started = True
    node._active_target_force_n = 0.8
    node._pen_state = "pen_down"
    node.retract_distance_m = 0.003
    node.contact_clearance_m = 0.002
    node.air_speed_mps = 0.005
    node._stop_force_client = object()
    node._publish_status = lambda *_args: events.append("status")
    node._send_hold_current_joints = lambda **_kwargs: events.append("hold")
    node._call = lambda *_args, **_kwargs: (
        events.append("stop") or SimpleNamespace(success=True)
    )
    node._current_tip = lambda: Point3(0.0, 0.0, 0.0)

    def plan(**_kwargs):
        events.append("plan_retract")
        return object()

    node._plan_cartesian = plan
    node._execute_trajectory = lambda *_args, **_kwargs: events.append("retract")
    node._wait_for_stable_retract = (
        lambda *_args, **_kwargs: events.append("stable")
    )
    node._contact_tip = Point3(0.0, 0.0, 0.0)
    node._force_start_tip = Point3(0.0, 0.0, 0.003)
    node._force_start_pose = object()

    node._lift_between_contact_strokes()

    assert events == ["status", "hold", "stop", "plan_retract", "retract", "stable"]
    assert not node._force_started
    assert node._active_target_force_n == 0.0
    assert node._pen_state == "pen_up"
    assert node._contact_tip is None


def test_contact_path_stops_entire_character_when_a_stroke_fails():
    node = object.__new__(ZComplianceValidationNode)
    _configure_contact_capacity(node)
    strokes = [
        [Point3(0.0, y, 0.003), Point3(0.005, y, 0.003)]
        for y in (0.0, 0.005, 0.010)
    ]
    orientation = Pose().orientation
    orientation.w = 1.0
    writes = []
    node.target_force_n = 0.8
    node.steady_force_min_n = 0.5
    node._compile_contact_strokes = lambda: strokes
    node._current_tool_pose_stamped = lambda: SimpleNamespace(
        pose=SimpleNamespace(orientation=orientation)
    )
    node._current_tip = lambda: Point3(0.0, 0.0, 0.003)
    node._switch_to_passthrough_force = lambda: None
    node._send_hold_current_joints = lambda: None
    node._publish_status = lambda *_args: None
    node._execute_air_tip_targets = lambda *_args: None
    node._prepare_force_baseline = lambda: None
    node._start_force_mode = lambda _force: None
    node._acquire_contact = lambda *_args, **_kwargs: None
    node._lift_between_contact_strokes = lambda: None

    def write(*_args):
        writes.append(node._active_stroke_index)
        if node._active_stroke_index == 2:
            raise RunStopped("stroke 2 failed")

    node._write_contact_path = write

    with pytest.raises(RunStopped, match="stroke 2 failed"):
        node._run_contact_path()

    assert writes == [1, 2]


def test_contact_path_watchdog_cancels_for_lateral_error_and_backtracking():
    source = inspect.getsource(ZComplianceValidationNode._execute_trajectory)
    path_branch = source.index('self._profile == "path_contact"')
    lateral = source.index("tracking.lateral_error_m >", path_branch)
    lateral_cancel = source.index("handle.cancel_goal_async()", lateral)
    lateral_abort = source.index("lateral error exceeded 0.5mm", lateral)
    reverse = source.index("if line_motion_reversed", path_branch)
    reverse_cancel = source.index("handle.cancel_goal_async()", reverse)

    assert path_branch < lateral < lateral_cancel < lateral_abort
    assert reverse < reverse_cancel


def test_csv_logging_uses_unique_run_numbers_without_overwriting(tmp_path: Path):
    existing = tmp_path / "line_001.csv"
    existing.write_text("existing\n", encoding="utf-8")
    messages = []
    node = object.__new__(ZComplianceValidationNode)
    node._run_directory = tmp_path
    node._csv_run_index = 0
    node._csv_file = None
    node._csv_writer = None
    node.get_logger = lambda: SimpleNamespace(info=messages.append)

    node._open_csv("line")
    node._close_csv()
    node._open_csv("line")
    node._close_csv()

    assert existing.read_text(encoding="utf-8") == "existing\n"
    assert sorted(path.name for path in tmp_path.glob("line_*.csv")) == [
        "line_001.csv",
        "line_002.csv",
        "line_003.csv",
    ]
    assert messages[-1].endswith("line_003.csv")
    header = (tmp_path / "line_003.csv").read_text(encoding="utf-8").splitlines()[0]
    assert "trajectory_file_id" in header
    assert "stroke_index" in header
    assert "planned_progress_m" in header
    assert "lateral_error_m" in header
    assert "pen_state" in header


def test_force_mode_request_commands_only_negative_base_z_compliance():
    paper = PointStamped()
    paper.header.frame_id = "base_link"
    paper.point.x, paper.point.y, paper.point.z = (0.4, 0.1, 0.2)

    request = force_mode_request(
        paper_point=paper,
        target_force_n=0.8,
        speed_limit_mps=0.0005,
        damping_factor=0.5,
        gain_scaling=0.3,
        xy_limit_m=0.003,
        rotation_limit_rad=0.02,
    )

    assert request.task_frame.header.frame_id == "base_link"
    assert request.selection_vector_z is True
    assert request.wrench.force.z == -0.8
    assert request.speed_limits.linear.z == 0.0005
    assert list(request.deviation_limits[:3]) == pytest.approx(
        [0.003, 0.003, 0.0005]
    )


def test_contact_loss_requires_continuous_time_below_threshold():
    lost, started = contact_lost(
        force_n=0.1,
        threshold_n=0.2,
        below_since=None,
        now=10.0,
        duration=0.3,
    )
    assert not lost
    assert started == 10.0
    assert contact_lost(
        force_n=0.1,
        threshold_n=0.2,
        below_since=started,
        now=10.3,
        duration=0.3,
    )[0]
    assert contact_lost(
        force_n=0.2,
        threshold_n=0.2,
        below_since=started,
        now=10.4,
        duration=0.3,
    ) == (False, None)


def test_service_interface_names_are_stable_and_manual_only():
    source = inspect.getsource(ZComplianceValidationNode.__init__)

    for name in (
        "start_switch_hold",
        "start_direction",
        "start_contact_hold",
        "start_line",
        "start_path_air",
        "start_path_contact",
    ):
        assert f'(\"{name}\",' in source
    assert 'f"/pen_writing/z_compliance/{service_name}"' in source
    assert '"/pen_writing/z_compliance/stop"' in source


def test_cleanup_order_is_cancel_hold_stop_force_retract_restore():
    source = inspect.getsource(ZComplianceValidationNode._safe_cleanup)
    calls = (
        "self._cancel_active_goal(wait=True)",
        "self._send_hold_current_joints(publish_state=False)",
        "self._stop_force_client",
        "self._plan_cartesian(",
        "self._wait_for_stable_retract(retract_start)",
        "self._restore_controllers()",
    )

    assert [source.index(call) for call in calls] == sorted(
        source.index(call) for call in calls
    )


def test_switch_hold_cleanup_explicitly_disables_retraction():
    source = inspect.getsource(ZComplianceValidationNode._run_profile)

    assert 'allow_retract=profile not in ("switch_hold", "path_air")' in source


def test_controller_verification_failure_keeps_restore_flag_armed():
    node = object.__new__(ZComplianceValidationNode)
    initial = {
        "joint_trajectory_controller": "active",
        PASSTHROUGH: "inactive",
        FORCE: "inactive",
    }
    invalid_after_switch = dict(initial)
    responses = iter((initial, invalid_after_switch))
    node._list_controllers = lambda: next(responses)
    node._publish_status = lambda *_args: None
    node._switch = lambda _delta: None
    node._controllers_switched = False

    with pytest.raises(RunStopped, match="verification failed"):
        node._switch_to_passthrough_force()

    assert node._controllers_switched is True


def test_force_start_rejection_does_not_arm_force_cleanup():
    node = object.__new__(ZComplianceValidationNode)
    node._paper_point = PointStamped()
    node._current_tip = lambda: SimpleNamespace(x=0.0, y=0.0, z=0.1)
    node._current_tool_pose_stamped = lambda: SimpleNamespace()
    node.direction_force_n = 0.2
    node.max_z_speed_mps = 0.0005
    node.damping_factor = 0.5
    node.gain_scaling = 0.3
    node.max_xy_error_m = 0.003
    node.max_rotation_error_rad = 0.02
    node._start_force_client = object()
    node._call = lambda *_args, **_kwargs: SimpleNamespace(success=False)
    node._baseline_force_n = 0.0
    node._force_started = False

    with pytest.raises(RunStopped, match="start_force_mode failed"):
        node._start_force_mode(node.direction_force_n)

    assert node._force_started is False


def test_live_data_rejects_stale_wrench_before_force_use():
    node = object.__new__(ZComplianceValidationNode)
    node._abort_event = threading.Event()
    node.data_timeout_sec = 0.2
    node._last_wrench_time = time.monotonic() - 1.0
    node._last_joint_time = time.monotonic()

    with pytest.raises(RunStopped, match="wrench data timed out"):
        node._assert_live_data()


def test_force_stop_failure_skips_blind_retract_and_restore():
    node = object.__new__(ZComplianceValidationNode)
    node._abort_event = threading.Event()
    node._controllers_switched = True
    node._force_started = True
    node._publish_status = lambda *_args: None
    node._cancel_active_goal = lambda **_kwargs: None
    node._send_hold_current_joints = lambda **_kwargs: None
    node._stop_force_client = object()
    node._call = lambda *_args, **_kwargs: SimpleNamespace(success=False)
    node._plan_cartesian = lambda **_kwargs: pytest.fail("unexpected retract")
    node._restore_controllers = lambda: pytest.fail("unexpected restore")

    error = node._safe_cleanup(allow_retract=True)

    assert "stop force failed; use robot stop" in error


def test_retract_acceptance_requires_distance_band_and_stability():
    assert retract_distance_is_stable(
        [0.00295, 0.00300, 0.00304], minimum_m=0.002, maximum_m=0.004
    )
    assert not retract_distance_is_stable(
        [0.00095, 0.00100], minimum_m=0.002, maximum_m=0.004
    )
    assert not retract_distance_is_stable(
        [0.0021, 0.0038], minimum_m=0.002, maximum_m=0.004
    )


def test_retract_failure_still_restores_original_controllers():
    node = object.__new__(ZComplianceValidationNode)
    node._abort_event = threading.Event()
    node._controllers_switched = True
    node._force_started = True
    node.retract_distance_m = 0.003
    node.air_speed_mps = 0.005
    node._publish_status = lambda *_args: None
    node._cancel_active_goal = lambda **_kwargs: None
    node._send_hold_current_joints = lambda **_kwargs: None
    node._stop_force_client = object()
    node._call = lambda *_args, **_kwargs: SimpleNamespace(success=True)
    node._safety_is_normal = lambda: True
    node._current_tip = lambda: SimpleNamespace(x=0.0, y=0.0, z=0.1)
    node._plan_cartesian = lambda **_kwargs: object()
    node._execute_trajectory = lambda *_args, **_kwargs: None
    node._wait_for_stable_retract = lambda _start: (_ for _ in ()).throw(
        RunStopped("retract TF stalled")
    )
    restored = []
    node._restore_controllers = lambda: restored.append(True)

    error = node._safe_cleanup(allow_retract=True)

    assert "retract failed: retract TF stalled" in error
    assert restored == [True]
