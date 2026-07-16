import inspect
from pathlib import Path
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import PointStamped, WrenchStamped
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur3e_force_pen_writing_py.z_compliance_validation_node import (
    contact_lost,
    controller_delta,
    controllers_match,
    duration_seconds,
    execution_completed_too_early,
    FORCE,
    force_mode_request,
    line_motion_reversed,
    MOTION_CONTROLLERS,
    PASSTHROUGH,
    relative_normal_force,
    retime_passthrough_trajectory,
    retract_distance_is_stable,
    RunStopped,
    validate_joint_trajectory,
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


def test_cartesian_plan_rejects_incomplete_fraction_before_execution():
    source = inspect.getsource(ZComplianceValidationNode._plan_cartesian)

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


def test_line_reverse_watchdog_cancels_goal_before_aborting():
    source = inspect.getsource(ZComplianceValidationNode._execute_trajectory)
    watchdog = source.index("if line_motion_reversed")
    cancel = source.index("handle.cancel_goal_async()", watchdog)
    abort = source.index('"line reversed beyond 0.1mm', watchdog)

    assert watchdog < cancel < abort


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

    assert 'allow_retract=profile != "switch_hold"' in source


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
