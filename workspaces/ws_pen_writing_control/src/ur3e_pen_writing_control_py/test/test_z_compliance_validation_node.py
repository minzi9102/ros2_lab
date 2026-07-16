import inspect
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import PointStamped, WrenchStamped
import pytest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur3e_pen_writing_control_py.z_compliance_validation_node import (
    contact_lost,
    controller_delta,
    controllers_match,
    FORCE,
    force_mode_request,
    MOTION_CONTROLLERS,
    PASSTHROUGH,
    relative_normal_force,
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
        validate_joint_trajectory(_trajectory((0.0, zeros), (1.0, safe_step)))
        is None
    )
    assert "strictly increasing" in validate_joint_trajectory(
        _trajectory((0.0, zeros), (0.0, safe_step))
    )
    assert "joint-space jump" in validate_joint_trajectory(
        _trajectory((0.0, zeros), (1.0, jump))
    )


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
