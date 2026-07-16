import math

import rclpy
from rclpy.context import Context
from rclpy.node import Node

from ur3e_force_pen_writing_py.geometry import Point3, Quaternion, transform_point
import ur3e_force_pen_writing_py.paper_seek_servo_node as paper_seek_module
from ur3e_force_pen_writing_py.paper_seek_servo_node import (
    ABORTED,
    BASELINING,
    DESCENDING,
    IDLE,
    WAITING_FOR_WRENCH,
    PaperSeekServoNode,
    contact_force_from_baseline,
    lowpass_force_z,
    next_paper_seek_offset,
    paper_seek_baseline_stats,
    paper_seek_controller_error,
    paper_seek_dynamic_threshold,
    paper_seek_tf_progressed,
    paper_seek_tool_pose_target,
)


class SeekHarness:
    def __init__(self, state=WAITING_FOR_WRENCH):
        self._state = state
        self._state_started = 10.0
        self._last_timer = 10.0
        self._last_progress = 10.0
        self._last_wrench_time = 0.0
        self._wrench_sequence = 0
        self._evaluated_sequence = 0
        self._filtered_force = 0.0
        self._filter_initialized = False
        self._baseline_samples = []
        self._baseline_n = 0.0
        self._threshold_n = 0.5
        self._last_actual_descent = 0.0
        self._contact_count = 0
        self._candidate = None
        self._retract_target_z = None
        self._snapshot = object()
        self._offset_m = 0.0
        self.wrench_timeout_sec = 0.2
        self.baseline_duration_sec = 1.0
        self.contact_threshold_n = 0.5
        self.sigma_multiplier = 6.0
        self.lowpass_alpha = 0.1
        self.abort_reason = None
        self.published_offsets = []
        self.statuses = []

    def _ensure_pose_mode(self):
        return None

    def _servo_healthy(self, _now):
        return True

    def _abort(self, reason):
        self.abort_reason = reason
        PaperSeekServoNode._abort(self, reason)

    def _publish_status(self, detail, *, error=False):
        self.statuses.append((self._state, detail, error))

    def _publish_target(self, _now):
        self.published_offsets.append(self._offset_m)


class StatusPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


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


def test_post_zero_timer_waits_for_first_wrench_and_holds_pose(monkeypatch):
    node = SeekHarness()
    monkeypatch.setattr(paper_seek_module.time, "monotonic", lambda: 10.01)

    PaperSeekServoNode._tick(node)

    assert node._state == WAITING_FOR_WRENCH
    assert node.abort_reason is None
    assert node.published_offsets == [0.0]


def test_post_zero_wrench_timeout_aborts_without_publishing_target(monkeypatch):
    node = SeekHarness()
    monkeypatch.setattr(paper_seek_module.time, "monotonic", lambda: 10.201)

    PaperSeekServoNode._tick(node)
    PaperSeekServoNode._tick(node)

    assert node._state == ABORTED
    assert node.abort_reason == "post-zero wrench data timed out"
    assert node.published_offsets == []


def test_wrench_timeout_after_first_sample_aborts_without_target(monkeypatch):
    node = SeekHarness(state=BASELINING)
    node._last_wrench_time = 10.0
    node._baseline_samples = [0.4]
    monkeypatch.setattr(paper_seek_module.time, "monotonic", lambda: 10.201)

    PaperSeekServoNode._tick(node)

    assert node._state == ABORTED
    assert node.abort_reason == "wrench data timed out"
    assert node.published_offsets == []


def test_first_post_zero_wrench_starts_a_full_baseline(monkeypatch):
    node = SeekHarness()
    PaperSeekServoNode._record_wrench(node, 0.4, 10.05)

    assert node._state == BASELINING
    assert node._state_started == 10.05
    assert node._baseline_samples == [0.4]

    PaperSeekServoNode._record_wrench(node, 0.4, 11.04)
    monkeypatch.setattr(paper_seek_module.time, "monotonic", lambda: 11.049)
    PaperSeekServoNode._tick(node)
    assert node._state == BASELINING

    PaperSeekServoNode._record_wrench(node, 0.4, 11.05)
    monkeypatch.setattr(paper_seek_module.time, "monotonic", lambda: 11.051)
    PaperSeekServoNode._tick(node)
    assert node._state == DESCENDING


def test_status_logger_can_change_from_info_to_error():
    context = Context()
    rclpy.init(context=context)
    node = Node("paper_seek_status_logger_test", context=context)
    node._status_pub = StatusPublisher()
    try:
        node._state = IDLE
        PaperSeekServoNode._publish_status(node, "ready")
        node._state = ABORTED
        PaperSeekServoNode._publish_status(node, "safe stop", error=True)
        assert node._status_pub.messages == [
            "IDLE: ready",
            "ABORTED: safe stop",
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)
