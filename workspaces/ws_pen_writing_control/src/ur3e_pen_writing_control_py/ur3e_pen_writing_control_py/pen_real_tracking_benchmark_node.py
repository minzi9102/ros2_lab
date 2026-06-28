import json
import sys
import time
from pathlib import Path

from moveit_msgs.msg import ServoStatus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode

from .pen_tracking_benchmark_node import (
    BenchmarkInfrastructureError,
    analyze_alignment_csv,
    benchmark_phases,
    joy_message_for_target,
    latest_alignment_row,
    write_summary_files,
)


NORMAL_OUTCOMES = {"completed", "performance_fail", "operator_b"}
ALIGNMENT_POSITION_READY_M = 0.005
ALIGNMENT_Z_AXIS_READY_DEG = 3.0
ALIGNMENT_READY_HOLD_SEC = 0.5


def should_return_home(outcome: str) -> bool:
    return outcome in NORMAL_OUTCOMES


def alignment_row_ready(
    row: dict[str, float],
    *,
    position_ready_m: float = ALIGNMENT_POSITION_READY_M,
    z_axis_ready_deg: float = ALIGNMENT_Z_AXIS_READY_DEG,
) -> bool:
    return (
        row["position_m"] <= position_ready_m
        and row["z_axis_deg"] <= z_axis_ready_deg
        and row.get("virtual_pen_settling", 0.0) == 0.0
    )


class BenchmarkSafetyAbort(RuntimeError):
    pass


class OperatorControlledStop(RuntimeError):
    pass


class PenRealTrackingBenchmarkNode(Node):
    def __init__(self) -> None:
        super().__init__("pen_real_tracking_benchmark")
        self.command_joy_topic = str(
            self.declare_parameter(
                "command_joy_topic",
                "/pen_writing/real_benchmark/joy",
            ).value
        )
        self.operator_joy_topic = str(
            self.declare_parameter("operator_joy_topic", "/joy").value
        )
        self.servo_status_topic = str(
            self.declare_parameter(
                "servo_status_topic",
                "/servo_node/status",
            ).value
        )
        self.alignment_error_log_path = Path(
            str(self.declare_parameter("alignment_error_log_path", "").value)
        )
        self.summary_json_path = Path(
            str(self.declare_parameter("summary_json_path", "").value)
        )
        self.summary_markdown_path = Path(
            str(self.declare_parameter("summary_markdown_path", "").value)
        )
        self.result_path = Path(
            str(self.declare_parameter("result_path", "").value)
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 50.0).value
        )
        self.ready_timeout_sec = float(
            self.declare_parameter("ready_timeout_sec", 30.0).value
        )
        self.arm_timeout_sec = float(
            self.declare_parameter("arm_timeout_sec", 10.0).value
        )
        self.alignment_ready_timeout_sec = float(
            self.declare_parameter("alignment_ready_timeout_sec", 30.0).value
        )

        self._status_seen = False
        self._safety_abort_reason: str | None = None
        self._controlled_stop_requested = False
        self._dashboard_request_pending = False
        self._joy_publisher = self.create_publisher(
            Joy,
            self.command_joy_topic,
            10,
        )
        self.create_subscription(
            ServoStatus,
            self.servo_status_topic,
            self._on_servo_status,
            10,
        )
        self.create_subscription(
            Joy,
            self.operator_joy_topic,
            self._on_operator_joy,
            10,
        )
        self._pause_client = self.create_client(
            SetBool,
            "/servo_node/pause_servo",
        )
        self._robot_mode_client = self.create_client(
            GetRobotMode,
            "/dashboard_client/get_robot_mode",
        )
        self._safety_mode_client = self.create_client(
            GetSafetyMode,
            "/dashboard_client/get_safety_mode",
        )
        self.create_timer(0.5, self._poll_dashboard_state)

    def run(self) -> int:
        outcome = "infrastructure_error"
        reason = ""
        try:
            self._wait_until_ready()
            self._publish_until_csv_starts()
            score_start_sec = self._wait_until_alignment_ready()
            self._run_phases()
            result = analyze_alignment_csv(
                self.alignment_error_log_path,
                score_start_sec=score_start_sec,
            )
            write_summary_files(
                result=result,
                json_path=self.summary_json_path,
                markdown_path=self.summary_markdown_path,
            )
            outcome = "completed" if result["status"] == "PASS" else "performance_fail"
            reason = f"benchmark completed with {result['status']}"
            self._freeze_and_pause()
        except OperatorControlledStop:
            outcome = "operator_b"
            reason = "operator B requested controlled stop"
            self._write_controlled_stop_summary()
            try:
                self._freeze_and_pause()
            except BenchmarkInfrastructureError as exc:
                outcome = "infrastructure_error"
                reason = str(exc)
        except BenchmarkSafetyAbort as exc:
            outcome = "safety_abort"
            reason = str(exc)
            self._publish_freeze()
        except BenchmarkInfrastructureError as exc:
            outcome = "infrastructure_error"
            reason = str(exc)
            self._publish_freeze()

        self._write_result(outcome, reason)
        log = self.get_logger().info if should_return_home(outcome) else self.get_logger().error
        log(f"Real benchmark outcome={outcome} reason={reason}")
        return 0 if should_return_home(outcome) else 2

    def _on_servo_status(self, msg: ServoStatus) -> None:
        self._status_seen = True
        if msg.code != ServoStatus.NO_WARNING:
            self._safety_abort_reason = (
                f"Servo status fault: code={msg.code} message={msg.message!r}"
            )

    def _on_operator_joy(self, msg: Joy) -> None:
        a_pressed = len(msg.buttons) > 0 and bool(msg.buttons[0])
        b_pressed = len(msg.buttons) > 1 and bool(msg.buttons[1])
        if a_pressed:
            self._safety_abort_reason = "operator A freeze requested"
        elif b_pressed:
            self._controlled_stop_requested = True

    def _poll_dashboard_state(self) -> None:
        if self._dashboard_request_pending:
            return
        robot_mode_ready = self._robot_mode_client.service_is_ready()
        safety_mode_ready = self._safety_mode_client.service_is_ready()
        if not (robot_mode_ready and safety_mode_ready):
            return
        self._dashboard_request_pending = True
        robot_future = self._robot_mode_client.call_async(GetRobotMode.Request())
        safety_future = self._safety_mode_client.call_async(GetSafetyMode.Request())

        def finish_poll(_future) -> None:
            if not robot_future.done() or not safety_future.done():
                return
            self._dashboard_request_pending = False
            try:
                robot_response = robot_future.result()
                safety_response = safety_future.result()
            except Exception as exc:  # noqa: BLE001 - service transport failure is a safety abort
                self._safety_abort_reason = f"dashboard state query failed: {exc}"
                return
            if robot_response is None or safety_response is None:
                self._safety_abort_reason = "dashboard state query failed"
                return
            if robot_response.robot_mode.mode != robot_response.robot_mode.RUNNING:
                self._safety_abort_reason = (
                    f"robot mode left RUNNING: {robot_response.robot_mode.mode}"
                )
            if safety_response.safety_mode.mode != safety_response.safety_mode.NORMAL:
                self._safety_abort_reason = (
                    f"safety mode left NORMAL: {safety_response.safety_mode.mode}"
                )

        robot_future.add_done_callback(finish_poll)
        safety_future.add_done_callback(finish_poll)

    def _check_stop_requests(self) -> None:
        if self._safety_abort_reason is not None:
            raise BenchmarkSafetyAbort(self._safety_abort_reason)
        if self._controlled_stop_requested:
            raise OperatorControlledStop()

    def _wait_until_ready(self) -> None:
        deadline = time.monotonic() + self.ready_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            self._check_stop_requests()
            if self._status_seen and self._joy_publisher.get_subscription_count() > 0:
                return
        raise BenchmarkInfrastructureError(
            "timed out waiting for Servo status and command Joy subscriber"
        )

    def _publish_until_csv_starts(self) -> None:
        deadline = time.monotonic() + self.arm_timeout_sec
        arm_message = joy_message_for_target(1.0, 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            self._joy_publisher.publish(arm_message)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._check_stop_requests()
            if self.alignment_error_log_path.exists():
                return
            time.sleep(1.0 / self.publish_rate_hz)
        raise BenchmarkInfrastructureError("timed out waiting for alignment CSV start")

    def _wait_until_alignment_ready(self) -> float:
        period_sec = 1.0 / self.publish_rate_hz
        deadline = time.monotonic() + self.alignment_ready_timeout_sec
        hold_started_at_sec = None
        hold_started_at_elapsed = None
        settle_message = joy_message_for_target(0.0, 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            self._joy_publisher.publish(settle_message)
            rclpy.spin_once(self, timeout_sec=0.0)
            self._check_stop_requests()
            row = latest_alignment_row(self.alignment_error_log_path)
            if row is not None and alignment_row_ready(row):
                now_sec = time.monotonic()
                if hold_started_at_sec is None:
                    hold_started_at_sec = now_sec
                    hold_started_at_elapsed = row["elapsed_sec"]
                if now_sec - hold_started_at_sec >= ALIGNMENT_READY_HOLD_SEC:
                    assert hold_started_at_elapsed is not None
                    self.get_logger().info(
                        "Alignment ready; tracking score starts at "
                        f"{hold_started_at_elapsed:.3f}s in CSV."
                    )
                    return hold_started_at_elapsed
            else:
                hold_started_at_sec = None
                hold_started_at_elapsed = None
            time.sleep(period_sec)
        raise BenchmarkInfrastructureError("timed out waiting for alignment ready")

    def _run_phases(self) -> None:
        for phase in benchmark_phases():
            message = joy_message_for_target(phase.target_x, phase.target_y)
            deadline = time.monotonic() + phase.duration_sec
            while rclpy.ok() and time.monotonic() < deadline:
                self._joy_publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.0)
                self._check_stop_requests()
                time.sleep(1.0 / self.publish_rate_hz)

    def _publish_freeze(self) -> None:
        freeze_message = joy_message_for_target(
            0.0,
            0.0,
            emergency_stop=True,
        )
        deadline = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < deadline:
            self._joy_publisher.publish(freeze_message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / self.publish_rate_hz)

    def _freeze_and_pause(self) -> None:
        self._publish_freeze()
        if not self._pause_client.wait_for_service(timeout_sec=2.0):
            raise BenchmarkInfrastructureError("Servo pause service unavailable")
        request = SetBool.Request()
        request.data = True
        future = self._pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            raise BenchmarkInfrastructureError("Servo pause request failed")

    def _write_controlled_stop_summary(self) -> None:
        result = {
            "status": "OPERATOR_B",
            "csv_path": str(self.alignment_error_log_path),
            "message": "Benchmark stopped by operator B before all phases completed.",
        }
        self.summary_json_path.write_text(
            json.dumps(result, indent=2) + "\n",
            encoding="utf-8",
        )
        self.summary_markdown_path.write_text(
            "# Pen Writing Stage3 Real Tracking Benchmark\n\n"
            "Status: **OPERATOR_B**\n",
            encoding="utf-8",
        )

    def _write_result(self, outcome: str, reason: str) -> None:
        content = json.dumps(
            {
                "outcome": outcome,
                "reason": reason,
                "return_home": should_return_home(outcome),
            },
            indent=2,
        )
        content += "\n"
        self.result_path.write_text(
            content,
            encoding="utf-8",
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PenRealTrackingBenchmarkNode()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Ctrl-C received; return-home path is disabled.")
        node._publish_freeze()
        node._write_result("keyboard_interrupt", "Ctrl-C")
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
