import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from moveit_msgs.msg import ServoStatus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


@dataclass(frozen=True)
class BenchmarkPhase:
    label: str
    target_x: float
    target_y: float
    duration_sec: float
    scored: bool = False


@dataclass(frozen=True)
class MetricThresholds:
    position_avg_m: float = 0.005
    position_max_m: float = 0.010
    z_axis_avg_deg: float = 5.0
    z_axis_max_deg: float = 8.0
    quaternion_z_axis_gap_max_deg: float = 2.0
    final_position_m: float = 0.005
    final_z_axis_deg: float = 3.0


class BenchmarkInfrastructureError(RuntimeError):
    pass


EIGHT_DIRECTION_PROFILE = "eight_direction"
LONG_MINUS_Y_PLUS_XY_PROFILE = "long_minus_y_plus_xy"
LONG_PLUS_X_PROFILE = "long_plus_x"


def joy_message_for_target(
    target_x: float,
    target_y: float,
    *,
    emergency_stop: bool = False,
    quit_requested: bool = False,
) -> Joy:
    msg = Joy()
    msg.axes = [-target_y, -target_x]
    msg.buttons = [
        1 if emergency_stop else 0,
        1 if quit_requested else 0,
    ]
    return msg


def benchmark_phases(profile: str = EIGHT_DIRECTION_PROFILE) -> list[BenchmarkPhase]:
    if profile == EIGHT_DIRECTION_PROFILE:
        return [
            BenchmarkPhase("initial_settle", 0.0, 0.0, 2.0),
            BenchmarkPhase("plus_x", 1.0, 0.0, 1.5, scored=True),
            BenchmarkPhase("minus_x", -1.0, 0.0, 1.5, scored=True),
            BenchmarkPhase("plus_y", 0.0, 1.0, 1.5, scored=True),
            BenchmarkPhase("minus_y", 0.0, -1.0, 1.5, scored=True),
            BenchmarkPhase("plus_xy", 1.0, 1.0, 1.5, scored=True),
            BenchmarkPhase("minus_xy", -1.0, -1.0, 1.5, scored=True),
            BenchmarkPhase("plus_x_minus_y", 1.0, -1.0, 1.5, scored=True),
            BenchmarkPhase("minus_x_plus_y", -1.0, 1.0, 1.5, scored=True),
            BenchmarkPhase("final_settle", 0.0, 0.0, 5.0),
        ]
    if profile == LONG_MINUS_Y_PLUS_XY_PROFILE:
        return [
            BenchmarkPhase("initial_settle", 0.0, 0.0, 2.0),
            BenchmarkPhase("minus_y", 0.0, -1.0, 5.0, scored=True),
            BenchmarkPhase("inter_settle", 0.0, 0.0, 2.0),
            BenchmarkPhase("plus_xy", 1.0, 1.0, 5.0, scored=True),
            BenchmarkPhase("final_settle", 0.0, 0.0, 5.0),
        ]
    if profile == LONG_PLUS_X_PROFILE:
        return [
            BenchmarkPhase("initial_settle", 0.0, 0.0, 2.0),
            BenchmarkPhase("plus_x", 1.0, 0.0, 5.0, scored=True),
            BenchmarkPhase("final_settle", 0.0, 0.0, 5.0),
        ]
    raise ValueError(f"unknown benchmark profile: {profile}")


def scored_phase_windows(
    phases: Iterable[BenchmarkPhase],
) -> dict[str, tuple[float, float]]:
    elapsed = 0.0
    windows: dict[str, tuple[float, float]] = {}
    for phase in phases:
        start = elapsed
        elapsed += phase.duration_sec
        if phase.scored:
            windows[phase.label] = (start, elapsed)
    return windows


def total_phase_duration(phases: Iterable[BenchmarkPhase]) -> float:
    return sum(phase.duration_sec for phase in phases)


def analyze_alignment_csv(
    csv_path: Path,
    *,
    phases: Iterable[BenchmarkPhase] | None = None,
    thresholds: MetricThresholds | None = None,
    score_start_sec: float = 0.0,
) -> dict:
    phases = list(benchmark_phases() if phases is None else phases)
    thresholds = MetricThresholds() if thresholds is None else thresholds
    rows = _read_alignment_rows(csv_path)
    if not rows:
        raise BenchmarkInfrastructureError(f"no rows found in {csv_path}")

    total_sequence_sec = total_phase_duration(phases)
    score_end_sec = score_start_sec + total_sequence_sec
    initial_exclusion_sec = score_start_sec + (
        phases[0].duration_sec if phases else 0.0
    )
    post_initial_rows = [
        row
        for row in rows
        if initial_exclusion_sec <= row["elapsed_sec"] <= score_end_sec
    ]
    if not post_initial_rows:
        raise BenchmarkInfrastructureError(
            "no samples found after the initial alignment exclusion window"
        )

    windows = scored_phase_windows(phases)
    phase_summaries = {
        label: _summarize_phase_rows(
            rows,
            score_start_sec=score_start_sec,
            start_sec=start_sec,
            end_sec=end_sec,
        )
        for label, (start_sec, end_sec) in windows.items()
    }
    overall = _summarize_rows(post_initial_rows)
    final_row = _last_finite_row(post_initial_rows)
    if final_row is None:
        raise BenchmarkInfrastructureError("no finite final sample found")

    checks = _evaluate_checks(
        overall=overall,
        final_row=final_row,
        rows=post_initial_rows,
        thresholds=thresholds,
    )
    status = "PASS" if all(item["passed"] for item in checks) else "FAIL"
    return {
        "status": status,
        "csv_path": str(csv_path),
        "sample_count": len(rows),
        "post_initial_sample_count": len(post_initial_rows),
        "score_start_sec": score_start_sec,
        "score_end_sec": score_end_sec,
        "initial_exclusion_sec": initial_exclusion_sec,
        "total_sequence_sec": total_sequence_sec,
        "overall": overall,
        "phases": phase_summaries,
        "final": {
            "elapsed_sec": final_row["elapsed_sec"],
            "position_m": final_row["position_m"],
            "z_axis_deg": final_row["z_axis_deg"],
            "full_quaternion_deg": final_row["full_quaternion_deg"],
        },
        "checks": checks,
    }


def write_summary_files(
    *,
    result: dict,
    json_path: Path,
    markdown_path: Path,
) -> None:
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(_format_markdown_summary(result), encoding="utf-8")


def _read_alignment_rows(csv_path: Path) -> list[dict[str, float]]:
    if not csv_path.exists():
        raise BenchmarkInfrastructureError(f"CSV file does not exist: {csv_path}")

    required_columns = {
        "elapsed_sec",
        "position_m",
        "z_axis_deg",
        "full_quaternion_deg",
    }
    rows: list[dict[str, float]] = []
    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise BenchmarkInfrastructureError("CSV file has no header")
        missing = required_columns.difference(reader.fieldnames)
        if missing:
            raise BenchmarkInfrastructureError(
                "CSV file is missing required columns: " + ", ".join(sorted(missing))
            )

        for raw in reader:
            row = {
                "elapsed_sec": float(raw["elapsed_sec"]),
                "position_m": float(raw["position_m"]),
                "z_axis_deg": float(raw["z_axis_deg"]),
                "full_quaternion_deg": float(raw["full_quaternion_deg"]),
            }
            for column in (
                "pose_command_armed",
                "pose_command_published",
                "has_motion_intent",
                "virtual_pen_settling",
            ):
                if column in raw and raw[column] != "":
                    row[column] = float(raw[column])
            rows.append(row)
    return rows


def latest_alignment_row(csv_path: Path) -> dict[str, float] | None:
    rows = _read_alignment_rows(csv_path)
    return _last_finite_row(rows)


def _summarize_rows(rows: list[dict[str, float]]) -> dict:
    finite_rows = [row for row in rows if _row_is_finite(row)]
    if not finite_rows:
        return {
            "sample_count": len(rows),
            "finite_sample_count": 0,
            "position_m": _empty_stats(),
            "z_axis_deg": _empty_stats(),
            "full_quaternion_deg": _empty_stats(),
            "quaternion_z_axis_gap_deg": _empty_stats(),
        }

    gaps = [
        abs(row["full_quaternion_deg"] - row["z_axis_deg"])
        for row in finite_rows
    ]
    return {
        "sample_count": len(rows),
        "finite_sample_count": len(finite_rows),
        "position_m": _stats([row["position_m"] for row in finite_rows]),
        "z_axis_deg": _stats([row["z_axis_deg"] for row in finite_rows]),
        "full_quaternion_deg": _stats(
            [row["full_quaternion_deg"] for row in finite_rows]
        ),
        "quaternion_z_axis_gap_deg": _stats(gaps),
    }


def _summarize_phase_rows(
    rows: list[dict[str, float]],
    *,
    score_start_sec: float,
    start_sec: float,
    end_sec: float,
) -> dict:
    absolute_start = score_start_sec + start_sec
    absolute_end = score_start_sec + end_sec
    phase_rows = [
        row
        for row in rows
        if absolute_start <= row["elapsed_sec"] < absolute_end
    ]
    summary = _summarize_rows(phase_rows)
    if end_sec - start_sec < 2.0:
        return summary

    first = _summarize_rows(
        [
            row
            for row in rows
            if absolute_start <= row["elapsed_sec"] < absolute_start + 1.0
        ]
    )
    last = _summarize_rows(
        [
            row
            for row in rows
            if absolute_end - 1.0 <= row["elapsed_sec"] < absolute_end
        ]
    )
    summary["convergence_1s"] = {
        "first": first,
        "last": last,
        "position_avg_delta_m": (
            last["position_m"]["avg"] - first["position_m"]["avg"]
        ),
        "position_max_delta_m": (
            last["position_m"]["max"] - first["position_m"]["max"]
        ),
    }
    return summary


def _empty_stats() -> dict[str, float]:
    return {"avg": math.nan, "max": math.nan, "min": math.nan}


def _stats(values: list[float]) -> dict[str, float]:
    return {
        "avg": sum(values) / len(values),
        "max": max(values),
        "min": min(values),
    }


def _row_is_finite(row: dict[str, float]) -> bool:
    return (
        math.isfinite(row["position_m"])
        and math.isfinite(row["z_axis_deg"])
        and math.isfinite(row["full_quaternion_deg"])
    )


def _last_finite_row(rows: list[dict[str, float]]) -> dict[str, float] | None:
    for row in reversed(rows):
        if _row_is_finite(row):
            return row
    return None


def _evaluate_checks(
    *,
    overall: dict,
    final_row: dict[str, float],
    rows: list[dict[str, float]],
    thresholds: MetricThresholds,
) -> list[dict]:
    finite_count = sum(1 for row in rows if _row_is_finite(row))
    checks = [
        _check(
            "no_nan_after_initial",
            finite_count == len(rows),
            finite_count,
            len(rows),
            "finite samples / total samples",
        ),
        _check(
            "position_avg",
            overall["position_m"]["avg"] <= thresholds.position_avg_m,
            overall["position_m"]["avg"],
            thresholds.position_avg_m,
            "m",
        ),
        _check(
            "position_max",
            overall["position_m"]["max"] <= thresholds.position_max_m,
            overall["position_m"]["max"],
            thresholds.position_max_m,
            "m",
        ),
        _check(
            "z_axis_avg",
            overall["z_axis_deg"]["avg"] <= thresholds.z_axis_avg_deg,
            overall["z_axis_deg"]["avg"],
            thresholds.z_axis_avg_deg,
            "deg",
        ),
        _check(
            "z_axis_max",
            overall["z_axis_deg"]["max"] <= thresholds.z_axis_max_deg,
            overall["z_axis_deg"]["max"],
            thresholds.z_axis_max_deg,
            "deg",
        ),
        _check(
            "quaternion_z_axis_gap_max",
            overall["quaternion_z_axis_gap_deg"]["max"]
            <= thresholds.quaternion_z_axis_gap_max_deg,
            overall["quaternion_z_axis_gap_deg"]["max"],
            thresholds.quaternion_z_axis_gap_max_deg,
            "deg",
        ),
        _check(
            "final_position",
            final_row["position_m"] <= thresholds.final_position_m,
            final_row["position_m"],
            thresholds.final_position_m,
            "m",
        ),
        _check(
            "final_z_axis",
            final_row["z_axis_deg"] <= thresholds.final_z_axis_deg,
            final_row["z_axis_deg"],
            thresholds.final_z_axis_deg,
            "deg",
        ),
    ]
    return checks


def _check(
    name: str,
    passed: bool,
    value,
    threshold,
    unit: str,
) -> dict:
    return {
        "name": name,
        "passed": bool(passed),
        "value": value,
        "threshold": threshold,
        "unit": unit,
    }


def _format_markdown_summary(result: dict) -> str:
    lines = [
        "# Pen Writing Stage2 Tracking Benchmark",
        "",
        f"Status: **{result['status']}**",
        "",
        f"CSV: `{result['csv_path']}`",
        f"Samples: {result['post_initial_sample_count']} post-initial / "
        f"{result['sample_count']} total",
        f"Score start: {result.get('score_start_sec', 0.0):.3f} s",
        f"Score end: {result.get('score_end_sec', 0.0):.3f} s",
        f"Initial exclusion: {result['initial_exclusion_sec']:.3f} s",
        "",
        "## Overall",
        "",
        "| Metric | Avg | Max |",
        "| --- | ---: | ---: |",
    ]
    overall = result["overall"]
    for metric in ("position_m", "z_axis_deg", "full_quaternion_deg"):
        lines.append(
            f"| {metric} | {_fmt(overall[metric]['avg'])} | "
            f"{_fmt(overall[metric]['max'])} |"
        )
    lines.extend(
        [
            "",
            "## Direction Phases",
            "",
            "| Phase | Samples | Position avg/max m | Z avg/max deg | Quat avg/max deg |",
            "| --- | ---: | ---: | ---: | ---: |",
        ]
    )
    for label, summary in result["phases"].items():
        lines.append(
            f"| {label} | {summary['finite_sample_count']} | "
            f"{_fmt(summary['position_m']['avg'])} / "
            f"{_fmt(summary['position_m']['max'])} | "
            f"{_fmt(summary['z_axis_deg']['avg'])} / "
            f"{_fmt(summary['z_axis_deg']['max'])} | "
            f"{_fmt(summary['full_quaternion_deg']['avg'])} / "
            f"{_fmt(summary['full_quaternion_deg']['max'])} |"
        )
    convergence = {
        label: summary["convergence_1s"]
        for label, summary in result["phases"].items()
        if "convergence_1s" in summary
    }
    if convergence:
        lines.extend(
            [
                "",
                "## Phase Convergence",
                "",
                "| Phase | First 1s pos avg/max m | Last 1s pos avg/max m | Avg delta m |",
                "| --- | ---: | ---: | ---: |",
            ]
        )
        for label, item in convergence.items():
            first = item["first"]["position_m"]
            last = item["last"]["position_m"]
            lines.append(
                f"| {label} | {_fmt(first['avg'])} / {_fmt(first['max'])} | "
                f"{_fmt(last['avg'])} / {_fmt(last['max'])} | "
                f"{_fmt(item['position_avg_delta_m'])} |"
            )
    lines.extend(["", "## Checks", "", "| Check | Result | Value | Threshold |"])
    lines.append("| --- | --- | ---: | ---: |")
    for check in result["checks"]:
        status = "PASS" if check["passed"] else "FAIL"
        lines.append(
            f"| {check['name']} | {status} | {_fmt(check['value'])} "
            f"{check['unit']} | {_fmt(check['threshold'])} {check['unit']} |"
        )
    lines.append("")
    return "\n".join(lines)


def _fmt(value) -> str:
    if isinstance(value, int):
        return str(value)
    if not isinstance(value, float):
        return str(value)
    if math.isnan(value):
        return "nan"
    return f"{value:.6f}"


class PenTrackingBenchmarkNode(Node):
    def __init__(self) -> None:
        super().__init__("pen_tracking_benchmark")

        self.joy_topic = str(
            self.declare_parameter("joy_topic", "/pen_writing/benchmark/joy").value
        )
        self.servo_status_topic = str(
            self.declare_parameter("servo_status_topic", "/servo_node/status").value
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
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 50.0).value
        )
        self.ready_timeout_sec = float(
            self.declare_parameter("ready_timeout_sec", 30.0).value
        )
        self.arm_timeout_sec = float(
            self.declare_parameter("arm_timeout_sec", 10.0).value
        )

        self._status_seen = False
        self._joy_publisher = self.create_publisher(Joy, self.joy_topic, 10)
        self._servo_status_subscription = self.create_subscription(
            ServoStatus,
            self.servo_status_topic,
            self._on_servo_status,
            10,
        )

    def run(self) -> int:
        self.get_logger().info(
            "Starting pen tracking benchmark: "
            f"joy_topic={self.joy_topic} csv={self.alignment_error_log_path}"
        )
        self._wait_until_ready()
        self._publish_until_csv_starts()
        self._run_phases(benchmark_phases())
        result = analyze_alignment_csv(self.alignment_error_log_path)
        write_summary_files(
            result=result,
            json_path=self.summary_json_path,
            markdown_path=self.summary_markdown_path,
        )
        self.get_logger().info(
            "Tracking benchmark finished with "
            f"{result['status']}. Summary: {self.summary_markdown_path}"
        )
        self._publish_quit()
        return 0

    def _on_servo_status(self, _msg: ServoStatus) -> None:
        self._status_seen = True

    def _wait_until_ready(self) -> None:
        deadline = time.monotonic() + self.ready_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._status_seen and self._joy_publisher.get_subscription_count() > 0:
                return
        raise BenchmarkInfrastructureError(
            "timed out waiting for Servo status and Joy subscribers"
        )

    def _publish_until_csv_starts(self) -> None:
        period_sec = 1.0 / self.publish_rate_hz
        deadline = time.monotonic() + self.arm_timeout_sec
        arm_message = joy_message_for_target(1.0, 0.0)
        while rclpy.ok() and time.monotonic() < deadline:
            self._joy_publisher.publish(arm_message)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.alignment_error_log_path.exists():
                self.get_logger().info(
                    "POSE command arm detected; alignment CSV has started."
                )
                return
            time.sleep(period_sec)
        raise BenchmarkInfrastructureError("timed out waiting for alignment CSV start")

    def _run_phases(self, phases: list[BenchmarkPhase]) -> None:
        period_sec = 1.0 / self.publish_rate_hz
        for phase in phases:
            message = joy_message_for_target(phase.target_x, phase.target_y)
            end_time = time.monotonic() + phase.duration_sec
            while rclpy.ok() and time.monotonic() < end_time:
                self._joy_publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period_sec)

    def _publish_quit(self) -> None:
        quit_message = joy_message_for_target(0.0, 0.0, quit_requested=True)
        period_sec = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + 0.3
        while rclpy.ok() and time.monotonic() < end_time:
            self._joy_publisher.publish(quit_message)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_sec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PenTrackingBenchmarkNode()
    exit_code = 0
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
        exit_code = 130
    except BenchmarkInfrastructureError as exc:
        node.get_logger().error(f"Tracking benchmark infrastructure failed: {exc}")
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
