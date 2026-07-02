import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Point, PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .pose_math import Point3, PoseTarget, Quaternion, transform_point


JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
DEFAULT_PREHOME = (
    -3.4876160003,
    -0.8786259996,
    -2.2516570001,
    -1.5821070003,
    1.570796,
    -1.9168200003,
)
MOTION_CONTROLLERS = {
    "scaled_joint_trajectory_controller",
    "joint_trajectory_controller",
    "forward_velocity_controller",
    "forward_position_controller",
    "forward_effort_controller",
    "force_mode_controller",
    "passthrough_trajectory_controller",
    "freedrive_mode_controller",
    "tool_contact_controller",
}


@dataclass(frozen=True)
class PhaseTimes:
    initial_settle: float = 2.0
    motion: float = 5.0
    final_settle: float = 5.0
    steady_start: float = 0.5
    steady_end: float = 4.5

    @property
    def motion_start(self) -> float:
        return self.initial_settle

    @property
    def motion_end(self) -> float:
        return self.initial_settle + self.motion

    @property
    def total(self) -> float:
        return self.motion_end + self.final_settle


@dataclass(frozen=True)
class Sample:
    elapsed: float
    phase: str
    target: Point3
    actual: Point3
    target_speed: float
    actual_speed: float
    joint_velocities: tuple[float, ...]


def prehome_script(
    joints=DEFAULT_PREHOME,
    *,
    acceleration: float = 0.3,
    velocity: float = 0.3,
) -> str:
    positions = ", ".join(f"{value:.10f}" for value in joints)
    return (
        "def speedl_benchmark_prehome():\n"
        f"  movej([{positions}], a={acceleration:.6f}, v={velocity:.6f})\n"
        "end\n"
    )


def speedl_script(
    *,
    speed: float = 0.06,
    acceleration: float = 0.2,
    duration: float = 5.0,
) -> str:
    return (
        "def speedl_benchmark_motion():\n"
        f"  speedl([{speed:.6f}, 0, 0, 0, 0, 0], "
        f"{acceleration:.6f}, {duration:.6f})\n"
        f"  stopl({acceleration:.6f})\n"
        "end\n"
    )


def stopl_script(acceleration: float = 0.2) -> str:
    return f"stopl({acceleration:.6f})\n"


def phase_at(elapsed: float, times: PhaseTimes) -> str:
    if elapsed < times.motion_start:
        return "initial_settle"
    if elapsed < times.motion_end:
        return "speedl_plus_x"
    return "final_settle"


def target_distance(elapsed: float, speed: float, times: PhaseTimes) -> float:
    motion_elapsed = min(max(elapsed - times.motion_start, 0.0), times.motion)
    return speed * motion_elapsed


def angular_error(left: float, right: float) -> float:
    return abs(math.atan2(math.sin(left - right), math.cos(left - right)))


def point_distance(left: Point3, right: Point3) -> float:
    return math.sqrt(
        (left.x - right.x) ** 2
        + (left.y - right.y) ** 2
        + (left.z - right.z) ** 2
    )


def _window_metrics(samples: list[Sample], start: float, end: float) -> dict:
    selected = [sample for sample in samples if start <= sample.elapsed <= end]
    if len(selected) < 2:
        return {}
    target_path = sum(
        point_distance(left.target, right.target)
        for left, right in zip(selected, selected[1:])
    )
    actual_path = sum(
        point_distance(left.actual, right.actual)
        for left, right in zip(selected, selected[1:])
    )
    errors = [
        point_distance(sample.target, sample.actual) for sample in selected
    ]
    duration = selected[-1].elapsed - selected[0].elapsed
    return {
        "sample_count": len(selected),
        "target_mean_speed_mps": target_path / duration,
        "target_peak_speed_mps": max(
            sample.target_speed for sample in selected
        ),
        "actual_mean_speed_mps": actual_path / duration,
        "actual_peak_speed_mps": max(
            sample.actual_speed for sample in selected
        ),
        "target_path_length_m": target_path,
        "actual_path_length_m": actual_path,
        "path_ratio": actual_path / target_path if target_path > 0.0 else None,
        "position_rms_m": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
    }


def summarize_samples(
    samples: list[Sample],
    *,
    times: PhaseTimes,
    joint_names=JOINT_NAMES,
) -> dict:
    full = _window_metrics(samples, times.motion_start, times.motion_end)
    steady = _window_metrics(
        samples,
        times.motion_start + times.steady_start,
        times.motion_start + times.steady_end,
    )
    joint_max = {
        name: max(
            (abs(sample.joint_velocities[index]) for sample in samples),
            default=0.0,
        )
        for index, name in enumerate(joint_names)
    }
    mean_speed = steady.get("actual_mean_speed_mps", 0.0)
    ratio = steady.get("path_ratio")
    if ratio is not None and 0.057 <= mean_speed <= 0.063 and 0.95 <= ratio <= 1.05:
        verdict = "downstream_normal"
    elif mean_speed < 0.05:
        verdict = "downstream_speed_insufficient"
    else:
        verdict = "inconclusive"
    return {
        "verdict": verdict,
        "full_motion_window": full,
        "steady_motion_window": steady,
        "joint_max_abs_velocity_radps": joint_max,
    }


class SpeedLBenchmarkNode(Node):
    def __init__(self) -> None:
        super().__init__("stage2_ursim_speedl_benchmark")
        self.speed = float(self.declare_parameter("speed_mps", 0.06).value)
        self.acceleration = float(
            self.declare_parameter("acceleration_mps2", 0.2).value
        )
        self.times = PhaseTimes(
            initial_settle=float(
                self.declare_parameter("initial_settle_sec", 2.0).value
            ),
            motion=float(self.declare_parameter("motion_sec", 5.0).value),
            final_settle=float(
                self.declare_parameter("final_settle_sec", 5.0).value
            ),
            steady_start=float(
                self.declare_parameter("steady_start_sec", 0.5).value
            ),
            steady_end=float(
                self.declare_parameter("steady_end_sec", 4.5).value
            ),
        )
        self.publish_rate = float(
            self.declare_parameter("publish_rate_hz", 100.0).value
        )
        self.paper_width = float(
            self.declare_parameter("paper_width_m", 0.60).value
        )
        self.paper_height = float(
            self.declare_parameter("paper_height_m", 0.16).value
        )
        self.initial_tip_x = float(
            self.declare_parameter("initial_tip_x_m", -0.24).value
        )
        self.base_frame = str(
            self.declare_parameter("base_frame", "base").value
        )
        self.tool_frame = str(
            self.declare_parameter("tool_frame", "tool0").value
        )
        offset = self.declare_parameter(
            "tool0_to_pen_tip_xyz", [0.0, 0.0, 0.14]
        ).value
        self.tool_to_tip = Point3(*(float(value) for value in offset))
        self.prehome = tuple(
            float(value)
            for value in self.declare_parameter(
                "prehome_joint_positions", list(DEFAULT_PREHOME)
            ).value
        )
        self.prehome_tolerance = float(
            self.declare_parameter("prehome_tolerance_rad", 0.02).value
        )
        self.prehome_timeout = float(
            self.declare_parameter("prehome_timeout_sec", 30.0).value
        )
        self.output_dir = Path(
            str(self.declare_parameter("output_dir", ".").value)
        )
        self._validate_parameters()

        self.script_pub = self.create_publisher(
            String, "/urscript_interface/script_command", 10
        )
        self.target_pub = self.create_publisher(
            PoseStamped, "/pen_writing/target_pose", 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/pen_writing/markers", 10
        )
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = "wait_services"
        self.state_started = self.get_clock().now()
        self.list_future = None
        self.latest_joints: dict[str, float] = {}
        self.latest_joint_velocity: dict[str, float] = {}
        self.previous_joint_positions: dict[str, float] = {}
        self.previous_joint_time = None
        self.prehome_sent = False
        self.motion_sent = False
        self.stop_sent = False
        self.benchmark_started = None
        self.initial_tip = None
        self.initial_orientation = None
        self.paper_center = None
        self.previous_actual = None
        self.previous_sample_time = None
        self.samples: list[Sample] = []
        self.target_path: list[Point3] = []
        self.actual_path: list[Point3] = []
        self.exit_code = 1
        self.finished = False
        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)

    def _validate_parameters(self) -> None:
        if (
            self.speed <= 0.0
            or self.acceleration <= 0.0
            or self.publish_rate <= 0.0
            or self.times.motion <= 0.0
            or not 0.0 <= self.times.steady_start < self.times.steady_end <= self.times.motion
            or len(self.prehome) != len(JOINT_NAMES)
        ):
            raise ValueError("invalid SpeedL benchmark parameters")

    def _on_joints(self, message: JointState) -> None:
        now = self.get_clock().now()
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if len(velocities) < len(positions) and self.previous_joint_time is not None:
            dt = (now - self.previous_joint_time).nanoseconds * 1e-9
            if dt > 0.0:
                velocities = {
                    name: (position - self.previous_joint_positions[name]) / dt
                    for name, position in positions.items()
                    if name in self.previous_joint_positions
                }
        self.latest_joints = positions
        self.latest_joint_velocity = velocities
        self.previous_joint_positions = positions
        self.previous_joint_time = now

    def _tick(self) -> None:
        if self.finished:
            return
        try:
            if self.state == "wait_services":
                self._wait_services()
            elif self.state == "prehome":
                self._wait_prehome()
            elif self.state == "benchmark":
                self._run_benchmark()
        except Exception as exc:
            self._fail(str(exc))

    def _wait_services(self) -> None:
        if not self.list_client.service_is_ready():
            if self._state_elapsed() > 20.0:
                raise RuntimeError("list_controllers service unavailable")
            return
        if self.list_future is None:
            self.list_future = self.list_client.call_async(ListControllers.Request())
            return
        if not self.list_future.done():
            return
        response = self.list_future.result()
        active_motion = [
            controller.name
            for controller in response.controller
            if controller.state == "active" and controller.name in MOTION_CONTROLLERS
        ]
        if active_motion:
            raise RuntimeError(f"active motion controllers: {active_motion}")
        if self.script_pub.get_subscription_count() < 1:
            if self._state_elapsed() > 20.0:
                raise RuntimeError("URScript interface has no subscriber")
            return
        self.state = "prehome"
        self.state_started = self.get_clock().now()
        self.get_logger().info("Controller isolation verified; sending pre-home")

    def _wait_prehome(self) -> None:
        if not self.prehome_sent:
            self._send_script(prehome_script(self.prehome))
            self.prehome_sent = True
            return
        if self._state_elapsed() > self.prehome_timeout:
            raise RuntimeError("pre-home joint convergence timed out")
        if not all(name in self.latest_joints for name in JOINT_NAMES):
            return
        maximum_error = max(
            angular_error(self.latest_joints[name], target)
            for name, target in zip(JOINT_NAMES, self.prehome)
        )
        if maximum_error >= self.prehome_tolerance:
            return
        pose = self._tool_pose()
        self.initial_tip = transform_point(pose, self.tool_to_tip)
        self.initial_orientation = pose.orientation
        self.paper_center = Point3(
            self.initial_tip.x - self.initial_tip_x,
            self.initial_tip.y,
            self.initial_tip.z - 0.002,
        )
        self.benchmark_started = self.get_clock().now()
        self.state = "benchmark"
        self.get_logger().info(
            f"Pre-home verified ({maximum_error:.6f} rad); benchmark started"
        )

    def _run_benchmark(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.benchmark_started).nanoseconds * 1e-9
        if elapsed >= self.times.motion_start and not self.motion_sent:
            self._send_script(
                speedl_script(
                    speed=self.speed,
                    acceleration=self.acceleration,
                    duration=self.times.motion,
                )
            )
            self.motion_sent = True
            self.get_logger().info("SpeedL +X command sent")
        if elapsed >= self.times.total:
            self._finish()
            return

        actual_pose = self._tool_pose()
        actual = transform_point(actual_pose, self.tool_to_tip)
        target = Point3(
            self.initial_tip.x + target_distance(elapsed, self.speed, self.times),
            self.initial_tip.y,
            self.initial_tip.z,
        )
        phase = phase_at(elapsed, self.times)
        target_speed = self.speed if phase == "speedl_plus_x" else 0.0
        actual_speed = 0.0
        if len(self.samples) >= 20:
            previous = self.samples[-20]
            dt = elapsed - previous.elapsed
            if dt > 0.0:
                actual_speed = point_distance(actual, previous.actual) / dt
        sample = Sample(
            elapsed=elapsed,
            phase=phase,
            target=target,
            actual=actual,
            target_speed=target_speed,
            actual_speed=actual_speed,
            joint_velocities=tuple(
                self.latest_joint_velocity.get(name, 0.0) for name in JOINT_NAMES
            ),
        )
        self.samples.append(sample)
        self.target_path.append(target)
        self.actual_path.append(actual)
        self.previous_actual = actual
        self.previous_sample_time = now
        self._publish_target(target, self.initial_orientation, now)
        self._publish_markers(sample, now)

    def _tool_pose(self) -> PoseTarget:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, Time()
            ).transform
        except TransformException as exc:
            raise RuntimeError(f"tool TF unavailable: {exc}") from exc
        return PoseTarget(
            position=Point3(
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            orientation=Quaternion(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def _publish_target(self, target: Point3, orientation: Quaternion, now) -> None:
        message = PoseStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = now.to_msg()
        message.pose.position.x = target.x
        message.pose.position.y = target.y
        message.pose.position.z = target.z
        message.pose.orientation.x = orientation.x
        message.pose.orientation.y = orientation.y
        message.pose.orientation.z = orientation.z
        message.pose.orientation.w = orientation.w
        self.target_pub.publish(message)

    def _publish_markers(self, sample: Sample, now) -> None:
        markers = [
            self._paper_marker(now),
            self._sphere_marker(1, "target_tip", sample.target, (0.1, 0.8, 1.0), now),
            self._sphere_marker(2, "actual_tip", sample.actual, (1.0, 0.2, 0.1), now),
            self._path_marker(3, "target_path", self.target_path, (0.1, 0.8, 1.0), now),
            self._path_marker(4, "actual_path", self.actual_path, (1.0, 0.2, 0.1), now),
            self._text_marker(sample, now),
        ]
        markers.extend(self._axis_markers(sample.target, self.initial_orientation, now))
        self.marker_pub.publish(MarkerArray(markers=markers))

    def _base_marker(self, marker_id: int, namespace: str, marker_type: int, now) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = now.to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _paper_marker(self, now) -> Marker:
        marker = self._base_marker(0, "paper", Marker.CUBE, now)
        marker.pose.position.x = self.paper_center.x
        marker.pose.position.y = self.paper_center.y
        marker.pose.position.z = self.paper_center.z
        marker.scale.x = self.paper_width
        marker.scale.y = self.paper_height
        marker.scale.z = 0.002
        marker.color = ColorRGBA(r=0.92, g=0.92, b=0.92, a=0.75)
        return marker

    def _sphere_marker(self, marker_id, namespace, point, color, now) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.SPHERE, now)
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
        marker.scale.x = marker.scale.y = marker.scale.z = 0.012
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
        return marker

    def _path_marker(self, marker_id, namespace, points, color, now) -> Marker:
        marker = self._base_marker(marker_id, namespace, Marker.LINE_STRIP, now)
        marker.scale.x = 0.003
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.9)
        marker.points = [Point(x=point.x, y=point.y, z=point.z) for point in points]
        return marker

    def _axis_markers(self, target, orientation, now) -> list[Marker]:
        axes = (
            (Point3(0.045, 0.0, 0.0), (1.0, 0.0, 0.0)),
            (Point3(0.0, 0.045, 0.0), (0.0, 1.0, 0.0)),
            (Point3(0.0, 0.0, 0.045), (0.0, 0.0, 1.0)),
        )
        pose = PoseTarget(target, orientation)
        markers = []
        for index, (axis, color) in enumerate(axes):
            endpoint = transform_point(pose, axis)
            marker = self._base_marker(10 + index, "target_axes", Marker.ARROW, now)
            marker.points = [
                Point(x=target.x, y=target.y, z=target.z),
                Point(x=endpoint.x, y=endpoint.y, z=endpoint.z),
            ]
            marker.scale.x = 0.003
            marker.scale.y = 0.007
            marker.scale.z = 0.010
            marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
            markers.append(marker)
        return markers

    def _text_marker(self, sample: Sample, now) -> Marker:
        marker = self._base_marker(20, "status", Marker.TEXT_VIEW_FACING, now)
        marker.pose.position.x = self.paper_center.x
        marker.pose.position.y = self.paper_center.y
        marker.pose.position.z = self.paper_center.z + 0.10
        marker.scale.z = 0.025
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.2, a=1.0)
        marker.text = (
            f"{sample.phase} | target {sample.target_speed:.3f} m/s | "
            f"actual {sample.actual_speed:.3f} m/s"
        )
        return marker

    def _send_script(self, text: str) -> None:
        self.script_pub.publish(String(data=text))

    def _state_elapsed(self) -> float:
        return (self.get_clock().now() - self.state_started).nanoseconds * 1e-9

    def _finish(self) -> None:
        self._send_stop()
        summary = summarize_samples(self.samples, times=self.times)
        summary.update(
            {
                "success": True,
                "speed_mps": self.speed,
                "acceleration_mps2": self.acceleration,
                "phase_times_sec": {
                    "initial_settle": self.times.initial_settle,
                    "speedl_plus_x": self.times.motion,
                    "final_settle": self.times.final_settle,
                    "steady_window_relative": [
                        self.times.steady_start,
                        self.times.steady_end,
                    ],
                },
            }
        )
        self._write_outputs(summary)
        self.exit_code = 0
        self.finished = True
        self.get_logger().info(
            f"SpeedL benchmark complete: {summary['verdict']} "
            f"output_dir={self.output_dir}"
        )
        rclpy.shutdown()

    def _fail(self, reason: str) -> None:
        self._send_stop()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        result = {"success": False, "reason": reason}
        (self.output_dir / "speedl_result.json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )
        self.get_logger().error(f"SpeedL benchmark failed: {reason}")
        self.exit_code = 2
        self.finished = True
        rclpy.shutdown()

    def _send_stop(self) -> None:
        if not self.stop_sent:
            self._send_script(stopl_script(self.acceleration))
            self.stop_sent = True

    def _write_outputs(self, summary: dict) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        with (self.output_dir / "speedl_samples.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "elapsed_sec",
                    "phase",
                    "target_x",
                    "target_y",
                    "target_z",
                    "actual_x",
                    "actual_y",
                    "actual_z",
                    "target_speed_mps",
                    "actual_speed_mps",
                    *[f"{name}_velocity_radps" for name in JOINT_NAMES],
                ]
            )
            for sample in self.samples:
                writer.writerow(
                    [
                        sample.elapsed,
                        sample.phase,
                        sample.target.x,
                        sample.target.y,
                        sample.target.z,
                        sample.actual.x,
                        sample.actual.y,
                        sample.actual.z,
                        sample.target_speed,
                        sample.actual_speed,
                        *sample.joint_velocities,
                    ]
                )
        (self.output_dir / "speedl_result.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )
        steady = summary["steady_motion_window"]
        full = summary["full_motion_window"]
        markdown = [
            "# URSim SpeedL Benchmark",
            "",
            f"- Verdict: `{summary['verdict']}`",
            f"- Command: `speedl(+X {self.speed:.3f} m/s, "
            f"a={self.acceleration:.3f} m/s², t={self.times.motion:.1f} s)`",
            "",
            "| Window | Target mean | Actual mean | Path ratio | Position RMS |",
            "|---|---:|---:|---:|---:|",
            self._metric_row("Full 5 s", full),
            self._metric_row("Steady 0.5–4.5 s", steady),
            "",
            "## Maximum joint velocity",
            "",
            "| Joint | Max abs velocity (rad/s) |",
            "|---|---:|",
        ]
        markdown.extend(
            f"| {name} | {value:.6f} |"
            for name, value in summary["joint_max_abs_velocity_radps"].items()
        )
        (self.output_dir / "speedl_result.md").write_text(
            "\n".join(markdown) + "\n", encoding="utf-8"
        )
        self._write_plot()

    @staticmethod
    def _metric_row(label: str, metrics: dict) -> str:
        return (
            f"| {label} | {metrics.get('target_mean_speed_mps', 0.0):.6f} | "
            f"{metrics.get('actual_mean_speed_mps', 0.0):.6f} | "
            f"{metrics.get('path_ratio') or 0.0:.6f} | "
            f"{metrics.get('position_rms_m', 0.0):.6f} |"
        )

    def _write_plot(self) -> None:
        import matplotlib

        matplotlib.use("Agg")
        from matplotlib import pyplot as plt

        figure, axis = plt.subplots(figsize=(10, 4))
        axis.plot(
            [sample.elapsed for sample in self.samples],
            [sample.target_speed for sample in self.samples],
            label="target",
        )
        axis.plot(
            [sample.elapsed for sample in self.samples],
            [sample.actual_speed for sample in self.samples],
            label="actual",
        )
        axis.axvspan(
            self.times.motion_start + self.times.steady_start,
            self.times.motion_start + self.times.steady_end,
            alpha=0.12,
            color="green",
            label="steady window",
        )
        axis.set(xlabel="Time (s)", ylabel="Pen-tip speed (m/s)")
        axis.grid(True)
        axis.legend()
        figure.tight_layout()
        figure.savefig(self.output_dir / "speedl_speed.png", dpi=150)
        plt.close(figure)

    def destroy_node(self):
        if not self.finished:
            self._send_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeedLBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._send_stop()
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
