from dataclasses import dataclass
import math
import statistics
import time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PointStamped, PoseStamped, WrenchStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from ur_msgs.srv import SetPayload

from .geometry import (
    Point3,
    PoseTarget,
    Quaternion,
    pose_target_from_transform,
    projected_force_z_in_base,
    rotate_vector,
    transform_point,
)


IDLE = "IDLE"
ZEROING = "ZEROING"
BASELINING = "BASELINING"
DESCENDING = "DESCENDING"
RETRACTING = "RETRACTING"
SUCCEEDED = "SUCCEEDED"
ABORTED = "ABORTED"
ACTIVE_STATES = (ZEROING, BASELINING, DESCENDING, RETRACTING)


def paper_seek_controller_error(states: dict[str, str]) -> str | None:
    if states.get("joint_trajectory_controller") != "active":
        return "joint_trajectory_controller is not active"
    for name in ("passthrough_trajectory_controller", "force_mode_controller"):
        if states.get(name) == "active":
            return f"{name} must be inactive before paper seek"
    return None


def paper_seek_tf_progressed(
    *, previous_descent_m: float, actual_descent_m: float
) -> bool:
    return actual_descent_m - previous_descent_m >= 0.00005


def lowpass_force_z(
    *, previous_fz_n: float, sample_fz_n: float, alpha: float, initialized: bool
) -> float:
    return sample_fz_n if not initialized else previous_fz_n + alpha * (
        sample_fz_n - previous_fz_n
    )


def contact_force_from_baseline(
    *, filtered_fz_n: float, baseline_fz_n: float, force_axis_sign: float
) -> float:
    return force_axis_sign * (filtered_fz_n - baseline_fz_n)


def next_paper_seek_offset(
    *, current_offset_m: float, down_speed_mps: float, dt_sec: float
) -> float:
    return current_offset_m - down_speed_mps * max(dt_sec, 0.0)


def paper_seek_baseline_stats(samples: list[float]) -> tuple[float, float]:
    if not samples:
        raise ValueError("paper seek baseline needs at least one sample")
    return (
        statistics.fmean(samples),
        statistics.pstdev(samples) if len(samples) > 1 else 0.0,
    )


def paper_seek_dynamic_threshold(
    *,
    minimum_threshold_n: float,
    baseline_standard_deviation_n: float,
    sigma_multiplier: float,
) -> float:
    return max(
        minimum_threshold_n,
        sigma_multiplier * baseline_standard_deviation_n,
    )


def paper_seek_tool_pose_target(
    *,
    captured_tip_xy: tuple[float, float],
    target_tip_z: float,
    captured_tool_orientation: Quaternion,
    tool0_to_pen_tip: Point3,
) -> PoseTarget:
    offset = rotate_vector(
        captured_tool_orientation,
        (tool0_to_pen_tip.x, tool0_to_pen_tip.y, tool0_to_pen_tip.z),
    )
    return PoseTarget(
        position=Point3(
            captured_tip_xy[0] - offset[0],
            captured_tip_xy[1] - offset[1],
            target_tip_z - offset[2],
        ),
        orientation=captured_tool_orientation,
    )


@dataclass
class SeekSnapshot:
    start_tip_z: float
    tip_xy: tuple[float, float]
    orientation: Quaternion


class PaperSeekServoNode(Node):
    def __init__(self) -> None:
        super().__init__("paper_seek_servo")
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.tool_frame = str(self.declare_parameter("tool_frame", "tool0").value)
        self.pose_topic = str(
            self.declare_parameter(
                "pose_command_topic", "/servo_node/pose_target_cmds"
            ).value
        )
        self.status_topic = str(
            self.declare_parameter("servo_status_topic", "/servo_node/status").value
        )
        self.command_type_service = str(
            self.declare_parameter(
                "command_type_service", "/servo_node/switch_command_type"
            ).value
        )
        self.wrench_topic = str(
            self.declare_parameter(
                "wrench_topic", "/force_torque_sensor_broadcaster/wrench"
            ).value
        )
        self.payload_mass_kg = float(
            self.declare_parameter("payload_mass_kg", 0.085).value
        )
        self.payload_cog_xyz = self._float_list(
            "payload_cog_xyz", [0.0, 0.0, 0.0], 3
        )
        tip = self._float_list(
            "tool0_to_pen_tip_xyz", [0.00079, -0.00076, 0.15172], 3
        )
        self.tool0_to_pen_tip = Point3(*tip)
        self.baseline_duration_sec = self._positive("baseline_duration_sec", 1.0)
        self.down_speed_mps = self._bounded(
            "down_speed_mps", 0.0005, upper=0.001
        )
        self.max_down_m = self._bounded("max_down_m", 0.005, upper=0.005)
        self.contact_threshold_n = self._positive("contact_threshold_n", 0.5)
        self.confirm_samples = int(
            self.declare_parameter("contact_confirm_samples", 5).value
        )
        self.lowpass_alpha = float(self.declare_parameter("lowpass_alpha", 0.1).value)
        self.force_axis_sign = float(
            self.declare_parameter("force_axis_sign", 1.0).value
        )
        self.sigma_multiplier = self._positive("sigma_multiplier", 6.0)
        self.wrench_timeout_sec = self._positive("wrench_timeout_sec", 0.2)
        self.servo_timeout_sec = self._positive("servo_timeout_sec", 1.0)
        self.motion_timeout_sec = self._bounded(
            "motion_timeout_sec", 1.0, lower=0.2, upper=2.0
        )
        self.retract_distance_m = self._bounded(
            "retract_distance_m", 0.003, upper=0.003
        )
        self.retract_timeout_sec = self._positive("retract_timeout_sec", 3.0)
        self.retract_tolerance_m = self._positive("retract_tolerance_m", 0.001)
        self.publish_rate_hz = self._positive("publish_rate_hz", 60.0)
        if self.confirm_samples <= 0:
            raise ValueError("contact_confirm_samples must be positive")
        if not 0.0 < self.lowpass_alpha <= 1.0:
            raise ValueError("lowpass_alpha must be in (0, 1]")
        if self.force_axis_sign == 0.0:
            raise ValueError("force_axis_sign must be non-zero")
        if not 0.0 <= self.payload_mass_kg <= 0.5:
            raise ValueError("payload_mass_kg must be in [0, 0.5]")

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self._status_pub = self.create_publisher(
            String, "/pen_writing/paper_seek_status", latched
        )
        self._point_pub = self.create_publisher(
            PointStamped, "/pen_writing/detected_paper_point", latched
        )
        self.create_subscription(WrenchStamped, self.wrench_topic, self._on_wrench, 20)
        self.create_subscription(ServoStatus, self.status_topic, self._on_status, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._command_client = self.create_client(
            ServoCommandType, self.command_type_service
        )
        self._payload_client = self.create_client(
            SetPayload, "/io_and_status_controller/set_payload"
        )
        self._zero_client = self.create_client(
            Trigger, "/io_and_status_controller/zero_ftsensor"
        )
        self._controllers_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.create_service(Trigger, "/pen_writing/start_paper_seek", self._start)

        self._state = IDLE
        self._snapshot: SeekSnapshot | None = None
        self._offset_m = 0.0
        self._baseline_n = 0.0
        self._threshold_n = self.contact_threshold_n
        self._baseline_samples: list[float] = []
        self._contact_count = 0
        self._candidate: Point3 | None = None
        self._retract_target_z: float | None = None
        self._state_started = time.monotonic()
        self._last_timer = self._state_started
        self._last_progress = self._state_started
        self._last_actual_descent = 0.0
        self._last_status_time = 0.0
        self._last_wrench_time = 0.0
        self._wrench_sequence = 0
        self._evaluated_sequence = 0
        self._filtered_force = 0.0
        self._filter_initialized = False
        self._command_ready = False
        self._command_future = None
        self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self._publish_status("paper seek idle")

    def _float_list(self, name: str, default: list[float], size: int) -> list[float]:
        values = [float(value) for value in self.declare_parameter(name, default).value]
        if len(values) != size or not all(math.isfinite(value) for value in values):
            raise ValueError(f"{name} must contain {size} finite values")
        return values

    def _positive(self, name: str, default: float) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be positive")
        return value

    def _bounded(
        self,
        name: str,
        default: float,
        *,
        lower: float = 0.0,
        upper: float,
    ) -> float:
        value = float(self.declare_parameter(name, default).value)
        if not lower < value <= upper:
            raise ValueError(f"{name} must be in ({lower}, {upper}]")
        return value

    def _on_status(self, _message: ServoStatus) -> None:
        self._last_status_time = time.monotonic()

    def _on_wrench(self, message: WrenchStamped) -> None:
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        source_frame = message.header.frame_id or self.tool_frame
        if source_frame != self.base_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.base_frame, source_frame, rclpy.time.Time()
                )
            except TransformException:
                return
            rotation = transform.transform.rotation
            orientation = Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        sample = projected_force_z_in_base(
            force_xyz=(
                float(message.wrench.force.x),
                float(message.wrench.force.y),
                float(message.wrench.force.z),
            ),
            source_orientation_in_base=orientation,
        )
        self._filtered_force = lowpass_force_z(
            previous_fz_n=self._filtered_force,
            sample_fz_n=sample,
            alpha=self.lowpass_alpha,
            initialized=self._filter_initialized,
        )
        self._filter_initialized = True
        self._last_wrench_time = time.monotonic()
        self._wrench_sequence += 1
        if self._state == BASELINING:
            self._baseline_samples.append(self._filtered_force)

    def _current_tool(self) -> PoseTarget | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        return pose_target_from_transform(transform)

    def _current_tip(self) -> Point3 | None:
        tool = self._current_tool()
        return None if tool is None else transform_point(tool, self.tool0_to_pen_tip)

    def _servo_healthy(self, now: float) -> bool:
        return self._last_status_time > 0.0 and (
            now - self._last_status_time <= self.servo_timeout_sec
        )

    def _start(self, _request, response):
        now = time.monotonic()
        if self._state in ACTIVE_STATES:
            response.success = False
            response.message = f"paper seek already running: {self._state}"
            return response
        if not self._command_ready or not self._servo_healthy(now):
            response.success = False
            response.message = "MoveIt Servo pose mode is not ready"
            return response
        if self._last_wrench_time == 0.0 or now - self._last_wrench_time > self.wrench_timeout_sec:
            response.success = False
            response.message = "paper seek wrench is stale"
            return response
        if not all(
            client.service_is_ready()
            for client in (
                self._payload_client,
                self._zero_client,
                self._controllers_client,
            )
        ):
            response.success = False
            response.message = "paper seek preparation service is unavailable"
            return response
        tool = self._current_tool()
        if tool is None:
            response.success = False
            response.message = "current tool pose is not available"
            return response
        tip = transform_point(tool, self.tool0_to_pen_tip)
        self._snapshot = SeekSnapshot(
            start_tip_z=tip.z,
            tip_xy=(tip.x, tip.y),
            orientation=tool.orientation,
        )
        self._offset_m = 0.0
        self._candidate = None
        self._retract_target_z = None
        self._state = ZEROING
        self._state_started = now
        response.success = True
        response.message = f"paper seek preparation started: start_tip_z={tip.z:.6f}"
        self._publish_status(response.message)
        self._check_controllers()
        return response

    def _check_controllers(self) -> None:
        future = self._controllers_client.call_async(ListControllers.Request())

        def completed(done) -> None:
            try:
                states = {item.name: item.state for item in done.result().controller}
            except Exception as exc:  # ROS future errors must abort motion preparation.
                self._abort(f"list_controllers failed: {exc}")
                return
            error = paper_seek_controller_error(states)
            if error:
                self._abort(f"controller precheck failed: {error}")
                return
            request = SetPayload.Request()
            request.mass = self.payload_mass_kg
            request.center_of_gravity.x = self.payload_cog_xyz[0]
            request.center_of_gravity.y = self.payload_cog_xyz[1]
            request.center_of_gravity.z = self.payload_cog_xyz[2]
            payload_future = self._payload_client.call_async(request)
            payload_future.add_done_callback(self._payload_done)

        future.add_done_callback(completed)

    def _payload_done(self, future) -> None:
        result = future.result()
        if self._state != ZEROING:
            return
        if result is None or not result.success:
            self._abort("set_payload failed")
            return
        zero_future = self._zero_client.call_async(Trigger.Request())
        zero_future.add_done_callback(self._zero_done)

    def _zero_done(self, future) -> None:
        result = future.result()
        if self._state != ZEROING:
            return
        if result is None or not result.success:
            self._abort("zero_ftsensor failed")
            return
        self._state = BASELINING
        self._state_started = time.monotonic()
        self._baseline_samples = []
        self._filter_initialized = False
        self._last_wrench_time = 0.0
        self._publish_status("preparation complete; collecting baseline")

    def _ensure_pose_mode(self) -> None:
        if self._command_ready:
            return
        if self._command_future is None:
            if not self._command_client.service_is_ready():
                return
            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.POSE
            self._command_future = self._command_client.call_async(request)
            return
        if not self._command_future.done():
            return
        result = self._command_future.result()
        self._command_future = None
        self._command_ready = bool(result is not None and result.success)

    def _tick(self) -> None:
        self._ensure_pose_mode()
        now = time.monotonic()
        dt = min(max(now - self._last_timer, 0.0), 0.1)
        self._last_timer = now
        if self._state not in ACTIVE_STATES:
            return
        if not self._servo_healthy(now):
            self._abort("MoveIt Servo status timed out")
            return
        if self._state != ZEROING and (
            self._last_wrench_time == 0.0
            or now - self._last_wrench_time > self.wrench_timeout_sec
        ):
            self._abort("wrench data timed out")
            return
        if self._state == BASELINING:
            if now - self._state_started >= self.baseline_duration_sec:
                if not self._baseline_samples:
                    self._abort("no baseline wrench samples")
                    return
                mean, stddev = paper_seek_baseline_stats(self._baseline_samples)
                self._baseline_n = mean
                self._threshold_n = paper_seek_dynamic_threshold(
                    minimum_threshold_n=self.contact_threshold_n,
                    baseline_standard_deviation_n=stddev,
                    sigma_multiplier=self.sigma_multiplier,
                )
                self._state = DESCENDING
                self._state_started = now
                self._last_progress = now
                self._last_actual_descent = 0.0
                self._contact_count = 0
                self._evaluated_sequence = self._wrench_sequence
                self._publish_status(
                    f"baseline captured: mean={mean:.3f}N stddev={stddev:.3f}N "
                    f"threshold={self._threshold_n:.3f}N"
                )
        elif self._state == DESCENDING:
            self._descend(now, dt)
        elif self._state == RETRACTING:
            self._check_retract(now)
        if self._state in ACTIVE_STATES and self._snapshot is not None:
            self._publish_target(now)

    def _descend(self, now: float, dt: float) -> None:
        assert self._snapshot is not None
        tip = self._current_tip()
        if tip is None:
            self._abort("current tool pose is unavailable")
            return
        actual_descent = self._snapshot.start_tip_z - tip.z
        if paper_seek_tf_progressed(
            previous_descent_m=self._last_actual_descent,
            actual_descent_m=actual_descent,
        ):
            self._last_actual_descent = actual_descent
            self._last_progress = now
        elif now - self._last_progress > self.motion_timeout_sec:
            self._abort(
                f"actual TF descent stalled: commanded={self._offset_m:.6f}m "
                f"actual={-actual_descent:.6f}m"
            )
            return
        next_offset = next_paper_seek_offset(
            current_offset_m=self._offset_m,
            down_speed_mps=self.down_speed_mps,
            dt_sec=dt,
        )
        if abs(next_offset) > self.max_down_m:
            self._abort(
                f"maximum descent reached offset={next_offset:.6f}m "
                f"limit={self.max_down_m:.6f}m"
            )
            return
        self._offset_m = next_offset
        if self._wrench_sequence == self._evaluated_sequence:
            return
        self._evaluated_sequence = self._wrench_sequence
        contact_force = contact_force_from_baseline(
            filtered_fz_n=self._filtered_force,
            baseline_fz_n=self._baseline_n,
            force_axis_sign=self.force_axis_sign,
        )
        self._contact_count = (
            self._contact_count + 1 if contact_force >= self._threshold_n else 0
        )
        if self._contact_count < self.confirm_samples:
            return
        self._candidate = tip
        self._retract_target_z = tip.z + self.retract_distance_m
        self._state = RETRACTING
        self._state_started = now
        self._publish_status(
            f"contact confirmed from actual TF: candidate_z={tip.z:.6f} "
            f"contact_force={contact_force:.3f}N"
        )

    def _check_retract(self, now: float) -> None:
        assert self._retract_target_z is not None
        tip = self._current_tip()
        if tip is None:
            self._abort("current tool pose is unavailable during retract")
            return
        if abs(tip.z - self._retract_target_z) <= self.retract_tolerance_m:
            assert self._candidate is not None
            point = PointStamped()
            point.header.frame_id = self.base_frame
            point.header.stamp = self.get_clock().now().to_msg()
            point.point.x = self._candidate.x
            point.point.y = self._candidate.y
            point.point.z = self._candidate.z
            self._point_pub.publish(point)
            self._state = SUCCEEDED
            self._publish_status(f"paper height committed: z={self._candidate.z:.6f}")
        elif now - self._state_started > self.retract_timeout_sec:
            self._abort("retraction timed out")

    def _publish_target(self, now: float) -> None:
        assert self._snapshot is not None
        target_z = (
            self._retract_target_z
            if self._state == RETRACTING
            else self._snapshot.start_tip_z + self._offset_m
        )
        assert target_z is not None
        target = paper_seek_tool_pose_target(
            captured_tip_xy=self._snapshot.tip_xy,
            target_tip_z=target_z,
            captured_tool_orientation=self._snapshot.orientation,
            tool0_to_pen_tip=self.tool0_to_pen_tip,
        )
        message = PoseStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = target.position.x
        message.pose.position.y = target.position.y
        message.pose.position.z = target.position.z
        message.pose.orientation.x = target.orientation.x
        message.pose.orientation.y = target.orientation.y
        message.pose.orientation.z = target.orientation.z
        message.pose.orientation.w = target.orientation.w
        self._pose_pub.publish(message)

    def _abort(self, reason: str) -> None:
        self._state = ABORTED
        self._candidate = None
        self._retract_target_z = None
        self._publish_status(reason, error=True)

    def _publish_status(self, detail: str, *, error: bool = False) -> None:
        message = f"{self._state}: {detail}"
        self._status_pub.publish(String(data=message))
        (self.get_logger().error if error else self.get_logger().info)(
            f"Paper seek {message}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PaperSeekServoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
