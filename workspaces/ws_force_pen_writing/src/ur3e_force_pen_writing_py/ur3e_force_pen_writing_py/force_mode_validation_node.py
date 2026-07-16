from collections import deque
from dataclasses import dataclass
import math
import time

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Point, PoseStamped, WrenchStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode
from ur_msgs.srv import SetForceMode, SetPayload
from visualization_msgs.msg import Marker

from .geometry import Point3, PoseTarget, Quaternion, rotate_vector


REQUIRED_CONFIRMATION = "I_CONFIRM_REAL_FORCE_MODE_TEST"
STATE_WAITING = "WAITING"
STATE_READY = "READY"
STATE_BASELINING = "BASELINING"
STATE_SWITCHING_TO_FORCE = "SWITCHING_TO_FORCE"
STATE_ACTIVE = "FORCE_MODE_ACTIVE"
STATE_STOPPING_FORCE = "STOPPING_FORCE"
STATE_RESTORING_POSITION = "RESTORING_POSITION"
STATE_HOLDING_POSITION = "HOLDING_POSITION"
STATE_RETRACTING = "RETRACTING"
STATE_SUCCEEDED = "SUCCEEDED"
STATE_ABORTED = "ABORTED"


@dataclass(frozen=True)
class ForceTestProfile:
    name: str
    target_force_n: float
    duration_sec: float
    max_displacement_m: float
    success_displacement_m: float = 0.0
    contact_force_n: float = 0.0
    contact_hold_sec: float = 0.0


PROFILES = {
    "hold": ForceTestProfile("hold", 0.0, 2.0, 0.001),
    "zero": ForceTestProfile("zero", 0.0, 2.0, 0.001),
    "direction": ForceTestProfile("direction", 0.5, 2.0, 0.001, 0.0002),
    "contact": ForceTestProfile(
        "contact",
        2.0,
        5.0,
        0.005,
        contact_force_n=1.0,
        contact_hold_sec=1.0,
    ),
}

POSITION_CONTROLLER = "joint_trajectory_controller"
FORCE_CONTROLLER = "force_mode_controller"


def controller_switch_request(activate: str, deactivate: str) -> SwitchController.Request:
    request = SwitchController.Request()
    request.activate_controllers = [activate]
    request.deactivate_controllers = [deactivate]
    request.strictness = SwitchController.Request.STRICT
    request.activate_asap = True
    request.timeout = Duration(sec=5)
    return request


def controller_switch_verified(
    states: dict[str, str], activate: str, deactivate: str
) -> bool:
    return states.get(activate) == "active" and states.get(deactivate) == "inactive"


def projected_displacement(
    start: Point3,
    current: Point3,
    axis: tuple[float, float, float],
) -> float:
    return sum(
        delta * direction
        for delta, direction in zip(
            (current.x - start.x, current.y - start.y, current.z - start.z),
            axis,
        )
    )


def force_delta_norm(
    baseline: tuple[float, float, float],
    current: tuple[float, float, float],
) -> float:
    return math.sqrt(
        sum((value - origin) ** 2 for value, origin in zip(current, baseline))
    )


def retracted_pose(
    current: PoseTarget,
    axis: tuple[float, float, float],
    distance_m: float,
) -> PoseTarget:
    return PoseTarget(
        position=Point3(
            current.position.x - axis[0] * distance_m,
            current.position.y - axis[1] * distance_m,
            current.position.z - axis[2] * distance_m,
        ),
        orientation=current.orientation,
    )


def position_distance(a: Point3, b: Point3) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def quaternion_distance(a: Quaternion, b: Quaternion) -> float:
    values_a = (a.x, a.y, a.z, a.w)
    values_b = (b.x, b.y, b.z, b.w)
    norm_a = math.sqrt(sum(value * value for value in values_a))
    norm_b = math.sqrt(sum(value * value for value in values_b))
    if norm_a == 0.0 or norm_b == 0.0:
        return math.inf
    dot = abs(sum(x * y for x, y in zip(values_a, values_b)) / (norm_a * norm_b))
    return 2.0 * math.acos(min(1.0, dot))


class ForceModeValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("force_mode_validation")
        confirmation = str(self.declare_parameter("human_confirmation", "").value)
        if confirmation != REQUIRED_CONFIRMATION:
            raise ValueError(
                f"human_confirmation must be {REQUIRED_CONFIRMATION}"
            )

        self.base_frame = str(self.declare_parameter("base_frame", "base").value)
        self.tool_frame = str(
            self.declare_parameter("tool_frame", "tool0_controller").value
        )
        self.servo_base_frame = str(
            self.declare_parameter("servo_base_frame", "base_link").value
        )
        self.servo_tool_frame = str(
            self.declare_parameter("servo_tool_frame", "tool0").value
        )
        self.wrench_topic = str(
            self.declare_parameter(
                "wrench_topic", "/force_torque_sensor_broadcaster/wrench"
            ).value
        )
        self.max_speed_mps = float(self.declare_parameter("max_speed_mps", 0.002).value)
        self.max_force_n = float(self.declare_parameter("max_force_n", 10.0).value)
        self.retract_distance_m = float(
            self.declare_parameter("retract_distance_m", 0.003).value
        )
        self.retract_timeout_sec = float(
            self.declare_parameter("retract_timeout_sec", 3.0).value
        )
        self.data_timeout_sec = float(
            self.declare_parameter("data_timeout_sec", 0.2).value
        )
        self.baseline_duration_sec = float(
            self.declare_parameter("baseline_duration_sec", 2.0).value
        )
        self.hold_duration_sec = float(
            self.declare_parameter("hold_duration_sec", 1.0).value
        )
        self.hold_translation_limit_m = float(
            self.declare_parameter("hold_translation_limit_m", 0.001).value
        )
        self.hold_rotation_limit_rad = float(
            self.declare_parameter(
                "hold_rotation_limit_rad", math.radians(1.0)
            ).value
        )
        self._validate_parameters()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._wrench_samples = deque(maxlen=500)
        self._last_wrench_time = 0.0
        self._servo_status_time = 0.0
        self._servo_healthy = False
        self._robot_running = False
        self._safety_normal = False
        self._dashboard_pending = False
        self._setup_pending = False
        self._controller_ready = False
        self._servo_pose_ready = False
        self._state = STATE_WAITING
        self._profile: ForceTestProfile | None = None
        self._start_pose: PoseTarget | None = None
        self._axis = (0.0, 0.0, 1.0)
        self._baseline_force = (0.0, 0.0, 0.0)
        self._state_started_at = 0.0
        self._contact_started_at: float | None = None
        self._retract_target: PoseTarget | None = None
        self._hold_target: PoseTarget | None = None
        self._servo_axis = (0.0, 0.0, 1.0)
        self._hold_verified = False
        self._pending_success = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(
            String, "/pen_writing/force_mode/status", latched
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, "/servo_node/pose_target_cmds", 10
        )
        self._marker_pub = self.create_publisher(
            Marker, "/pen_writing/force_mode/direction", latched
        )
        self.create_subscription(WrenchStamped, self.wrench_topic, self._on_wrench, 10)
        self.create_subscription(
            ServoStatus, "/servo_node/status", self._on_servo_status, 10
        )

        self._list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._servo_mode_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )
        self._payload_client = self.create_client(
            SetPayload, "/io_and_status_controller/set_payload"
        )
        self._zero_ft_client = self.create_client(
            Trigger, "/io_and_status_controller/zero_ftsensor"
        )
        self._start_force_client = self.create_client(
            SetForceMode, "/force_mode_controller/start_force_mode"
        )
        self._stop_force_client = self.create_client(
            Trigger, "/force_mode_controller/stop_force_mode"
        )
        self._robot_mode_client = self.create_client(
            GetRobotMode, "/dashboard_client/get_robot_mode"
        )
        self._safety_mode_client = self.create_client(
            GetSafetyMode, "/dashboard_client/get_safety_mode"
        )

        for name in PROFILES:
            self.create_service(
                Trigger,
                f"/pen_writing/force_mode/start_{name}",
                lambda request, response, profile=name: self._start_test(
                    profile, request, response
                ),
            )
        self.create_service(
            Trigger, "/pen_writing/force_mode/stop", self._stop_test
        )
        self.create_timer(0.01, self._on_timer)
        self.create_timer(0.5, self._poll_dashboard)
        self.create_timer(0.5, self._ensure_setup)
        self._publish_status("waiting for real robot control interfaces")

    def _validate_parameters(self) -> None:
        if not 0.0 < self.max_speed_mps <= 0.002:
            raise ValueError("max_speed_mps must be in (0, 0.002]")
        if not 0.0 < self.max_force_n <= 10.0:
            raise ValueError("max_force_n must be in (0, 10]")
        if not 0.0 < self.retract_distance_m <= 0.003:
            raise ValueError("retract_distance_m must be in (0, 0.003]")
        if self.data_timeout_sec <= 0.0 or self.baseline_duration_sec <= 0.0:
            raise ValueError("data and baseline timeouts must be positive")
        if not 0.0 < self.hold_duration_sec <= 2.0:
            raise ValueError("hold_duration_sec must be in (0, 2]")
        if not 0.0 < self.hold_translation_limit_m <= 0.001:
            raise ValueError("hold_translation_limit_m must be in (0, 0.001]")
        if not 0.0 < self.hold_rotation_limit_rad <= math.radians(1.0):
            raise ValueError("hold_rotation_limit_rad must be in (0, 1 degree]")

    def _on_wrench(self, msg: WrenchStamped) -> None:
        self._last_wrench_time = time.monotonic()
        self._wrench_samples.append(
            (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z)
        )

    def _on_servo_status(self, msg: ServoStatus) -> None:
        self._servo_status_time = time.monotonic()
        self._servo_healthy = msg.code == ServoStatus.NO_WARNING

    def _poll_dashboard(self) -> None:
        if self._dashboard_pending:
            return
        if not (
            self._robot_mode_client.service_is_ready()
            and self._safety_mode_client.service_is_ready()
        ):
            return
        self._dashboard_pending = True
        robot_future = self._robot_mode_client.call_async(GetRobotMode.Request())
        safety_future = self._safety_mode_client.call_async(GetSafetyMode.Request())

        def done(_future) -> None:
            if not robot_future.done() or not safety_future.done():
                return
            self._dashboard_pending = False
            try:
                robot = robot_future.result()
                safety = safety_future.result()
                self._robot_running = bool(
                    robot and robot.success and robot.robot_mode.mode == RobotMode.RUNNING
                )
                self._safety_normal = bool(
                    safety and safety.success and safety.safety_mode.mode == SafetyMode.NORMAL
                )
            except Exception:  # noqa: BLE001 - transport failure blocks motion
                self._robot_running = False
                self._safety_normal = False

        robot_future.add_done_callback(done)
        safety_future.add_done_callback(done)

    def _ensure_setup(self) -> None:
        if self._state != STATE_WAITING:
            return
        if self._setup_pending or (self._controller_ready and self._servo_pose_ready):
            return
        if not self._list_client.service_is_ready():
            return
        self._setup_pending = True
        future = self._list_client.call_async(ListControllers.Request())

        def listed(done_future) -> None:
            response = done_future.result()
            states = {item.name: item.state for item in response.controller}
            if states.get(POSITION_CONTROLLER) == "active":
                self._controller_ready = True
                self._request_servo_pose_mode()
                return
            self._setup_pending = False
            self._publish_status(
                f"waiting for {POSITION_CONTROLLER} to become active"
            )

        future.add_done_callback(listed)

    def _request_servo_pose_mode(self) -> None:
        if not self._servo_mode_client.service_is_ready():
            self._setup_pending = False
            return
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.POSE
        future = self._servo_mode_client.call_async(request)

        def done(done_future) -> None:
            response = done_future.result()
            self._servo_pose_ready = bool(response and response.success)
            self._setup_pending = False
            if self._servo_pose_ready and self._state == STATE_WAITING:
                self._state = STATE_READY
                self._publish_status("ready; verify RViz direction before starting a profile")

        future.add_done_callback(done)

    def _start_test(self, profile_name: str, _request, response):
        now = time.monotonic()
        if profile_name != "hold" and not self._hold_verified:
            response.success = False
            response.message = "run and pass start_hold before motion profiles"
            return response
        reason = self._start_block_reason(now)
        if reason:
            response.success = False
            response.message = reason
            return response
        pose = self._lookup_tool_pose()
        if pose is None:
            response.success = False
            response.message = "tool TF unavailable"
            return response

        self._profile = PROFILES[profile_name]
        if profile_name == "hold":
            self._hold_verified = False
        self._start_pose = pose
        self._axis = rotate_vector(pose.orientation, (0.0, 0.0, 1.0))
        self._publish_direction_marker(pose)
        self._pending_success = False
        self._contact_started_at = None
        self._state = STATE_BASELINING
        self._state_started_at = now
        self._wrench_samples.clear()
        self._publish_status(f"{profile_name}: setting payload=0 and zeroing F/T")
        payload = SetPayload.Request()
        payload.mass = 0.0
        payload_future = self._payload_client.call_async(payload)

        def payload_done(done_future) -> None:
            result = done_future.result()
            if result is None or not result.success:
                self._abort("set_payload(0) failed", safe_retract=False)
                return
            zero_future = self._zero_ft_client.call_async(Trigger.Request())

            def zero_done(zero_done_future) -> None:
                zero_result = zero_done_future.result()
                if zero_result is None or not zero_result.success:
                    self._abort("zero_ftsensor failed", safe_retract=False)
                    return
                self._state_started_at = time.monotonic()
                self._wrench_samples.clear()
                self._publish_status(f"{profile_name}: collecting F/T baseline")

            zero_future.add_done_callback(zero_done)

        payload_future.add_done_callback(payload_done)
        response.success = True
        response.message = f"{profile_name} profile accepted"
        return response

    def _start_block_reason(self, now: float) -> str | None:
        if self._state not in (STATE_READY, STATE_SUCCEEDED, STATE_ABORTED):
            return f"test unavailable while state={self._state}"
        if not self._controller_ready or not self._servo_pose_ready:
            return "position controller or Servo POSE mode is not ready"
        if not self._robot_running or not self._safety_normal:
            return "robot must be RUNNING with safety mode NORMAL"
        if now - self._last_wrench_time > self.data_timeout_sec:
            return f"wrench data is stale on {self.wrench_topic}"
        if not self._servo_healthy or now - self._servo_status_time > self.data_timeout_sec:
            return "MoveIt Servo status is stale or unhealthy"
        required = (self._payload_client, self._zero_ft_client, self._start_force_client)
        if not all(client.service_is_ready() for client in required):
            return "payload, zero F/T, or force mode service is unavailable"
        return None

    def _stop_test(self, _request, response):
        if self._state not in (
            STATE_BASELINING,
            STATE_ACTIVE,
            STATE_HOLDING_POSITION,
            STATE_RETRACTING,
        ):
            response.success = False
            response.message = f"no active test: state={self._state}"
            return response
        self._abort("operator requested stop", safe_retract=False)
        response.success = True
        response.message = "stop requested"
        return response

    def _on_timer(self) -> None:
        now = time.monotonic()
        if self._state == STATE_BASELINING:
            if not self._live_data_healthy(now):
                self._abort("state or sensor data became unhealthy", safe_retract=False)
            elif now - self._state_started_at >= self.baseline_duration_sec:
                if len(self._wrench_samples) < 5:
                    self._abort("insufficient F/T baseline samples", safe_retract=False)
                else:
                    self._baseline_force = tuple(
                        sum(sample[index] for sample in self._wrench_samples)
                        / len(self._wrench_samples)
                        for index in range(3)
                    )
                    self._switch_to_force_mode()
        elif self._state == STATE_ACTIVE:
            self._monitor_active(now)
        elif self._state == STATE_HOLDING_POSITION:
            self._monitor_hold(now)
        elif self._state == STATE_RETRACTING:
            self._monitor_retraction(now)

    def _live_data_healthy(self, now: float) -> bool:
        return (
            self._robot_running
            and self._safety_normal
            and self._servo_healthy
            and now - self._last_wrench_time <= self.data_timeout_sec
            and now - self._servo_status_time <= self.data_timeout_sec
        )

    def _start_force_mode(self) -> None:
        assert self._profile is not None and self._start_pose is not None
        request = SetForceMode.Request()
        request.task_frame = self._pose_message(self._start_pose, self.base_frame)
        request.selection_vector_z = True
        request.wrench.force.z = self._profile.target_force_n
        request.type = SetForceMode.Request.NO_TRANSFORM
        request.speed_limits.linear.z = self.max_speed_mps
        request.deviation_limits = [0.005] * 6
        request.damping_factor = 0.5
        request.gain_scaling = 0.3
        future = self._start_force_client.call_async(request)

        def done(done_future) -> None:
            result = done_future.result()
            if result is None or not result.success:
                self._restore_after_failed_force_start()
                return
            self._state = STATE_ACTIVE
            self._state_started_at = time.monotonic()
            self._publish_status(
                f"{self._profile.name}: force mode active "
                f"target={self._profile.target_force_n:.1f}N"
            )

        future.add_done_callback(done)

    def _switch_to_force_mode(self) -> None:
        self._state = STATE_SWITCHING_TO_FORCE
        self._controller_ready = False
        self._publish_status("switching position controller to force mode controller")

        def done(success: bool) -> None:
            if not success:
                self._state = STATE_RESTORING_POSITION
                self._publish_status(
                    "force controller switch failed; restoring position controller"
                )

                def restored(restored_successfully: bool) -> None:
                    self._state = STATE_ABORTED
                    self._publish_status(
                        "force controller switch failed; position controller restored"
                        if restored_successfully
                        else "controller switch and position restore failed; use robot stop"
                    )

                self._restore_position_controller(restored)
                return
            self._start_force_mode()

        self._switch_controllers(
            activate=FORCE_CONTROLLER,
            deactivate=POSITION_CONTROLLER,
            done=done,
        )

    def _restore_after_failed_force_start(self) -> None:
        self._state = STATE_RESTORING_POSITION
        self._publish_status("start_force_mode failed; restoring position controller")

        def done(success: bool) -> None:
            self._state = STATE_ABORTED
            self._publish_status(
                "start_force_mode failed; position controller restored"
                if success
                else (
                    "start_force_mode failed and position controller restore failed; "
                    "use robot stop"
                )
            )

        self._restore_position_controller(done)

    def _monitor_active(self, now: float) -> None:
        assert self._profile is not None and self._start_pose is not None
        if not self._live_data_healthy(now):
            self._abort("state or sensor data became unhealthy", safe_retract=False)
            return
        pose = self._lookup_tool_pose()
        if pose is None:
            self._abort("tool TF unavailable", safe_retract=False)
            return
        displacement = projected_displacement(
            self._start_pose.position, pose.position, self._axis
        )
        force = force_delta_norm(self._baseline_force, self._wrench_samples[-1])
        if force > self.max_force_n:
            self._abort(f"force limit exceeded: {force:.3f}N", safe_retract=True)
            return
        if abs(displacement) > self._profile.max_displacement_m:
            self._abort(
                f"displacement limit exceeded: {displacement:.6f}m",
                safe_retract=True,
            )
            return
        elapsed = now - self._state_started_at
        if (
            self._profile.name == "direction"
            and displacement >= self._profile.success_displacement_m
        ):
            self._finish_active(success=True, reason="positive tool-Z direction verified")
            return
        elif self._profile.name == "contact":
            if force >= self._profile.contact_force_n:
                self._contact_started_at = self._contact_started_at or now
                if now - self._contact_started_at >= self._profile.contact_hold_sec:
                    self._finish_active(success=True, reason="foam contact held")
                    return
            else:
                self._contact_started_at = None
        if elapsed >= self._profile.duration_sec:
            success = self._profile.name in ("hold", "zero")
            reason = (
                "zero-force drift test complete" if success else "profile timed out"
            )
            self._finish_active(success=success, reason=reason)

    def _finish_active(self, *, success: bool, reason: str) -> None:
        if self._state != STATE_ACTIVE:
            return
        self._state = STATE_STOPPING_FORCE
        self._pending_success = success
        self._publish_status(f"stopping force mode: {reason}")
        future = self._stop_force_client.call_async(Trigger.Request())

        def done(done_future) -> None:
            result = done_future.result()
            if result is None or not result.success:
                self._state = STATE_ABORTED
                self._publish_status("stop_force_mode failed; use robot stop")
                return
            self._state = STATE_RESTORING_POSITION
            self._publish_status("force mode stopped; restoring position controller")

            def restored(successfully: bool) -> None:
                if not successfully:
                    self._state = STATE_ABORTED
                    self._publish_status(
                        "position controller restore failed; use robot stop"
                    )
                    return
                self._begin_hold_position()

            self._restore_position_controller(restored)

        future.add_done_callback(done)

    def _begin_hold_position(self) -> None:
        pose = self._lookup_pose(self.servo_base_frame, self.servo_tool_frame)
        axis = self._force_axis_in_servo_base()
        if pose is None or axis is None:
            self._state = STATE_ABORTED
            self._publish_status(
                "position controller restored but Servo frame TF is unavailable"
            )
            return
        self._hold_target = pose
        self._servo_axis = axis
        self._state = STATE_HOLDING_POSITION
        self._state_started_at = time.monotonic()
        self._publish_status(
            f"holding current {self.servo_tool_frame} pose without retraction"
        )

    def _monitor_hold(self, now: float) -> None:
        assert self._hold_target is not None and self._profile is not None
        if not self._live_data_healthy(now):
            self._state = STATE_ABORTED
            self._publish_status("hold aborted because state or data became unhealthy")
            return
        self._pose_pub.publish(
            self._pose_message(self._hold_target, self.servo_base_frame)
        )
        pose = self._lookup_pose(self.servo_base_frame, self.servo_tool_frame)
        if pose is None:
            self._state = STATE_ABORTED
            self._publish_status("hold aborted because Servo frame TF is unavailable")
            return
        translation = position_distance(pose.position, self._hold_target.position)
        rotation = quaternion_distance(pose.orientation, self._hold_target.orientation)
        if translation > self.hold_translation_limit_m:
            self._state = STATE_ABORTED
            self._publish_status(f"hold translation exceeded: {translation:.6f}m")
            return
        if rotation > self.hold_rotation_limit_rad:
            self._state = STATE_ABORTED
            self._publish_status(f"hold rotation exceeded: {rotation:.6f}rad")
            return
        if now - self._state_started_at < self.hold_duration_sec:
            return
        if self._profile.name == "hold":
            self._hold_verified = self._pending_success
            self._state = STATE_SUCCEEDED if self._pending_success else STATE_ABORTED
            self._publish_status(
                "hold verification succeeded without retraction"
                if self._pending_success
                else "force limits failed; hold completed without retraction"
            )
            return
        self._retract_target = retracted_pose(
            self._hold_target, self._servo_axis, self.retract_distance_m
        )
        self._state = STATE_RETRACTING
        self._state_started_at = now
        self._publish_status("hold verified; retracting 3 mm in Servo frames")

    def _monitor_retraction(self, now: float) -> None:
        assert self._retract_target is not None
        if not self._live_data_healthy(now):
            self._state = STATE_ABORTED
            self._publish_status("retraction aborted because state or data became unhealthy")
            return
        self._pose_pub.publish(
            self._pose_message(self._retract_target, self.servo_base_frame)
        )
        pose = self._lookup_pose(self.servo_base_frame, self.servo_tool_frame)
        if pose and position_distance(pose.position, self._retract_target.position) <= 0.001:
            self._state = STATE_SUCCEEDED if self._pending_success else STATE_ABORTED
            self._publish_status(
                "profile succeeded and retraction verified"
                if self._pending_success
                else "profile aborted; safety retraction verified"
            )
        elif now - self._state_started_at > self.retract_timeout_sec:
            self._state = STATE_ABORTED
            self._publish_status("retraction timed out")

    def _abort(self, reason: str, *, safe_retract: bool) -> None:
        if self._state == STATE_ACTIVE and self._stop_force_client.service_is_ready():
            if safe_retract and self._robot_running and self._safety_normal:
                self._finish_active(success=False, reason=reason)
                return
            self._state = STATE_RESTORING_POSITION
            future = self._stop_force_client.call_async(Trigger.Request())

            def stopped(done_future) -> None:
                result = done_future.result()
                if result is None or not result.success:
                    self._state = STATE_ABORTED
                    self._publish_status(f"{reason}; stop_force_mode failed; use robot stop")
                    return

                def restored(success: bool) -> None:
                    self._state = STATE_ABORTED
                    self._publish_status(
                        f"{reason}; position controller restored"
                        if success
                        else f"{reason}; position controller restore failed; use robot stop"
                    )

                self._restore_position_controller(restored)

            future.add_done_callback(stopped)
            return
        self._state = STATE_ABORTED
        self._publish_status(reason)

    def _restore_position_controller(self, done) -> None:
        def restored(success: bool) -> None:
            self._controller_ready = success
            done(success)

        self._switch_controllers(
            activate=POSITION_CONTROLLER,
            deactivate=FORCE_CONTROLLER,
            done=restored,
        )

    def _switch_controllers(self, *, activate: str, deactivate: str, done) -> None:
        if not self._switch_client.service_is_ready():
            done(False)
            return
        request = controller_switch_request(activate, deactivate)
        future = self._switch_client.call_async(request)

        def switched(done_future) -> None:
            result = done_future.result()
            if result is None or not result.ok or not self._list_client.service_is_ready():
                done(False)
                return
            listed = self._list_client.call_async(ListControllers.Request())

            def verified(list_future) -> None:
                response = list_future.result()
                if response is None:
                    done(False)
                    return
                states = {item.name: item.state for item in response.controller}
                done(controller_switch_verified(states, activate, deactivate))

            listed.add_done_callback(verified)

        future.add_done_callback(switched)

    def _lookup_tool_pose(self) -> PoseTarget | None:
        return self._lookup_pose(self.base_frame, self.tool_frame)

    def _lookup_pose(self, base_frame: str, tool_frame: str) -> PoseTarget | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                base_frame, tool_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return PoseTarget(
            position=Point3(translation.x, translation.y, translation.z),
            orientation=Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _force_axis_in_servo_base(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.servo_base_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        rotation = transform.transform.rotation
        return rotate_vector(
            Quaternion(rotation.x, rotation.y, rotation.z, rotation.w), self._axis
        )

    def _pose_message(self, pose: PoseTarget, frame_id: str) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose.position.x
        message.pose.position.y = pose.position.y
        message.pose.position.z = pose.position.z
        message.pose.orientation.x = pose.orientation.x
        message.pose.orientation.y = pose.orientation.y
        message.pose.orientation.z = pose.orientation.z
        message.pose.orientation.w = pose.orientation.w
        return message

    def _publish_direction_marker(self, pose: PoseTarget) -> None:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "force_mode_direction"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.008
        marker.scale.y = 0.016
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.a = 1.0
        marker.points = [
            Point(x=pose.position.x, y=pose.position.y, z=pose.position.z),
            Point(
                x=pose.position.x + self._axis[0] * 0.05,
                y=pose.position.y + self._axis[1] * 0.05,
                z=pose.position.z + self._axis[2] * 0.05,
            ),
        ]
        self._marker_pub.publish(marker)

    def _publish_status(self, detail: str) -> None:
        self._status_pub.publish(String(data=f"{self._state}: {detail}"))
        message = f"Force mode validation state={self._state}: {detail}"
        if self._state == STATE_ABORTED:
            self.get_logger().error(message)
        else:
            self.get_logger().info(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ForceModeValidationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
