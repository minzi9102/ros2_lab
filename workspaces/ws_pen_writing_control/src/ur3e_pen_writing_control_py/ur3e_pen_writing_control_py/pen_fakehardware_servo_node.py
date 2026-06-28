import csv
import math
from pathlib import Path
import time
from dataclasses import dataclass
from typing import Iterable, TextIO

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .joy_mapping import JoyControl, JoyMapper
from .pen_math import (
    PaperBounds,
    PlanarVelocity,
    SmoothPlanarVelocity,
    VirtualPenState,
)
from .pose_math import (
    ContinuousPenOrientation,
    Point3,
    PoseTarget,
    Quaternion,
    rotate_vector,
    tool_pose_from_pen_tip_pose,
    transform_point,
)


def has_planar_motion_intent(control: JoyControl) -> bool:
    return math.hypot(control.target_x, control.target_y) > 1e-9


def should_publish_pose_command(
    *,
    pose_command_armed: bool,
    has_motion_intent: bool,
    virtual_pen_settling: bool,
    tool_pose_aligned: bool,
    servo_health_fault: bool,
) -> bool:
    if not pose_command_armed or servo_health_fault:
        return False
    return has_motion_intent or virtual_pen_settling or not tool_pose_aligned


def is_virtual_pen_settling(
    *,
    velocity: PlanarVelocity,
    tilt_rad: float,
    speed_tolerance_mps: float,
    tilt_tolerance_rad: float,
    orientation_error_rad: float = 0.0,
) -> bool:
    return (
        math.hypot(velocity.x, velocity.y) > speed_tolerance_mps
        or abs(tilt_rad) > tilt_tolerance_rad
        or abs(orientation_error_rad) > tilt_tolerance_rad
    )


def rotate_tool_offset(quaternion: Quaternion, offset: Point3) -> Point3:
    rotated = rotate_vector(quaternion, (offset.x, offset.y, offset.z))
    return Point3(x=rotated[0], y=rotated[1], z=rotated[2])


def paper_origin_from_current_tool0(
    *,
    current_tool_position: Point3,
    current_tool_orientation: Quaternion,
    tool0_to_pen_tip: Point3,
    initial_tip_x: float,
    initial_tip_y: float,
    fixed_paper_z: float,
) -> tuple[Point3, float]:
    current_pen_tip_offset = rotate_tool_offset(
        current_tool_orientation,
        tool0_to_pen_tip,
    )
    estimated_tip_z = current_tool_position.z + current_pen_tip_offset.z
    return (
        Point3(
            x=current_tool_position.x + current_pen_tip_offset.x - initial_tip_x,
            y=current_tool_position.y + current_pen_tip_offset.y - initial_tip_y,
            z=fixed_paper_z,
        ),
        estimated_tip_z,
    )


def is_servo_status_fresh(
    *,
    status_seen: bool,
    last_status_time: float,
    now_sec: float,
    timeout_sec: float,
) -> bool:
    return status_seen and now_sec - last_status_time <= timeout_sec


@dataclass(frozen=True)
class ToolAlignmentError:
    position_m: float
    z_axis_rad: float
    full_quaternion_rad: float


def quaternion_angular_distance(
    current: Quaternion,
    target: Quaternion,
) -> float:
    current_norm = math.sqrt(
        current.x * current.x
        + current.y * current.y
        + current.z * current.z
        + current.w * current.w
    )
    target_norm = math.sqrt(
        target.x * target.x
        + target.y * target.y
        + target.z * target.z
        + target.w * target.w
    )
    if current_norm < 1e-12 or target_norm < 1e-12:
        raise ValueError("orientation quaternion norm must be non-zero")

    dot = (
        current.x * target.x
        + current.y * target.y
        + current.z * target.z
        + current.w * target.w
    ) / (current_norm * target_norm)
    return 2.0 * math.acos(clamp(abs(dot), 0.0, 1.0))


def tool_alignment_error(
    *,
    current_tool_pose: PoseTarget,
    target_tool_pose: PoseTarget,
) -> ToolAlignmentError:
    dx = current_tool_pose.position.x - target_tool_pose.position.x
    dy = current_tool_pose.position.y - target_tool_pose.position.y
    dz = current_tool_pose.position.z - target_tool_pose.position.z
    current_z = rotate_vector(current_tool_pose.orientation, (0.0, 0.0, 1.0))
    target_z = rotate_vector(target_tool_pose.orientation, (0.0, 0.0, 1.0))
    dot = clamp(
        current_z[0] * target_z[0]
        + current_z[1] * target_z[1]
        + current_z[2] * target_z[2],
        -1.0,
        1.0,
    )
    return ToolAlignmentError(
        position_m=math.sqrt(dx * dx + dy * dy + dz * dz),
        z_axis_rad=math.acos(dot),
        full_quaternion_rad=quaternion_angular_distance(
            current_tool_pose.orientation,
            target_tool_pose.orientation,
        ),
    )


def is_tool_pose_aligned(
    *,
    current_tool_pose: PoseTarget,
    target_tool_pose: PoseTarget,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
) -> bool:
    error = tool_alignment_error(
        current_tool_pose=current_tool_pose,
        target_tool_pose=target_tool_pose,
    )
    return (
        error.position_m <= position_tolerance_m
        and error.z_axis_rad <= orientation_tolerance_rad
    )


def pose_target_from_transform(transform: TransformStamped) -> PoseTarget:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return PoseTarget(
        position=Point3(x=translation.x, y=translation.y, z=translation.z),
        orientation=Quaternion(
            x=rotation.x,
            y=rotation.y,
            z=rotation.z,
            w=rotation.w,
        ),
    )


def tool_tip_point_from_tool_pose(
    *,
    tool_pose: PoseTarget,
    tool0_to_pen_tip: Point3,
) -> Point3:
    return transform_point(tool_pose, tool0_to_pen_tip)


def tool_tail_to_tip_points(
    *,
    tool_pose: PoseTarget,
    tool0_to_pen_tip: Point3,
) -> tuple[Point3, Point3]:
    return (
        tool_pose.position,
        tool_tip_point_from_tool_pose(
            tool_pose=tool_pose,
            tool0_to_pen_tip=tool0_to_pen_tip,
        ),
    )


def pose_axis_points(
    *,
    pose: PoseTarget,
    local_axis: Point3,
    axis_length_m: float,
) -> tuple[Point3, Point3]:
    endpoint = transform_point(
        pose,
        Point3(
            x=local_axis.x * axis_length_m,
            y=local_axis.y * axis_length_m,
            z=local_axis.z * axis_length_m,
        ),
    )
    return pose.position, endpoint


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class RuntimeFrames:
    base_frame: str
    paper_frame: str
    tool_frame: str


class AlignmentErrorCsvLogger:
    HEADER = (
        "elapsed_sec",
        "position_m",
        "z_axis_deg",
        "full_quaternion_deg",
        "pose_command_armed",
        "pose_command_published",
        "has_motion_intent",
        "virtual_pen_settling",
    )

    def __init__(self, *, path: str, sample_rate_hz: float) -> None:
        if sample_rate_hz <= 0.0:
            raise ValueError("alignment_error_log_rate_hz must be greater than zero")
        self.path = Path(path) if path else None
        self.sample_period_sec = 1.0 / sample_rate_hz
        self._file: TextIO | None = None
        self._writer = None
        self._started_at_sec: float | None = None
        self._last_sample_time_sec: float | None = None
        self.sample_count = 0

    @property
    def started(self) -> bool:
        return self._file is not None

    def record(
        self,
        *,
        now_sec: float,
        start_requested: bool,
        error: ToolAlignmentError | None,
        pose_command_armed: bool,
        pose_command_published: bool,
        has_motion_intent: bool,
        virtual_pen_settling: bool,
    ) -> bool:
        if self.path is None:
            return False
        if not self.started:
            if not start_requested:
                return False
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open(
                "w",
                encoding="utf-8",
                newline="",
                buffering=1,
            )
            self._writer = csv.writer(self._file)
            self._writer.writerow(self.HEADER)
            self._started_at_sec = now_sec

        if (
            self._last_sample_time_sec is not None
            and now_sec - self._last_sample_time_sec + 1e-12
            < self.sample_period_sec
        ):
            return False

        position_m = math.nan if error is None else error.position_m
        z_axis_deg = math.nan if error is None else math.degrees(error.z_axis_rad)
        full_quaternion_deg = (
            math.nan if error is None else math.degrees(error.full_quaternion_rad)
        )
        self._writer.writerow(
            (
                f"{now_sec - self._started_at_sec:.9f}",
                f"{position_m:.9f}",
                f"{z_axis_deg:.6f}",
                f"{full_quaternion_deg:.6f}",
                int(pose_command_armed),
                int(pose_command_published),
                int(has_motion_intent),
                int(virtual_pen_settling),
            )
        )
        self._last_sample_time_sec = now_sec
        self.sample_count += 1
        return True

    def close(self) -> None:
        if self._file is None:
            return
        self._file.close()
        self._file = None
        self._writer = None


class PenFakeHardwareServoNode(Node):
    """Bridge the stage-1 virtual pen model to fake-hardware MoveIt Servo POSE mode."""

    def __init__(self) -> None:
        super().__init__("pen_fakehardware_servo")

        self.frames = RuntimeFrames(
            base_frame=str(self.declare_parameter("base_frame", "base_link").value),
            paper_frame=str(self.declare_parameter("paper_frame", "paper_frame").value),
            tool_frame=str(self.declare_parameter("tool_frame", "tool0").value),
        )
        self.marker_topic = str(
            self.declare_parameter("marker_topic", "/pen_writing/markers").value
        )
        self.pose_command_topic = str(
            self.declare_parameter(
                "pose_command_topic",
                "/servo_node/pose_target_cmds",
            ).value
        )
        self.command_type_service = str(
            self.declare_parameter(
                "command_type_service",
                "/servo_node/switch_command_type",
            ).value
        )
        self.servo_status_topic = str(
            self.declare_parameter("servo_status_topic", "/servo_node/status").value
        )
        self.servo_status_timeout_sec = float(
            self.declare_parameter("servo_status_timeout_sec", 1.0).value
        )
        self.servo_status_warn_period_sec = float(
            self.declare_parameter("servo_status_warn_period_sec", 1.0).value
        )
        self.joy_topic = str(self.declare_parameter("joy_topic", "/joy").value)
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 60.0).value
        )
        self.joy_deadzone = float(self.declare_parameter("joy_deadzone", 0.08).value)
        self.joy_timeout_sec = float(
            self.declare_parameter("joy_timeout_sec", 0.25).value
        )
        self.start_from_current_tool0 = bool(
            self.declare_parameter("start_from_current_tool0", True).value
        )
        self.require_motion_before_pose_command = bool(
            self.declare_parameter("require_motion_before_pose_command", True).value
        )
        self.tf_lookup_warn_period_sec = float(
            self.declare_parameter("tf_lookup_warn_period_sec", 1.0).value
        )
        self.alignment_error_log_path = str(
            self.declare_parameter("alignment_error_log_path", "").value
        )
        self.alignment_error_log_rate_hz = float(
            self.declare_parameter("alignment_error_log_rate_hz", 20.0).value
        )
        self.max_planar_speed_mps = float(
            self.declare_parameter("max_planar_speed_mps", 0.03).value
        )
        self.acceleration_mps2 = float(
            self.declare_parameter("acceleration_mps2", 0.08).value
        )
        self.deceleration_mps2 = float(
            self.declare_parameter("deceleration_mps2", 0.16).value
        )
        self.yaw_hold_speed_mps = float(
            self.declare_parameter("yaw_hold_speed_mps", 0.005).value
        )
        self.tilt_activate_speed_mps = float(
            self.declare_parameter("tilt_activate_speed_mps", 0.01).value
        )
        self.tilt_rate_degps = float(
            self.declare_parameter("tilt_rate_degps", 10.0).value
        )
        self.untilt_rate_degps = float(
            self.declare_parameter("untilt_rate_degps", 12.0).value
        )
        self.max_pen_axis_angular_speed_degps = float(
            self.declare_parameter(
                "max_pen_axis_angular_speed_degps",
                12.0,
            ).value
        )
        self.tool_position_tolerance_m = float(
            self.declare_parameter("tool_position_tolerance_m", 0.005).value
        )
        self.tool_orientation_tolerance_deg = float(
            self.declare_parameter("tool_orientation_tolerance_deg", 3.0).value
        )
        self.pose_settle_speed_mps = float(
            self.declare_parameter("pose_settle_speed_mps", 0.002).value
        )
        self.pose_settle_tilt_deg = float(
            self.declare_parameter("pose_settle_tilt_deg", 0.5).value
        )
        self.pen_length_m = float(self.declare_parameter("pen_length_m", 0.14).value)
        self.pen_radius_m = float(self.declare_parameter("pen_radius_m", 0.006).value)
        self.pen_tip_radius_m = float(
            self.declare_parameter("pen_tip_radius_m", 0.01).value
        )
        self.target_pen_tip_axis_length_m = float(
            self.declare_parameter("target_pen_tip_axis_length_m", 0.08).value
        )
        self.target_tool0_axis_length_m = float(
            self.declare_parameter("target_tool0_axis_length_m", 0.08).value
        )
        self.tool0_axis_length_m = float(
            self.declare_parameter("tool0_axis_length_m", 0.08).value
        )
        self.fixed_tilt_deg = float(self.declare_parameter("fixed_tilt_deg", 20.0).value)
        self.paper_width_m = float(self.declare_parameter("paper_width_m", 0.24).value)
        self.paper_height_m = float(self.declare_parameter("paper_height_m", 0.16).value)
        self.paper_origin_xyz = self._declare_float_list(
            "paper_origin_xyz",
            [0.45, 0.0, 0.12],
            expected_size=3,
        )
        tool0_to_pen_tip_xyz = self._declare_float_list(
            "tool0_to_pen_tip_xyz",
            [0.0, 0.0, -self.pen_length_m],
            expected_size=3,
        )
        self.tool0_to_pen_tip = Point3(
            x=tool0_to_pen_tip_xyz[0],
            y=tool0_to_pen_tip_xyz[1],
            z=tool0_to_pen_tip_xyz[2],
        )
        initial_tip_xy = self._declare_float_list(
            "initial_tip_xy",
            [0.0, 0.0],
            expected_size=2,
        )

        self._validate_parameters()

        self._pose_publisher = self.create_publisher(
            PoseStamped,
            self.pose_command_topic,
            10,
        )
        self._marker_publisher = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._command_type_client = self.create_client(
            ServoCommandType,
            self.command_type_service,
        )
        self._command_type_future = None
        self._pose_mode_ready = False
        self._pose_command_armed = not self.require_motion_before_pose_command
        self._paper_origin = (
            None if self.start_from_current_tool0 else self._configured_paper_origin()
        )
        self._last_tf_warn_time = 0.0
        self._joy_subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self._on_joy_message,
            10,
        )
        self._servo_status_subscription = self.create_subscription(
            ServoStatus,
            self.servo_status_topic,
            self._on_servo_status_message,
            10,
        )
        self._joy_mapper = JoyMapper(deadzone=self.joy_deadzone)
        self._latest_joy_control = JoyControl()
        self._last_joy_msg_time = 0.0
        self._last_servo_status_time = 0.0
        self._last_servo_status_warn_time = 0.0
        self._servo_status_seen = False
        self._servo_health_fault = False
        self._alignment_error_logger = AlignmentErrorCsvLogger(
            path=self.alignment_error_log_path,
            sample_rate_hz=self.alignment_error_log_rate_hz,
        )
        self._velocity = SmoothPlanarVelocity(
            max_speed_mps=self.max_planar_speed_mps,
            acceleration_mps2=self.acceleration_mps2,
            deceleration_mps2=self.deceleration_mps2,
        )
        self._pen_state = VirtualPenState(
            initial_tip_x=initial_tip_xy[0],
            initial_tip_y=initial_tip_xy[1],
            initial_yaw=math.pi,
            paper_bounds=PaperBounds(
                width=self.paper_width_m,
                height=self.paper_height_m,
            ),
            yaw_hold_speed_mps=self.yaw_hold_speed_mps,
            target_tilt_rad=math.radians(self.fixed_tilt_deg),
            tilt_activate_speed_mps=self.tilt_activate_speed_mps,
            tilt_rate_radps=math.radians(self.tilt_rate_degps),
            untilt_rate_radps=math.radians(self.untilt_rate_degps),
        )
        self._pen_orientation = ContinuousPenOrientation(
            initial_pen_pose=self._pen_state.pose,
            pen_length=self.pen_length_m,
            max_axis_angular_speed_radps=math.radians(
                self.max_pen_axis_angular_speed_degps
            ),
        )
        self._last_timer_time = time.monotonic()

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "Pen fake-hardware Servo node started. "
            f"base_frame={self.frames.base_frame} tool_frame={self.frames.tool_frame} "
            f"pose_topic={self.pose_command_topic} marker_topic={self.marker_topic} "
            f"servo_status_topic={self.servo_status_topic} joy_topic={self.joy_topic} "
            f"rate={self.publish_rate_hz:.1f}Hz "
            f"servo_command_type=POSE "
            f"require_motion_before_pose_command={self.require_motion_before_pose_command} "
            f"tool_position_tolerance={self.tool_position_tolerance_m:.3f}m "
            f"tool_orientation_tolerance={self.tool_orientation_tolerance_deg:.1f}deg "
            f"max_pen_axis_rate="
            f"{self.max_pen_axis_angular_speed_degps:.1f}deg/s "
            f"alignment_error_log_rate={self.alignment_error_log_rate_hz:.1f}Hz "
            f"tool0_to_pen_tip=({self.tool0_to_pen_tip.x:.3f}, "
            f"{self.tool0_to_pen_tip.y:.3f}, {self.tool0_to_pen_tip.z:.3f})"
        )

    def _declare_float_list(
        self,
        name: str,
        default_value: list[float],
        *,
        expected_size: int,
    ) -> list[float]:
        value = self.declare_parameter(name, default_value).value
        result = [float(item) for item in value]
        if len(result) != expected_size:
            raise ValueError(f"{name} must contain {expected_size} values")
        return result

    def _validate_parameters(self) -> None:
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.joy_timeout_sec <= 0.0:
            raise ValueError("joy_timeout_sec must be greater than zero")
        if self.servo_status_timeout_sec <= 0.0:
            raise ValueError("servo_status_timeout_sec must be greater than zero")
        if self.servo_status_warn_period_sec <= 0.0:
            raise ValueError("servo_status_warn_period_sec must be greater than zero")
        if self.tf_lookup_warn_period_sec <= 0.0:
            raise ValueError("tf_lookup_warn_period_sec must be greater than zero")
        if self.alignment_error_log_rate_hz <= 0.0:
            raise ValueError("alignment_error_log_rate_hz must be greater than zero")
        if self.tool_position_tolerance_m <= 0.0:
            raise ValueError("tool_position_tolerance_m must be greater than zero")
        if self.tool_orientation_tolerance_deg <= 0.0:
            raise ValueError("tool_orientation_tolerance_deg must be greater than zero")
        if self.pose_settle_speed_mps < 0.0:
            raise ValueError("pose_settle_speed_mps must be non-negative")
        if self.pose_settle_tilt_deg < 0.0:
            raise ValueError("pose_settle_tilt_deg must be non-negative")
        if self.pen_length_m <= 0.0:
            raise ValueError("pen_length_m must be greater than zero")
        if self.pen_radius_m <= 0.0:
            raise ValueError("pen_radius_m must be greater than zero")
        if self.pen_tip_radius_m <= 0.0:
            raise ValueError("pen_tip_radius_m must be greater than zero")
        if self.target_pen_tip_axis_length_m <= 0.0:
            raise ValueError("target_pen_tip_axis_length_m must be greater than zero")
        if self.target_tool0_axis_length_m <= 0.0:
            raise ValueError("target_tool0_axis_length_m must be greater than zero")
        if self.tool0_axis_length_m <= 0.0:
            raise ValueError("tool0_axis_length_m must be greater than zero")
        if self.paper_width_m <= 0.0 or self.paper_height_m <= 0.0:
            raise ValueError("paper dimensions must be greater than zero")
        if self.fixed_tilt_deg < 0.0 or self.fixed_tilt_deg >= 90.0:
            raise ValueError("fixed_tilt_deg must be in [0, 90)")
        if self.tilt_activate_speed_mps < 0.0:
            raise ValueError("tilt_activate_speed_mps must be non-negative")
        if self.tilt_rate_degps <= 0.0:
            raise ValueError("tilt_rate_degps must be greater than zero")
        if self.untilt_rate_degps <= 0.0:
            raise ValueError("untilt_rate_degps must be greater than zero")
        if self.max_pen_axis_angular_speed_degps <= 0.0:
            raise ValueError(
                "max_pen_axis_angular_speed_degps must be greater than zero"
            )

    def _on_joy_message(self, msg: Joy) -> None:
        self._latest_joy_control = self._joy_mapper.map(msg.axes, msg.buttons)
        self._last_joy_msg_time = time.monotonic()

    def _on_servo_status_message(self, msg: ServoStatus) -> None:
        self._last_servo_status_time = time.monotonic()
        self._servo_status_seen = True
        now_sec = time.monotonic()
        if (
            msg.code != ServoStatus.NO_WARNING
            and now_sec - self._last_servo_status_warn_time
            >= self.servo_status_warn_period_sec
        ):
            self._last_servo_status_warn_time = now_sec
            self.get_logger().warn(
                f"MoveIt Servo status warning: code={msg.code} message={msg.message!r}"
            )

    def _on_timer(self) -> None:
        if self._paper_origin is None:
            self._initialize_paper_origin()
            return
        if not self._ensure_pose_mode_ready():
            return

        now_sec = time.monotonic()
        dt_sec = now_sec - self._last_timer_time
        self._last_timer_time = now_sec

        control = self._current_control(now_sec)
        has_motion_intent = has_planar_motion_intent(control)
        if has_motion_intent and not self._pose_command_armed:
            self._pose_command_armed = True
            self.get_logger().info("Motion input received. Arming POSE commands.")

        if self._pose_command_armed and not self._is_servo_status_healthy(now_sec):
            self._servo_health_fault = True
            self._pose_command_armed = False
            self.get_logger().error(
                "MoveIt Servo status timed out after pose commands were armed. "
                "Stopping pen pose publication and shutting down this node."
            )
            rclpy.shutdown()
            return

        if control.emergency_stop:
            velocity = self._velocity.stop_immediately()
            self._pose_command_armed = False
            target_tool_pose = self._make_tool_pose_target()
            current_tool_pose = self._lookup_current_tool_pose()
            self._publish_tf_markers_and_pose(
                velocity,
                target_tool_pose=target_tool_pose,
                current_tool_pose=current_tool_pose,
                publish_pose=False,
            )
            self._record_alignment_error(
                now_sec=now_sec,
                target_tool_pose=target_tool_pose,
                current_tool_pose=current_tool_pose,
                pose_command_published=False,
                has_motion_intent=has_motion_intent,
                virtual_pen_settling=False,
            )
            if control.quit_requested:
                self.get_logger().info("Joy quit requested. Pen Servo node shutting down.")
                rclpy.shutdown()
            return
        else:
            velocity = self._velocity.update(control.target_x, control.target_y, dt_sec)

        pen_pose = self._pen_state.update(velocity, dt_sec)
        self._pen_orientation.update(pen_pose, dt_sec)
        target_tool_pose = self._make_tool_pose_target()
        current_tool_pose = self._lookup_current_tool_pose()
        tool_pose_aligned = (
            current_tool_pose is not None
            and is_tool_pose_aligned(
                current_tool_pose=current_tool_pose,
                target_tool_pose=target_tool_pose,
                position_tolerance_m=self.tool_position_tolerance_m,
                orientation_tolerance_rad=math.radians(
                    self.tool_orientation_tolerance_deg
                ),
            )
        )
        virtual_pen_settling = is_virtual_pen_settling(
            velocity=velocity,
            tilt_rad=self._pen_state.pose.tilt_rad,
            speed_tolerance_mps=self.pose_settle_speed_mps,
            tilt_tolerance_rad=math.radians(self.pose_settle_tilt_deg),
            orientation_error_rad=self._pen_orientation.axis_error_rad,
        )
        publish_pose = should_publish_pose_command(
            pose_command_armed=self._pose_command_armed,
            has_motion_intent=has_motion_intent,
            virtual_pen_settling=virtual_pen_settling,
            tool_pose_aligned=tool_pose_aligned,
            servo_health_fault=self._servo_health_fault,
        )
        self._publish_tf_markers_and_pose(
            velocity,
            target_tool_pose=target_tool_pose,
            current_tool_pose=current_tool_pose,
            publish_pose=publish_pose,
        )
        self._record_alignment_error(
            now_sec=now_sec,
            target_tool_pose=target_tool_pose,
            current_tool_pose=current_tool_pose,
            pose_command_published=publish_pose,
            has_motion_intent=has_motion_intent,
            virtual_pen_settling=virtual_pen_settling,
        )

        if control.quit_requested:
            self.get_logger().info("Joy quit requested. Pen Servo node shutting down.")
            rclpy.shutdown()

    def _initialize_paper_origin(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.frames.base_frame,
                self.frames.tool_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self._warn_throttled(
                "Waiting for current tool pose before initializing paper origin: "
                f"{exc}"
            )
            return

        pose = self._pen_state.pose
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        current_tool_position = Point3(
            x=translation.x,
            y=translation.y,
            z=translation.z,
        )
        current_tool_orientation = Quaternion(
            x=rotation.x,
            y=rotation.y,
            z=rotation.z,
            w=rotation.w,
        )
        self._paper_origin, estimated_tip_z = paper_origin_from_current_tool0(
            current_tool_position=current_tool_position,
            current_tool_orientation=current_tool_orientation,
            tool0_to_pen_tip=self.tool0_to_pen_tip,
            initial_tip_x=pose.tip_x,
            initial_tip_y=pose.tip_y,
            fixed_paper_z=self.paper_origin_xyz[2],
        )
        self.get_logger().info(
            "Initialized paper origin XY from current tool0 pose and pen-tip offset; "
            "using fixed paper Z: "
            f"({self._paper_origin.x:.3f}, {self._paper_origin.y:.3f}, "
            f"{self._paper_origin.z:.3f}); estimated current pen-tip Z="
            f"{estimated_tip_z:.3f}"
        )

    def _ensure_pose_mode_ready(self) -> bool:
        if self._pose_mode_ready:
            return True
        if not self._command_type_client.service_is_ready():
            self._command_type_client.wait_for_service(timeout_sec=0.0)
            return False
        if self._command_type_future is None:
            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.POSE
            self._command_type_future = self._command_type_client.call_async(request)
            self.get_logger().info("Requested MoveIt Servo POSE command mode.")
            return False
        if not self._command_type_future.done():
            return False

        response = self._command_type_future.result()
        self._command_type_future = None
        if response is not None and response.success:
            self._pose_mode_ready = True
            self._last_timer_time = time.monotonic()
            self.get_logger().info("MoveIt Servo accepted POSE command mode.")
            return True

        self.get_logger().warn("MoveIt Servo rejected POSE command mode; retrying.")
        return False

    def _is_servo_status_healthy(self, now_sec: float) -> bool:
        return is_servo_status_fresh(
            status_seen=self._servo_status_seen,
            last_status_time=self._last_servo_status_time,
            now_sec=now_sec,
            timeout_sec=self.servo_status_timeout_sec,
        )

    def _current_control(self, now_sec: float) -> JoyControl:
        if self._last_joy_msg_time == 0.0:
            return JoyControl()
        if now_sec - self._last_joy_msg_time > self.joy_timeout_sec:
            return JoyControl()
        return self._latest_joy_control

    def _lookup_current_tool_pose(self) -> PoseTarget | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.frames.base_frame,
                self.frames.tool_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self._warn_throttled(
                f"Waiting for current tool pose while checking alignment: {exc}"
            )
            return None
        return pose_target_from_transform(transform)

    def _make_tool_pose_target(self) -> PoseTarget:
        return tool_pose_from_pen_tip_pose(
            pen_pose=self._pen_state.pose,
            paper_origin=self._paper_origin,
            pen_length=self.pen_length_m,
            tool0_to_pen_tip_xyz=self.tool0_to_pen_tip,
            orientation_override=self._pen_orientation.orientation,
        )

    def _record_alignment_error(
        self,
        *,
        now_sec: float,
        target_tool_pose: PoseTarget,
        current_tool_pose: PoseTarget | None,
        pose_command_published: bool,
        has_motion_intent: bool,
        virtual_pen_settling: bool,
    ) -> None:
        was_started = self._alignment_error_logger.started
        error = None
        if current_tool_pose is not None:
            error = tool_alignment_error(
                current_tool_pose=current_tool_pose,
                target_tool_pose=target_tool_pose,
            )
        self._alignment_error_logger.record(
            now_sec=now_sec,
            start_requested=pose_command_published,
            error=error,
            pose_command_armed=self._pose_command_armed,
            pose_command_published=pose_command_published,
            has_motion_intent=has_motion_intent,
            virtual_pen_settling=virtual_pen_settling,
        )
        if not was_started and self._alignment_error_logger.started:
            self.get_logger().info(
                "Tool alignment error recording started: "
                f"{self._alignment_error_logger.path}"
            )

    def _publish_tf_markers_and_pose(
        self,
        velocity: PlanarVelocity,
        *,
        target_tool_pose: PoseTarget,
        current_tool_pose: PoseTarget | None,
        publish_pose: bool,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(
            [
                self._paper_transform(stamp),
                self._pen_tip_transform(stamp, target_tool_pose),
            ]
        )
        self._marker_publisher.publish(
            self._make_marker_array(
                velocity,
                target_tool_pose=target_tool_pose,
                current_tool_pose=current_tool_pose,
            )
        )
        if publish_pose:
            self._pose_publisher.publish(self._make_pose_stamped(stamp, target_tool_pose))

    def _paper_transform(self, stamp) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frames.base_frame
        transform.child_frame_id = self.frames.paper_frame
        transform.transform.translation.x = self._paper_origin.x
        transform.transform.translation.y = self._paper_origin.y
        transform.transform.translation.z = self._paper_origin.z
        transform.transform.rotation.w = 1.0
        return transform

    def _pen_tip_transform(
        self,
        stamp,
        target_tool_pose: PoseTarget,
    ) -> TransformStamped:
        tip_base = transform_point(target_tool_pose, self.tool0_to_pen_tip)
        tip = self._base_to_paper_point(tip_base)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frames.paper_frame
        transform.child_frame_id = "pen_tip"
        transform.transform.translation.x = tip.x
        transform.transform.translation.y = tip.y
        transform.transform.translation.z = tip.z
        transform.transform.rotation.x = target_tool_pose.orientation.x
        transform.transform.rotation.y = target_tool_pose.orientation.y
        transform.transform.rotation.z = target_tool_pose.orientation.z
        transform.transform.rotation.w = target_tool_pose.orientation.w
        return transform

    def _make_pose_stamped(self, stamp, target: PoseTarget) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frames.base_frame
        msg.pose.position.x = target.position.x
        msg.pose.position.y = target.position.y
        msg.pose.position.z = target.position.z
        msg.pose.orientation.x = target.orientation.x
        msg.pose.orientation.y = target.orientation.y
        msg.pose.orientation.z = target.orientation.z
        msg.pose.orientation.w = target.orientation.w
        return msg

    def _make_marker_array(
        self,
        velocity: PlanarVelocity,
        *,
        target_tool_pose: PoseTarget,
        current_tool_pose: PoseTarget | None,
    ) -> MarkerArray:
        tip_base = transform_point(target_tool_pose, self.tool0_to_pen_tip)
        tool_base = target_tool_pose.position
        tip = self._base_to_paper_point(tip_base)
        tool = self._base_to_paper_point(tool_base)
        target_pen_tip_pose = PoseTarget(
            position=tip_base,
            orientation=target_tool_pose.orientation,
        )

        markers = [
            self._paper_marker(marker_id=0),
            self._paper_bounds_marker(marker_id=1),
            self._tip_marker(marker_id=2, tip=tip),
            self._axis_marker(marker_id=3, tip=tip, tail=tool),
            self._motion_marker(marker_id=4, tip=tip, velocity=velocity),
            self._tail_marker(marker_id=5, tail=tool),
            self._actual_tool_to_pen_tip_marker(
                marker_id=6,
                current_tool_pose=current_tool_pose,
            ),
            self._pose_axis_marker(
                marker_id=7,
                pose=target_pen_tip_pose,
                local_axis=Point3(x=1.0, y=0.0, z=0.0),
                axis_length_m=self.target_pen_tip_axis_length_m,
                namespace="target_pen_tip_x_axis",
                color=ColorRGBA(r=0.95, g=0.10, b=0.10, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=8,
                pose=target_pen_tip_pose,
                local_axis=Point3(x=0.0, y=1.0, z=0.0),
                axis_length_m=self.target_pen_tip_axis_length_m,
                namespace="target_pen_tip_y_axis",
                color=ColorRGBA(r=0.10, g=0.85, b=0.20, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=9,
                pose=target_pen_tip_pose,
                local_axis=Point3(x=0.0, y=0.0, z=1.0),
                axis_length_m=self.target_pen_tip_axis_length_m,
                namespace="target_pen_tip_z_axis",
                color=ColorRGBA(r=0.10, g=0.35, b=1.0, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=10,
                pose=target_tool_pose,
                local_axis=Point3(x=1.0, y=0.0, z=0.0),
                axis_length_m=self.target_tool0_axis_length_m,
                namespace="target_tool0_x_axis",
                color=ColorRGBA(r=1.0, g=0.10, b=0.85, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=11,
                pose=target_tool_pose,
                local_axis=Point3(x=0.0, y=1.0, z=0.0),
                axis_length_m=self.target_tool0_axis_length_m,
                namespace="target_tool0_y_axis",
                color=ColorRGBA(r=1.0, g=0.85, b=0.05, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=12,
                pose=target_tool_pose,
                local_axis=Point3(x=0.0, y=0.0, z=1.0),
                axis_length_m=self.target_tool0_axis_length_m,
                namespace="target_tool0_z_axis",
                color=ColorRGBA(r=0.95, g=0.95, b=1.0, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=13,
                pose=current_tool_pose,
                local_axis=Point3(x=1.0, y=0.0, z=0.0),
                axis_length_m=self.tool0_axis_length_m,
                namespace="actual_tool0_x_axis",
                color=ColorRGBA(r=0.95, g=0.10, b=0.10, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=14,
                pose=current_tool_pose,
                local_axis=Point3(x=0.0, y=1.0, z=0.0),
                axis_length_m=self.tool0_axis_length_m,
                namespace="actual_tool0_y_axis",
                color=ColorRGBA(r=0.10, g=0.85, b=0.20, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=15,
                pose=current_tool_pose,
                local_axis=Point3(x=0.0, y=0.0, z=1.0),
                axis_length_m=self.tool0_axis_length_m,
                namespace="actual_tool0_z_axis",
                color=ColorRGBA(r=0.10, g=0.35, b=1.0, a=1.0),
            ),
        ]
        return MarkerArray(markers=markers)

    def _base_marker(self, marker_id: int, marker_type: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.frames.paper_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.frame_locked = False
        return marker

    def _paper_marker(self, marker_id: int) -> Marker:
        marker = self._base_marker(marker_id, Marker.CUBE, "paper")
        marker.pose.position.z = -0.001
        marker.scale.x = self.paper_width_m
        marker.scale.y = self.paper_height_m
        marker.scale.z = 0.002
        marker.color = ColorRGBA(r=0.92, g=0.92, b=0.86, a=0.70)
        return marker

    def _paper_bounds_marker(self, marker_id: int) -> Marker:
        marker = self._base_marker(marker_id, Marker.LINE_STRIP, "paper_bounds")
        marker.scale.x = 0.003
        marker.color = ColorRGBA(r=0.12, g=0.12, b=0.12, a=1.0)
        half_w = self.paper_width_m / 2.0
        half_h = self.paper_height_m / 2.0
        marker.points = self._points(
            [
                (-half_w, -half_h, 0.002),
                (half_w, -half_h, 0.002),
                (half_w, half_h, 0.002),
                (-half_w, half_h, 0.002),
                (-half_w, -half_h, 0.002),
            ]
        )
        return marker

    def _tip_marker(self, marker_id: int, tip: Point3) -> Marker:
        marker = self._base_marker(marker_id, Marker.SPHERE, "pen_tip")
        marker.pose.position.x = tip.x
        marker.pose.position.y = tip.y
        marker.pose.position.z = tip.z + self.pen_tip_radius_m
        marker.scale.x = self.pen_tip_radius_m * 2.0
        marker.scale.y = self.pen_tip_radius_m * 2.0
        marker.scale.z = self.pen_tip_radius_m * 2.0
        marker.color = ColorRGBA(r=0.05, g=0.25, b=0.95, a=1.0)
        return marker

    def _tail_marker(self, marker_id: int, tail: Point3) -> Marker:
        marker = self._base_marker(marker_id, Marker.SPHERE, "pen_tail")
        marker.pose.position.x = tail.x
        marker.pose.position.y = tail.y
        marker.pose.position.z = tail.z
        marker.scale.x = self.pen_radius_m * 2.2
        marker.scale.y = self.pen_radius_m * 2.2
        marker.scale.z = self.pen_radius_m * 2.2
        marker.color = ColorRGBA(r=0.95, g=0.65, b=0.10, a=1.0)
        return marker

    def _axis_marker(self, marker_id: int, tip: Point3, tail: Point3) -> Marker:
        marker = self._base_marker(marker_id, Marker.ARROW, "pen_axis")
        marker.points = self._points(
            [
                (tail.x, tail.y, tail.z),
                (tip.x, tip.y, tip.z),
            ]
        )
        marker.scale.x = self.pen_radius_m
        marker.scale.y = self.pen_radius_m * 2.4
        marker.scale.z = self.pen_radius_m * 2.4
        marker.color = ColorRGBA(r=0.03, g=0.58, b=0.34, a=1.0)
        return marker

    def _motion_marker(
        self,
        marker_id: int,
        tip: Point3,
        velocity: PlanarVelocity,
    ) -> Marker:
        marker = self._base_marker(marker_id, Marker.ARROW, "motion_direction")
        speed = math.hypot(velocity.x, velocity.y)
        if speed < 1e-9:
            marker.action = Marker.DELETE
            return marker

        scale = min(0.08, 0.04 + speed)
        marker.points = self._points(
            [
                (tip.x, tip.y, 0.02),
                (
                    tip.x + velocity.x / speed * scale,
                    tip.y + velocity.y / speed * scale,
                    0.02,
                ),
            ]
        )
        marker.scale.x = 0.004
        marker.scale.y = 0.012
        marker.scale.z = 0.012
        marker.color = ColorRGBA(r=0.85, g=0.10, b=0.18, a=1.0)
        return marker

    def _actual_tool_to_pen_tip_marker(
        self,
        marker_id: int,
        current_tool_pose: PoseTarget | None,
    ) -> Marker:
        marker = self._base_marker(marker_id, Marker.ARROW, "actual_tool0_to_pen_tip")
        if current_tool_pose is None:
            marker.action = Marker.DELETE
            return marker

        start_base, end_base = tool_tail_to_tip_points(
            tool_pose=current_tool_pose,
            tool0_to_pen_tip=self.tool0_to_pen_tip,
        )
        start = self._base_to_paper_point(start_base)
        end = self._base_to_paper_point(end_base)
        marker.points = self._points(
            [
                (start.x, start.y, start.z),
                (end.x, end.y, end.z),
            ]
        )
        marker.scale.x = 0.003
        marker.scale.y = 0.010
        marker.scale.z = 0.010
        marker.color = ColorRGBA(r=0.05, g=0.85, b=0.95, a=1.0)
        return marker

    def _pose_axis_marker(
        self,
        *,
        marker_id: int,
        pose: PoseTarget | None,
        local_axis: Point3,
        axis_length_m: float,
        namespace: str,
        color: ColorRGBA,
    ) -> Marker:
        marker = self._base_marker(marker_id, Marker.ARROW, namespace)
        if pose is None:
            marker.action = Marker.DELETE
            return marker

        start_base, end_base = pose_axis_points(
            pose=pose,
            local_axis=local_axis,
            axis_length_m=axis_length_m,
        )
        start = self._base_to_paper_point(start_base)
        end = self._base_to_paper_point(end_base)
        marker.points = self._points(
            [
                (start.x, start.y, start.z),
                (end.x, end.y, end.z),
            ]
        )
        marker.scale.x = 0.004
        marker.scale.y = 0.010
        marker.scale.z = 0.010
        marker.color = color
        return marker

    def _configured_paper_origin(self) -> Point3:
        return Point3(
            x=self.paper_origin_xyz[0],
            y=self.paper_origin_xyz[1],
            z=self.paper_origin_xyz[2],
        )

    def _base_to_paper_point(self, point: Point3) -> Point3:
        return Point3(
            x=point.x - self._paper_origin.x,
            y=point.y - self._paper_origin.y,
            z=point.z - self._paper_origin.z,
        )

    def _warn_throttled(self, message: str) -> None:
        now_sec = time.monotonic()
        if now_sec - self._last_tf_warn_time >= self.tf_lookup_warn_period_sec:
            self._last_tf_warn_time = now_sec
            self.get_logger().warn(message)

    def destroy_node(self):
        if self._alignment_error_logger.started:
            self.get_logger().info(
                "Tool alignment error recording stopped after "
                f"{self._alignment_error_logger.sample_count} samples: "
                f"{self._alignment_error_logger.path}"
            )
        self._alignment_error_logger.close()
        return super().destroy_node()

    @staticmethod
    def _points(points: Iterable[tuple[float, float, float]]) -> list[Point]:
        result = []
        for x, y, z in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            result.append(point)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PenFakeHardwareServoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard interrupt received.")
    except (RuntimeError, ValueError) as exc:
        print(f"Pen fake-hardware Servo node refused to start: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
