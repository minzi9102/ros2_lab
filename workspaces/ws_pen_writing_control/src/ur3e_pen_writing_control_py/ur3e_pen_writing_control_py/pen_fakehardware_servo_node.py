import csv
import math
from pathlib import Path
import statistics
import time
import traceback
from dataclasses import dataclass
from typing import Iterable, TextIO

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    TransformStamped,
    TwistStamped,
    WrenchStamped,
)
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from ur_msgs.srv import SetPayload
from visualization_msgs.msg import Marker, MarkerArray

from .joy_mapping import JoyControl, JoyMapper
from .pen_math import (
    PaperBounds,
    PenPose2D,
    PlanarVelocity,
    SmoothPlanarVelocity,
    VirtualPenState,
)
from .pose_math import (
    ContinuousPenOrientation,
    Point3,
    PoseTarget,
    Quaternion,
    orientation_frame_from_pen_pose,
    quaternion_from_orientation_frame,
    rotate_vector,
    tool_pose_from_pen_tip_pose,
    transform_point,
)


PAPER_SEEK_REQUIRED_CONTROLLER = "joint_trajectory_controller"
PAPER_SEEK_INCOMPATIBLE_CONTROLLERS = (
    "passthrough_trajectory_controller",
    "force_mode_controller",
)
PAPER_SEEK_MIN_TF_PROGRESS_M = 0.00005


def paper_seek_controller_error(states: dict[str, str]) -> str | None:
    active_incompatible = tuple(
        name for name in PAPER_SEEK_INCOMPATIBLE_CONTROLLERS
        if states.get(name) == "active"
    )
    if states.get(PAPER_SEEK_REQUIRED_CONTROLLER) != "active":
        return f"{PAPER_SEEK_REQUIRED_CONTROLLER} is not active"
    if active_incompatible:
        return "incompatible controllers are active: " + ", ".join(active_incompatible)
    return None


def paper_seek_tf_progressed(
    *, previous_descent_m: float, actual_descent_m: float,
    minimum_progress_m: float = PAPER_SEEK_MIN_TF_PROGRESS_M,
) -> bool:
    return actual_descent_m >= previous_descent_m + minimum_progress_m


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


def initial_active_servo_command_mode(configured_mode: str) -> str:
    return "pose" if configured_mode == "twist_linear_only" else configured_mode


def should_publish_constant_linear_twist(
    *,
    configured_mode: str,
    command_armed: bool,
    servo_health_fault: bool,
) -> bool:
    return (
        configured_mode == "twist_constant_linear"
        and command_armed
        and not servo_health_fault
    )


def should_switch_linear_only_to_twist(
    *,
    configured_mode: str,
    active_mode: str,
    command_armed: bool,
    has_motion_intent: bool,
    virtual_pen_settling: bool,
    tool_pose_aligned: bool,
) -> bool:
    return (
        configured_mode == "twist_linear_only"
        and active_mode == "pose"
        and command_armed
        and not has_motion_intent
        and not virtual_pen_settling
        and tool_pose_aligned
    )


def target_orientation_for_command(
    *,
    configured_mode: str,
    diagnostic_orientation_mode: str,
    fixed_vertical_orientation: Quaternion,
    frozen_orientation: Quaternion | None,
    dynamic_orientation: Quaternion,
) -> Quaternion:
    if diagnostic_orientation_mode == "fixed_vertical":
        return fixed_vertical_orientation
    if configured_mode == "twist_linear_only" and frozen_orientation is not None:
        return frozen_orientation
    return dynamic_orientation


def fixed_vertical_pen_orientation(*, pen_length: float) -> Quaternion:
    return quaternion_from_orientation_frame(
        orientation_frame_from_pen_pose(
            pen_pose=PenPose2D(
                tip_x=0.0,
                tip_y=0.0,
                yaw=math.pi,
                tilt_rad=0.0,
            ),
            pen_length=pen_length,
        )
    )


def configured_initial_tip_xy(
    *,
    initial_tip_xy: list[float],
    initial_tip_x_m: float,
    initial_tip_y_m: float,
) -> list[float]:
    if math.isfinite(initial_tip_x_m) and math.isfinite(initial_tip_y_m):
        return [initial_tip_x_m, initial_tip_y_m]
    return initial_tip_xy


def pose_mode_became_ready(*, was_ready: bool, is_ready: bool) -> bool:
    return is_ready and not was_ready


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


def paper_seek_tool_pose_target(
    *,
    captured_tip_xy: tuple[float, float],
    target_tip_z: float,
    captured_tool_orientation: Quaternion,
    tool0_to_pen_tip: Point3,
) -> PoseTarget:
    """Build a seek target that only changes the pen-tip base Z coordinate."""
    tip_offset = rotate_tool_offset(
        captured_tool_orientation,
        tool0_to_pen_tip,
    )
    return PoseTarget(
        position=Point3(
            x=captured_tip_xy[0] - tip_offset.x,
            y=captured_tip_xy[1] - tip_offset.y,
            z=target_tip_z - tip_offset.z,
        ),
        orientation=captured_tool_orientation,
    )


def is_servo_status_fresh(
    *,
    status_seen: bool,
    last_status_time: float,
    now_sec: float,
    timeout_sec: float,
) -> bool:
    return status_seen and now_sec - last_status_time <= timeout_sec


def is_session_timed_out(
    *,
    max_session_duration_sec: float,
    session_started_at_sec: float,
    now_sec: float,
) -> bool:
    return (
        max_session_duration_sec > 0.0
        and now_sec - session_started_at_sec >= max_session_duration_sec
    )


PAPER_SEEK_IDLE = "IDLE"
PAPER_SEEK_ZEROING = "ZEROING"
PAPER_SEEK_BASELINING = "BASELINING"
PAPER_SEEK_DESCENDING = "DESCENDING"
PAPER_SEEK_RETRACTING = "RETRACTING"
PAPER_SEEK_SUCCEEDED = "SUCCEEDED"
PAPER_SEEK_CONTACT_FOUND = PAPER_SEEK_SUCCEEDED
PAPER_SEEK_ABORTED = "ABORTED"


def lowpass_force_z(
    *,
    previous_fz_n: float,
    sample_fz_n: float,
    alpha: float,
    initialized: bool,
) -> float:
    if not initialized:
        return sample_fz_n
    return previous_fz_n + alpha * (sample_fz_n - previous_fz_n)


def contact_force_from_baseline(
    *,
    filtered_fz_n: float,
    baseline_fz_n: float,
    force_axis_sign: float,
) -> float:
    return force_axis_sign * (filtered_fz_n - baseline_fz_n)


def next_paper_seek_offset(
    *,
    current_offset_m: float,
    down_speed_mps: float,
    dt_sec: float,
) -> float:
    return current_offset_m - down_speed_mps * max(dt_sec, 0.0)


def projected_force_z_in_base(
    *,
    force_xyz: tuple[float, float, float],
    source_orientation_in_base: Quaternion,
) -> float:
    return rotate_vector(source_orientation_in_base, force_xyz)[2]


def paper_seek_baseline_stats(samples: list[float]) -> tuple[float, float]:
    if not samples:
        raise ValueError("paper seek baseline needs at least one sample")
    mean = statistics.fmean(samples)
    standard_deviation = statistics.pstdev(samples) if len(samples) > 1 else 0.0
    return mean, standard_deviation


def paper_seek_dynamic_threshold(
    *,
    minimum_threshold_n: float,
    baseline_standard_deviation_n: float,
    sigma_multiplier: float,
) -> float:
    return max(
        minimum_threshold_n,
        baseline_standard_deviation_n * sigma_multiplier,
    )


@dataclass(frozen=True)
class ToolAlignmentError:
    position_m: float
    z_axis_rad: float
    full_quaternion_rad: float


@dataclass(frozen=True)
class CartesianTwist:
    linear: tuple[float, float, float]
    angular: tuple[float, float, float]


def _quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def quaternion_rotation_vector(
    current: Quaternion,
    target: Quaternion,
) -> tuple[float, float, float]:
    current_norm = math.sqrt(
        current.x**2 + current.y**2 + current.z**2 + current.w**2
    )
    target_norm = math.sqrt(target.x**2 + target.y**2 + target.z**2 + target.w**2)
    if current_norm < 1e-12 or target_norm < 1e-12:
        raise ValueError("orientation quaternion norm must be non-zero")
    current_unit = Quaternion(
        x=current.x / current_norm,
        y=current.y / current_norm,
        z=current.z / current_norm,
        w=current.w / current_norm,
    )
    target_unit = Quaternion(
        x=target.x / target_norm,
        y=target.y / target_norm,
        z=target.z / target_norm,
        w=target.w / target_norm,
    )
    relative = _quaternion_multiply(
        target_unit,
        Quaternion(
            x=-current_unit.x,
            y=-current_unit.y,
            z=-current_unit.z,
            w=current_unit.w,
        ),
    )
    if relative.w < 0.0:
        relative = Quaternion(
            x=-relative.x,
            y=-relative.y,
            z=-relative.z,
            w=-relative.w,
        )
    vector_norm = math.sqrt(relative.x**2 + relative.y**2 + relative.z**2)
    if vector_norm < 1e-12:
        return (0.0, 0.0, 0.0)
    angle = 2.0 * math.atan2(vector_norm, clamp(relative.w, 0.0, 1.0))
    scale = angle / vector_norm
    return (relative.x * scale, relative.y * scale, relative.z * scale)


def _limit_vector(
    vector: tuple[float, float, float],
    limit: float,
) -> tuple[float, float, float]:
    magnitude = math.sqrt(sum(component * component for component in vector))
    if magnitude <= limit or magnitude < 1e-12:
        return vector
    scale = limit / magnitude
    return tuple(component * scale for component in vector)


def twist_feedforward_command(
    *,
    previous_target: PoseTarget | None,
    target: PoseTarget,
    current: PoseTarget | None,
    dt_sec: float,
    position_gain: float,
    orientation_gain: float,
    linear_correction_limit_mps: float,
    angular_correction_limit_radps: float,
    angular_enabled: bool = True,
) -> CartesianTwist:
    if previous_target is None or dt_sec <= 0.0:
        linear_feedforward = (0.0, 0.0, 0.0)
        angular_feedforward = (0.0, 0.0, 0.0)
    else:
        linear_feedforward = (
            (target.position.x - previous_target.position.x) / dt_sec,
            (target.position.y - previous_target.position.y) / dt_sec,
            (target.position.z - previous_target.position.z) / dt_sec,
        )
        angular_feedforward = (
            tuple(
                component / dt_sec
                for component in quaternion_rotation_vector(
                    previous_target.orientation,
                    target.orientation,
                )
            )
            if angular_enabled
            else (0.0, 0.0, 0.0)
        )

    linear_correction = (0.0, 0.0, 0.0)
    angular_correction = (0.0, 0.0, 0.0)
    if current is not None:
        linear_correction = _limit_vector(
            (
                position_gain * (target.position.x - current.position.x),
                position_gain * (target.position.y - current.position.y),
                position_gain * (target.position.z - current.position.z),
            ),
            linear_correction_limit_mps,
        )
        if angular_enabled:
            angular_correction = _limit_vector(
                tuple(
                    orientation_gain * component
                    for component in quaternion_rotation_vector(
                        current.orientation,
                        target.orientation,
                    )
                ),
                angular_correction_limit_radps,
            )

    return CartesianTwist(
        linear=tuple(
            feedforward + correction
            for feedforward, correction in zip(
                linear_feedforward,
                linear_correction,
            )
        ),
        angular=tuple(
            feedforward + correction
            for feedforward, correction in zip(
                angular_feedforward,
                angular_correction,
            )
        ),
    )


def twist_constant_linear_command(velocity: PlanarVelocity) -> CartesianTwist:
    return CartesianTwist(
        linear=(velocity.x, velocity.y, 0.0),
        angular=(0.0, 0.0, 0.0),
    )


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
        self._stopped = False

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
        if self.path is None or self._stopped:
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

    def stop(self) -> None:
        self._stopped = True
        self.close()

    def close(self) -> None:
        if self._file is None:
            return
        self._file.close()
        self._file = None
        self._writer = None


class PenFakeHardwareServoNode(Node):
    """Bridge the stage-1 virtual pen model to MoveIt Servo."""

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
        self.twist_command_topic = str(
            self.declare_parameter(
                "twist_command_topic",
                "/servo_node/delta_twist_cmds",
            ).value
        )
        self.target_pose_topic = str(
            self.declare_parameter(
                "target_pose_topic",
                "/pen_writing/target_pose",
            ).value
        )
        self.servo_command_mode = str(
            self.declare_parameter("servo_command_mode", "pose").value
        )
        self.twist_position_gain = float(
            self.declare_parameter("twist_position_gain", 2.0).value
        )
        self.twist_orientation_gain = float(
            self.declare_parameter("twist_orientation_gain", 2.0).value
        )
        self.twist_linear_correction_limit_mps = float(
            self.declare_parameter(
                "twist_linear_correction_limit_mps",
                0.03,
            ).value
        )
        self.twist_angular_correction_limit_radps = float(
            self.declare_parameter(
                "twist_angular_correction_limit_radps",
                0.3,
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
        self.max_session_duration_sec = float(
            self.declare_parameter("max_session_duration_sec", 0.0).value
        )
        self.paper_seek_enabled = bool(
            self.declare_parameter("paper_seek_enabled", False).value
        )
        self.paper_seek_wrench_topic = str(
            self.declare_parameter(
                "paper_seek_wrench_topic",
                "/force_torque_sensor_broadcaster/wrench",
            ).value
        )
        self.paper_seek_baseline_duration_sec = float(
            self.declare_parameter("paper_seek_baseline_duration_sec", 1.0).value
        )
        self.paper_seek_down_speed_mps = float(
            self.declare_parameter("paper_seek_down_speed_mps", 0.0005).value
        )
        self.paper_seek_max_down_m = float(
            self.declare_parameter("paper_seek_max_down_m", 0.005).value
        )
        self.paper_seek_contact_threshold_n = float(
            self.declare_parameter("paper_seek_contact_threshold_n", 0.5).value
        )
        self.paper_seek_contact_confirm_samples = int(
            self.declare_parameter("paper_seek_contact_confirm_samples", 5).value
        )
        self.paper_seek_lowpass_alpha = float(
            self.declare_parameter("paper_seek_lowpass_alpha", 0.1).value
        )
        self.paper_seek_force_axis_sign = float(
            self.declare_parameter("paper_seek_force_axis_sign", 1.0).value
        )
        self.paper_seek_sigma_multiplier = float(
            self.declare_parameter("paper_seek_sigma_multiplier", 6.0).value
        )
        self.paper_seek_wrench_timeout_sec = float(
            self.declare_parameter("paper_seek_wrench_timeout_sec", 0.2).value
        )
        self.paper_seek_motion_timeout_sec = float(
            self.declare_parameter("paper_seek_motion_timeout_sec", 1.0).value
        )
        self.paper_seek_retract_distance_m = float(
            self.declare_parameter("paper_seek_retract_distance_m", 0.003).value
        )
        self.paper_seek_retract_timeout_sec = float(
            self.declare_parameter("paper_seek_retract_timeout_sec", 3.0).value
        )
        self.paper_seek_retract_tolerance_m = float(
            self.declare_parameter("paper_seek_retract_tolerance_m", 0.001).value
        )
        self.paper_seek_configure_payload = bool(
            self.declare_parameter("paper_seek_configure_payload", False).value
        )
        self.paper_seek_payload_mass_kg = float(
            self.declare_parameter("paper_seek_payload_mass_kg", -1.0).value
        )
        self.paper_seek_payload_cog_xyz = self._declare_float_list(
            "paper_seek_payload_cog_xyz",
            [0.0, 0.0, 0.0],
            expected_size=3,
        )
        self.paper_seek_zero_ft_before_start = bool(
            self.declare_parameter("paper_seek_zero_ft_before_start", False).value
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
        self.diagnostic_orientation_mode = str(
            self.declare_parameter("diagnostic_orientation_mode", "dynamic").value
        )
        self.fixed_tilt_deg = float(self.declare_parameter("fixed_tilt_deg", 20.0).value)
        self.diagnostic_freeze_tip_xy = bool(
            self.declare_parameter("diagnostic_freeze_tip_xy", False).value
        )
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
        initial_tip_xy = configured_initial_tip_xy(
            initial_tip_xy=initial_tip_xy,
            initial_tip_x_m=float(
                self.declare_parameter("initial_tip_x_m", math.nan).value
            ),
            initial_tip_y_m=float(
                self.declare_parameter("initial_tip_y_m", math.nan).value
            ),
        )

        self._validate_parameters()
        self._active_servo_command_mode = initial_active_servo_command_mode(
            self.servo_command_mode
        )
        self._linear_only_frozen_orientation: Quaternion | None = None
        self._fixed_vertical_orientation = fixed_vertical_pen_orientation(
            pen_length=self.pen_length_m
        )

        self._pose_publisher = self.create_publisher(
            PoseStamped,
            self.pose_command_topic,
            10,
        )
        self._twist_publisher = self.create_publisher(
            TwistStamped,
            self.twist_command_topic,
            10,
        )
        self._target_pose_publisher = self.create_publisher(
            PoseStamped,
            self.target_pose_topic,
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
        self._command_mode_ready = False
        self._pose_command_armed = not self.require_motion_before_pose_command
        self._paper_seek_payload_client = self.create_client(
            SetPayload,
            "/io_and_status_controller/set_payload",
        )
        self._paper_seek_zero_ft_client = self.create_client(
            Trigger,
            "/io_and_status_controller/zero_ftsensor",
        )
        self._paper_seek_list_controllers_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )
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
        self._paper_seek_wrench_subscription = (
            self.create_subscription(
                WrenchStamped,
                self.paper_seek_wrench_topic,
                self._on_paper_seek_wrench,
                10,
            )
            if self.paper_seek_enabled
            else None
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._paper_seek_status_publisher = self.create_publisher(
            String,
            "/pen_writing/paper_seek_status",
            latched_qos,
        )
        self._paper_seek_point_publisher = self.create_publisher(
            PointStamped,
            "/pen_writing/detected_paper_point",
            latched_qos,
        )
        self._joy_mapper = JoyMapper(deadzone=self.joy_deadzone)
        self._latest_joy_control = JoyControl()
        self._last_joy_msg_time = 0.0
        self._last_servo_status_time = 0.0
        self._last_servo_status_warn_time = 0.0
        self._servo_status_seen = False
        self._servo_health_fault = False
        self._shutdown_reason = "not requested"
        self._shutdown_requested_at_sec: float | None = None
        self._alignment_error_logger = AlignmentErrorCsvLogger(
            path=self.alignment_error_log_path,
            sample_rate_hz=self.alignment_error_log_rate_hz,
        )
        self.create_service(
            Trigger,
            "/pen_writing/stop_alignment_logging",
            self._on_stop_alignment_logging,
        )
        self.create_service(
            Trigger,
            "/pen_writing/start_paper_seek",
            self._on_start_paper_seek,
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
            freeze_tip_xy=self.diagnostic_freeze_tip_xy,
        )
        self._pen_orientation = ContinuousPenOrientation(
            initial_pen_pose=self._pen_state.pose,
            pen_length=self.pen_length_m,
            max_axis_angular_speed_radps=math.radians(
                self.max_pen_axis_angular_speed_degps
            ),
        )
        self._session_started_at_sec = time.monotonic()
        self._last_timer_time = time.monotonic()
        self._previous_target_pose: PoseTarget | None = None
        self._paper_seek_state = PAPER_SEEK_IDLE
        self._paper_seek_started_at_sec = 0.0
        self._paper_seek_offset_m = 0.0
        self._paper_seek_start_tip_z = 0.0
        self._paper_seek_captured_tip_xy = (0.0, 0.0)
        self._paper_seek_captured_tool_orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0,
        )
        self._paper_seek_baseline_fz_n = 0.0
        self._paper_seek_contact_threshold_active_n = (
            self.paper_seek_contact_threshold_n
        )
        self._paper_seek_baseline_samples: list[float] = []
        self._paper_seek_contact_count = 0
        self._paper_seek_detected_z: float | None = None
        self._paper_seek_candidate_z: float | None = None
        self._paper_seek_candidate_point: Point3 | None = None
        self._paper_seek_retract_target_z: float | None = None
        self._paper_seek_retract_started_at_sec = 0.0
        self._paper_seek_last_progress_status_sec = 0.0
        self._paper_seek_last_actual_descent_m = 0.0
        self._paper_seek_last_actual_progress_at_sec = 0.0
        self._paper_seek_wrench_seen = False
        self._paper_seek_wrench_sequence = 0
        self._paper_seek_last_evaluated_wrench_sequence = 0
        self._paper_seek_last_wrench_time = 0.0
        self._paper_seek_filter_initialized = False
        self._paper_seek_filtered_fz_n = 0.0
        self._publish_paper_seek_status("paper seek idle")

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "Pen fake-hardware Servo node started. "
            f"base_frame={self.frames.base_frame} tool_frame={self.frames.tool_frame} "
            f"pose_topic={self.pose_command_topic} twist_topic={self.twist_command_topic} "
            f"target_pose_topic={self.target_pose_topic} marker_topic={self.marker_topic} "
            f"servo_status_topic={self.servo_status_topic} joy_topic={self.joy_topic} "
            f"rate={self.publish_rate_hz:.1f}Hz "
            f"servo_command_mode={self.servo_command_mode} "
            f"active_servo_command_mode={self._active_servo_command_mode} "
            f"require_motion_before_pose_command={self.require_motion_before_pose_command} "
            f"tool_position_tolerance={self.tool_position_tolerance_m:.3f}m "
            f"tool_orientation_tolerance={self.tool_orientation_tolerance_deg:.1f}deg "
            f"max_pen_axis_rate="
            f"{self.max_pen_axis_angular_speed_degps:.1f}deg/s "
            f"fixed_tilt={self.fixed_tilt_deg:.1f}deg "
            f"diagnostic_orientation_mode={self.diagnostic_orientation_mode} "
            f"diagnostic_freeze_tip_xy={self.diagnostic_freeze_tip_xy} "
            f"max_session_duration={self.max_session_duration_sec:.1f}s "
            f"paper_seek_enabled={self.paper_seek_enabled} "
            f"paper_seek_wrench_topic={self.paper_seek_wrench_topic} "
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
        if self.servo_command_mode not in (
            "pose",
            "twist_feedforward",
            "twist_linear_only",
            "twist_constant_linear",
        ):
            raise ValueError(
                "servo_command_mode must be 'pose', 'twist_feedforward', "
                "'twist_linear_only', or 'twist_constant_linear'"
            )
        if self.diagnostic_orientation_mode not in ("dynamic", "fixed_vertical"):
            raise ValueError(
                "diagnostic_orientation_mode must be 'dynamic' or "
                "'fixed_vertical'"
            )
        if self.twist_position_gain < 0.0:
            raise ValueError("twist_position_gain must be non-negative")
        if self.twist_orientation_gain < 0.0:
            raise ValueError("twist_orientation_gain must be non-negative")
        if self.twist_linear_correction_limit_mps <= 0.0:
            raise ValueError(
                "twist_linear_correction_limit_mps must be greater than zero"
            )
        if self.twist_angular_correction_limit_radps <= 0.0:
            raise ValueError(
                "twist_angular_correction_limit_radps must be greater than zero"
            )
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
        if self.max_session_duration_sec < 0.0:
            raise ValueError("max_session_duration_sec must be non-negative")
        if self.paper_seek_baseline_duration_sec <= 0.0:
            raise ValueError(
                "paper_seek_baseline_duration_sec must be greater than zero"
            )
        if self.paper_seek_down_speed_mps <= 0.0:
            raise ValueError("paper_seek_down_speed_mps must be greater than zero")
        if self.paper_seek_down_speed_mps > 0.001:
            raise ValueError("paper_seek_down_speed_mps must be <= 0.001")
        if self.paper_seek_max_down_m <= 0.0:
            raise ValueError("paper_seek_max_down_m must be greater than zero")
        if self.paper_seek_contact_threshold_n <= 0.0:
            raise ValueError("paper_seek_contact_threshold_n must be greater than zero")
        if self.paper_seek_contact_confirm_samples <= 0:
            raise ValueError("paper_seek_contact_confirm_samples must be positive")
        if not 0.0 < self.paper_seek_lowpass_alpha <= 1.0:
            raise ValueError("paper_seek_lowpass_alpha must be in (0, 1]")
        if self.paper_seek_force_axis_sign == 0.0:
            raise ValueError("paper_seek_force_axis_sign must be non-zero")
        if self.paper_seek_sigma_multiplier <= 0.0:
            raise ValueError("paper_seek_sigma_multiplier must be greater than zero")
        if self.paper_seek_wrench_timeout_sec <= 0.0:
            raise ValueError("paper_seek_wrench_timeout_sec must be greater than zero")
        if not 0.2 <= self.paper_seek_motion_timeout_sec <= 2.0:
            raise ValueError("paper_seek_motion_timeout_sec must be in [0.2, 2.0]")
        if not 0.0 < self.paper_seek_retract_distance_m <= 0.003:
            raise ValueError("paper_seek_retract_distance_m must be in (0, 0.003]")
        if self.paper_seek_retract_timeout_sec <= 0.0:
            raise ValueError("paper_seek_retract_timeout_sec must be greater than zero")
        if self.paper_seek_retract_tolerance_m <= 0.0:
            raise ValueError("paper_seek_retract_tolerance_m must be greater than zero")
        if self.paper_seek_payload_mass_kg > 3.0:
            raise ValueError("paper_seek_payload_mass_kg must be <= 3.0")
        if any(abs(value) > 0.5 for value in self.paper_seek_payload_cog_xyz):
            raise ValueError("paper_seek_payload_cog_xyz components must be <= 0.5m")
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

    def _on_paper_seek_wrench(self, msg: WrenchStamped) -> None:
        source_frame = msg.header.frame_id or self.frames.tool_frame
        source_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        if source_frame != self.frames.base_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.frames.base_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
            except TransformException as exc:
                self._warn_throttled(
                    f"Waiting to transform paper-seek wrench from "
                    f"{source_frame} to {self.frames.base_frame}: {exc}"
                )
                return
            rotation = transform.transform.rotation
            source_orientation = Quaternion(
                x=rotation.x,
                y=rotation.y,
                z=rotation.z,
                w=rotation.w,
            )
        projected_force = projected_force_z_in_base(
            force_xyz=(
                float(msg.wrench.force.x),
                float(msg.wrench.force.y),
                float(msg.wrench.force.z),
            ),
            source_orientation_in_base=source_orientation,
        )
        self._paper_seek_filtered_fz_n = lowpass_force_z(
            previous_fz_n=self._paper_seek_filtered_fz_n,
            sample_fz_n=projected_force,
            alpha=self.paper_seek_lowpass_alpha,
            initialized=self._paper_seek_filter_initialized,
        )
        self._paper_seek_filter_initialized = True
        self._paper_seek_wrench_seen = True
        self._paper_seek_last_wrench_time = time.monotonic()
        self._paper_seek_wrench_sequence += 1
        if self._paper_seek_state == PAPER_SEEK_BASELINING:
            self._paper_seek_baseline_samples.append(
                self._paper_seek_filtered_fz_n
            )

    def _on_start_paper_seek(self, _request, response):
        now_sec = time.monotonic()
        if not self.paper_seek_enabled:
            response.success = False
            response.message = "paper seek is disabled"
            return response
        if self._paper_seek_state in (
            PAPER_SEEK_ZEROING,
            PAPER_SEEK_BASELINING,
            PAPER_SEEK_DESCENDING,
            PAPER_SEEK_RETRACTING,
        ):
            response.success = False
            response.message = f"paper seek already running: {self._paper_seek_state}"
            return response
        if self._paper_origin is None:
            response.success = False
            response.message = "paper origin is not initialized"
            return response
        if not self._paper_seek_wrench_seen:
            response.success = False
            response.message = f"no wrench received on {self.paper_seek_wrench_topic}"
            return response
        if (
            now_sec - self._paper_seek_last_wrench_time
            > self.paper_seek_wrench_timeout_sec
        ):
            response.success = False
            response.message = "paper seek wrench is stale"
            return response
        if not self._is_servo_status_healthy(now_sec):
            response.success = False
            response.message = "MoveIt Servo status is not healthy"
            return response
        if not self._paper_seek_list_controllers_client.service_is_ready():
            response.success = False
            response.message = "list_controllers service is unavailable"
            return response
        if self.paper_seek_configure_payload:
            if self.paper_seek_payload_mass_kg < 0.0:
                response.success = False
                response.message = "paper seek payload mass is not configured"
                return response
            if not self._paper_seek_payload_client.service_is_ready():
                response.success = False
                response.message = "set_payload service is unavailable"
                return response
        if (
            self.paper_seek_zero_ft_before_start
            and not self._paper_seek_zero_ft_client.service_is_ready()
        ):
            response.success = False
            response.message = "zero_ftsensor service is unavailable"
            return response

        current_tool_pose = self._lookup_current_tool_pose()
        if current_tool_pose is None:
            response.success = False
            response.message = "current tool pose is not available"
            return response
        current_tip = tool_tip_point_from_tool_pose(
            tool_pose=current_tool_pose,
            tool0_to_pen_tip=self.tool0_to_pen_tip,
        )

        self._paper_seek_state = PAPER_SEEK_ZEROING
        self._paper_seek_started_at_sec = now_sec
        self._paper_seek_start_tip_z = current_tip.z
        self._paper_seek_captured_tip_xy = (current_tip.x, current_tip.y)
        self._paper_seek_captured_tool_orientation = current_tool_pose.orientation
        self._paper_seek_offset_m = 0.0
        self._paper_seek_baseline_fz_n = self._paper_seek_filtered_fz_n
        self._paper_seek_baseline_samples = []
        self._paper_seek_contact_threshold_active_n = (
            self.paper_seek_contact_threshold_n
        )
        self._paper_seek_contact_count = 0
        self._paper_seek_detected_z = None
        self._paper_seek_candidate_z = None
        self._paper_seek_candidate_point = None
        self._paper_seek_retract_target_z = None
        self._paper_seek_last_evaluated_wrench_sequence = (
            self._paper_seek_wrench_sequence
        )
        self._pose_command_armed = True
        self._previous_target_pose = None
        response.success = True
        response.message = (
            "paper seek preparation started: "
            f"start_tip_z={self._paper_seek_start_tip_z:.6f}"
        )
        self._publish_paper_seek_status(response.message)
        self._prepare_paper_seek()
        return response

    def _prepare_paper_seek(self) -> None:
        def begin_zeroing() -> None:
            if self._paper_seek_state != PAPER_SEEK_ZEROING:
                return
            if not self.paper_seek_zero_ft_before_start:
                self._begin_paper_seek_baseline()
                return
            future = self._paper_seek_zero_ft_client.call_async(Trigger.Request())

            def zeroed(done_future) -> None:
                result = done_future.result()
                if self._paper_seek_state != PAPER_SEEK_ZEROING:
                    return
                if result is None or not result.success:
                    self._abort_paper_seek("zero_ftsensor failed")
                    return
                self._begin_paper_seek_baseline()

            future.add_done_callback(zeroed)

        def configure_payload() -> None:
            if not self.paper_seek_configure_payload:
                begin_zeroing()
                return
            request = SetPayload.Request()
            request.mass = self.paper_seek_payload_mass_kg
            request.center_of_gravity.x = self.paper_seek_payload_cog_xyz[0]
            request.center_of_gravity.y = self.paper_seek_payload_cog_xyz[1]
            request.center_of_gravity.z = self.paper_seek_payload_cog_xyz[2]
            future = self._paper_seek_payload_client.call_async(request)

            def payload_set(done_future) -> None:
                result = done_future.result()
                if self._paper_seek_state != PAPER_SEEK_ZEROING:
                    return
                if result is None or not result.success:
                    self._abort_paper_seek("set_payload failed")
                    return
                begin_zeroing()

            future.add_done_callback(payload_set)

        future = self._paper_seek_list_controllers_client.call_async(
            ListControllers.Request()
        )

        def controllers_listed(done_future) -> None:
            try:
                result = done_future.result()
            except Exception as exc:
                self._abort_paper_seek(f"list_controllers failed: {exc}")
                return
            if self._paper_seek_state != PAPER_SEEK_ZEROING:
                return
            if result is None:
                self._abort_paper_seek("list_controllers returned no response")
                return
            states = {
                controller.name: controller.state for controller in result.controller
            }
            error = paper_seek_controller_error(states)
            if error:
                self._abort_paper_seek(f"controller precheck failed: {error}")
                return
            configure_payload()

        future.add_done_callback(controllers_listed)

    def _begin_paper_seek_baseline(self) -> None:
        self._paper_seek_state = PAPER_SEEK_BASELINING
        self._paper_seek_started_at_sec = time.monotonic()
        self._paper_seek_baseline_samples = []
        self._paper_seek_filter_initialized = False
        self._paper_seek_wrench_seen = False
        self._paper_seek_last_wrench_time = 0.0
        self._publish_paper_seek_status("preparation complete; collecting baseline")

    def _on_timer(self) -> None:
        now_sec = time.monotonic()
        if is_session_timed_out(
            max_session_duration_sec=self.max_session_duration_sec,
            session_started_at_sec=self._session_started_at_sec,
            now_sec=now_sec,
        ):
            if self._paper_seek_active():
                self._abort_paper_seek("maximum session duration reached")
            self._velocity.stop_immediately()
            self._pose_command_armed = False
            self._publish_zero_twist()
            self._request_shutdown(
                "maximum session duration reached: "
                f"elapsed={now_sec - self._session_started_at_sec:.3f}s "
                f"limit={self.max_session_duration_sec:.3f}s",
                level="warn",
            )
            return

        if self._paper_origin is None:
            self._initialize_paper_origin()
            return
        was_command_mode_ready = self._command_mode_ready
        if not self._ensure_command_mode_ready():
            return
        if pose_mode_became_ready(
            was_ready=was_command_mode_ready,
            is_ready=self._command_mode_ready,
        ):
            return

        dt_sec = now_sec - self._last_timer_time
        self._last_timer_time = now_sec

        control = self._current_control(now_sec)
        has_motion_intent = has_planar_motion_intent(control)
        if has_motion_intent and not self._pose_command_armed:
            self._pose_command_armed = True
            self.get_logger().info("Motion input received. Arming Servo commands.")

        if self._pose_command_armed and not self._is_servo_status_healthy(now_sec):
            if self._paper_seek_active():
                self._abort_paper_seek("MoveIt Servo status timed out")
            self._servo_health_fault = True
            self._pose_command_armed = False
            self._publish_zero_twist()
            self._request_shutdown(
                "MoveIt Servo status timed out after commands were armed: "
                f"last_status_age={now_sec - self._last_servo_status_time:.3f}s "
                f"timeout={self.servo_status_timeout_sec:.3f}s",
                level="error",
            )
            return

        if control.emergency_stop:
            velocity = self._velocity.stop_immediately()
            self._pose_command_armed = False
            if self._paper_seek_state in (
                PAPER_SEEK_ZEROING,
                PAPER_SEEK_BASELINING,
                PAPER_SEEK_DESCENDING,
                PAPER_SEEK_RETRACTING,
            ):
                self._abort_paper_seek("emergency stop requested")
            self._publish_zero_twist()
            target_tool_pose = self._make_tool_pose_target()
            current_tool_pose = self._lookup_current_tool_pose()
            self._publish_tf_markers_and_pose(
                velocity,
                target_tool_pose=target_tool_pose,
                current_tool_pose=current_tool_pose,
                publish_pose=False,
                dt_sec=dt_sec,
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
                self._request_shutdown(
                    "Joy B quit requested while emergency stop branch was active",
                )
            return

        if self._paper_seek_active():
            self._handle_paper_seek_timer(now_sec=now_sec, dt_sec=dt_sec)
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
        switching_to_linear_only = should_switch_linear_only_to_twist(
            configured_mode=self.servo_command_mode,
            active_mode=self._active_servo_command_mode,
            command_armed=self._pose_command_armed,
            has_motion_intent=has_motion_intent,
            virtual_pen_settling=virtual_pen_settling,
            tool_pose_aligned=tool_pose_aligned,
        )
        if switching_to_linear_only:
            self._linear_only_frozen_orientation = target_tool_pose.orientation
            self._active_servo_command_mode = "twist_feedforward"
            self._command_mode_ready = False
            self._command_type_future = None
            self.get_logger().info(
                "Initial POSE alignment complete. Switching to linear-only TWIST."
            )

        publish_pose = not switching_to_linear_only and (
            should_publish_pose_command(
                pose_command_armed=self._pose_command_armed,
                has_motion_intent=has_motion_intent,
                virtual_pen_settling=virtual_pen_settling,
                tool_pose_aligned=tool_pose_aligned,
                servo_health_fault=self._servo_health_fault,
            )
            or should_publish_constant_linear_twist(
                configured_mode=self.servo_command_mode,
                command_armed=self._pose_command_armed,
                servo_health_fault=self._servo_health_fault,
            )
        )
        self._publish_tf_markers_and_pose(
            velocity,
            target_tool_pose=target_tool_pose,
            current_tool_pose=current_tool_pose,
            publish_pose=publish_pose,
            dt_sec=dt_sec,
        )
        self._record_alignment_error(
            now_sec=now_sec,
            target_tool_pose=target_tool_pose,
            current_tool_pose=current_tool_pose,
            pose_command_published=publish_pose,
            has_motion_intent=has_motion_intent,
            virtual_pen_settling=(
                virtual_pen_settling or switching_to_linear_only
            ),
        )

        if control.quit_requested:
            self._request_shutdown("Joy B quit requested after normal update branch")

    def _request_shutdown(self, reason: str, *, level: str = "info") -> None:
        self._publish_zero_twist()
        now_sec = time.monotonic()
        if self._shutdown_requested_at_sec is None:
            self._shutdown_reason = reason
            self._shutdown_requested_at_sec = now_sec

        joy_age = (
            math.inf
            if self._last_joy_msg_time == 0.0
            else now_sec - self._last_joy_msg_time
        )
        status_age = (
            math.inf
            if self._last_servo_status_time == 0.0
            else now_sec - self._last_servo_status_time
        )
        message = (
            "Pen Servo node requesting shutdown. "
            f"reason={self._shutdown_reason!r} "
            f"pose_command_armed={self._pose_command_armed} "
            f"command_mode_ready={self._command_mode_ready} "
            f"servo_status_seen={self._servo_status_seen} "
            f"servo_status_age_sec={status_age:.3f} "
            f"joy_msg_age_sec={joy_age:.3f} "
            f"latest_control={self._latest_joy_control}"
        )
        if level == "error":
            self.get_logger().error(message)
        elif level == "warn":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)
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

    def _ensure_command_mode_ready(self) -> bool:
        if self._command_mode_ready:
            return True
        if not self._command_type_client.service_is_ready():
            self._command_type_client.wait_for_service(timeout_sec=0.0)
            return False
        if self._command_type_future is None:
            request = ServoCommandType.Request()
            request.command_type = (
                ServoCommandType.Request.POSE
                if self._active_servo_command_mode == "pose"
                else ServoCommandType.Request.TWIST
            )
            self._command_type_future = self._command_type_client.call_async(request)
            self.get_logger().info(
                "Requested MoveIt Servo "
                f"{self._active_servo_command_mode} command mode."
            )
            return False
        if not self._command_type_future.done():
            return False

        response = self._command_type_future.result()
        self._command_type_future = None
        if response is not None and response.success:
            self._command_mode_ready = True
            self._last_timer_time = time.monotonic()
            self.get_logger().info(
                "MoveIt Servo accepted "
                f"{self._active_servo_command_mode} command mode."
            )
            return True

        self.get_logger().warn(
            "MoveIt Servo rejected "
            f"{self._active_servo_command_mode} command mode; retrying."
        )
        return False

    def _is_servo_status_healthy(self, now_sec: float) -> bool:
        return is_servo_status_fresh(
            status_seen=self._servo_status_seen,
            last_status_time=self._last_servo_status_time,
            now_sec=now_sec,
            timeout_sec=self.servo_status_timeout_sec,
        )

    def _handle_paper_seek_timer(self, *, now_sec: float, dt_sec: float) -> None:
        velocity = self._velocity.stop_immediately()
        self._pose_command_armed = True

        if not self._is_servo_status_healthy(now_sec):
            self._abort_paper_seek("MoveIt Servo status timed out")
        elif self._paper_seek_state == PAPER_SEEK_ZEROING:
            pass
        elif (
            self._paper_seek_wrench_seen
            and
            now_sec - self._paper_seek_last_wrench_time
            > self.paper_seek_wrench_timeout_sec
        ):
            self._abort_paper_seek("wrench data timed out")
        elif self._paper_seek_state == PAPER_SEEK_BASELINING:
            if (
                now_sec - self._paper_seek_started_at_sec
                >= self.paper_seek_baseline_duration_sec
            ):
                if not self._paper_seek_baseline_samples:
                    self._abort_paper_seek("no baseline wrench samples")
                    return
                baseline_mean, baseline_stddev = paper_seek_baseline_stats(
                    self._paper_seek_baseline_samples
                )
                self._paper_seek_baseline_fz_n = baseline_mean
                self._paper_seek_contact_threshold_active_n = (
                    paper_seek_dynamic_threshold(
                        minimum_threshold_n=self.paper_seek_contact_threshold_n,
                        baseline_standard_deviation_n=baseline_stddev,
                        sigma_multiplier=self.paper_seek_sigma_multiplier,
                    )
                )
                self._paper_seek_state = PAPER_SEEK_DESCENDING
                self._paper_seek_last_actual_descent_m = 0.0
                self._paper_seek_last_actual_progress_at_sec = now_sec
                self._previous_target_pose = None
                self._publish_paper_seek_status(
                    "baseline captured: "
                    f"mean={baseline_mean:.3f}N stddev={baseline_stddev:.3f}N "
                    f"threshold={self._paper_seek_contact_threshold_active_n:.3f}N"
                )
        elif self._paper_seek_state == PAPER_SEEK_DESCENDING:
            current_tool_pose = self._lookup_current_tool_pose()
            if current_tool_pose is None:
                self._abort_paper_seek("current tool pose is unavailable")
                return
            current_tip = tool_tip_point_from_tool_pose(
                tool_pose=current_tool_pose,
                tool0_to_pen_tip=self.tool0_to_pen_tip,
            )
            actual_descent = self._paper_seek_start_tip_z - current_tip.z
            if paper_seek_tf_progressed(
                previous_descent_m=self._paper_seek_last_actual_descent_m,
                actual_descent_m=actual_descent,
            ):
                self._paper_seek_last_actual_descent_m = actual_descent
                self._paper_seek_last_actual_progress_at_sec = now_sec
            elif (
                now_sec - self._paper_seek_last_actual_progress_at_sec
                > self.paper_seek_motion_timeout_sec
            ):
                self._abort_paper_seek(
                    "actual TF descent stalled: "
                    f"commanded={self._paper_seek_offset_m:.6f}m "
                    f"actual={-actual_descent:.6f}m"
                )
                return
            next_offset = next_paper_seek_offset(
                current_offset_m=self._paper_seek_offset_m,
                down_speed_mps=self.paper_seek_down_speed_mps,
                dt_sec=dt_sec,
            )
            if abs(next_offset) > self.paper_seek_max_down_m:
                self._abort_paper_seek(
                    "maximum descent reached "
                    f"offset={next_offset:.6f}m "
                    f"limit={self.paper_seek_max_down_m:.6f}m"
                )
            else:
                self._paper_seek_offset_m = next_offset
                contact_force_n = contact_force_from_baseline(
                    filtered_fz_n=self._paper_seek_filtered_fz_n,
                    baseline_fz_n=self._paper_seek_baseline_fz_n,
                    force_axis_sign=self.paper_seek_force_axis_sign,
                )
                if now_sec - self._paper_seek_last_progress_status_sec >= 1.0:
                    self._paper_seek_last_progress_status_sec = now_sec
                    self._publish_paper_seek_status(
                        f"offset={self._paper_seek_offset_m:.6f}m "
                        f"force={contact_force_n:.3f}N "
                        f"threshold={self._paper_seek_contact_threshold_active_n:.3f}N "
                        f"confirm={self._paper_seek_contact_count}/"
                        f"{self.paper_seek_contact_confirm_samples}"
                    )
                new_wrench_sample = (
                    self._paper_seek_wrench_sequence
                    != self._paper_seek_last_evaluated_wrench_sequence
                )
                if new_wrench_sample:
                    self._paper_seek_last_evaluated_wrench_sequence = (
                        self._paper_seek_wrench_sequence
                    )
                    if (
                        contact_force_n
                        >= self._paper_seek_contact_threshold_active_n
                    ):
                        self._paper_seek_contact_count += 1
                    else:
                        self._paper_seek_contact_count = 0

                if new_wrench_sample and (
                    self._paper_seek_contact_count
                    >= self.paper_seek_contact_confirm_samples
                ):
                    self._paper_seek_candidate_z = current_tip.z
                    self._paper_seek_candidate_point = current_tip
                    self._paper_seek_retract_target_z = (
                        current_tip.z + self.paper_seek_retract_distance_m
                    )
                    self._paper_seek_retract_started_at_sec = now_sec
                    self._paper_seek_state = PAPER_SEEK_RETRACTING
                    self._previous_target_pose = None
                    self._publish_paper_seek_status(
                        "contact confirmed from actual TF: "
                        f"candidate_z={current_tip.z:.6f} "
                        f"contact_force={contact_force_n:.3f}N"
                    )
        elif self._paper_seek_state == PAPER_SEEK_RETRACTING:
            assert self._paper_seek_retract_target_z is not None
            assert self._paper_seek_candidate_z is not None
            current_tool_pose = self._lookup_current_tool_pose()
            if current_tool_pose is None:
                self._abort_paper_seek("current tool pose is unavailable during retract")
            else:
                current_tip = tool_tip_point_from_tool_pose(
                    tool_pose=current_tool_pose,
                    tool0_to_pen_tip=self.tool0_to_pen_tip,
                )
                if (
                    abs(current_tip.z - self._paper_seek_retract_target_z)
                    <= self.paper_seek_retract_tolerance_m
                ):
                    self._paper_origin = Point3(
                        x=self._paper_origin.x,
                        y=self._paper_origin.y,
                        z=self._paper_seek_candidate_z,
                    )
                    self._paper_seek_detected_z = self._paper_seek_candidate_z
                    self._paper_seek_state = PAPER_SEEK_SUCCEEDED
                    self._pose_command_armed = False
                    self._publish_zero_twist()
                    self._publish_detected_paper_point()
                    self._publish_paper_seek_status(
                        f"paper height committed: z={self._paper_seek_detected_z:.6f}"
                    )
                elif (
                    now_sec - self._paper_seek_retract_started_at_sec
                    > self.paper_seek_retract_timeout_sec
                ):
                    self._abort_paper_seek("retraction timed out")

        target_tool_pose = self._make_tool_pose_target()
        current_tool_pose = self._lookup_current_tool_pose()
        publish_pose = self._paper_seek_state in (
            PAPER_SEEK_ZEROING,
            PAPER_SEEK_BASELINING,
            PAPER_SEEK_DESCENDING,
            PAPER_SEEK_RETRACTING,
        )
        self._publish_tf_markers_and_pose(
            velocity,
            target_tool_pose=target_tool_pose,
            current_tool_pose=current_tool_pose,
            publish_pose=publish_pose,
            dt_sec=dt_sec,
        )
        self._record_alignment_error(
            now_sec=now_sec,
            target_tool_pose=target_tool_pose,
            current_tool_pose=current_tool_pose,
            pose_command_published=publish_pose,
            has_motion_intent=False,
            virtual_pen_settling=False,
        )

    def _paper_seek_active(self) -> bool:
        return self._paper_seek_state in (
            PAPER_SEEK_ZEROING,
            PAPER_SEEK_BASELINING,
            PAPER_SEEK_DESCENDING,
            PAPER_SEEK_RETRACTING,
        )

    def _abort_paper_seek(self, reason: str) -> None:
        self._paper_seek_state = PAPER_SEEK_ABORTED
        self._paper_seek_candidate_z = None
        self._paper_seek_candidate_point = None
        self._paper_seek_retract_target_z = None
        self._pose_command_armed = False
        self._publish_zero_twist()
        self._publish_paper_seek_status(reason, error=True)

    def _publish_paper_seek_status(self, detail: str, *, error: bool = False) -> None:
        message = f"{self._paper_seek_state}: {detail}"
        self._paper_seek_status_publisher.publish(String(data=message))
        if error:
            self.get_logger().error(f"Paper seek {message}")
        else:
            self.get_logger().info(f"Paper seek {message}")

    def _publish_detected_paper_point(self) -> None:
        assert self._paper_seek_candidate_point is not None
        point = PointStamped()
        point.header.frame_id = self.frames.base_frame
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = self._paper_seek_candidate_point.x
        point.point.y = self._paper_seek_candidate_point.y
        point.point.z = self._paper_seek_candidate_point.z
        self._paper_seek_point_publisher.publish(point)

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
        if self._paper_seek_active():
            return paper_seek_tool_pose_target(
                captured_tip_xy=self._paper_seek_captured_tip_xy,
                target_tip_z=self._paper_seek_target_tip_z(),
                captured_tool_orientation=(
                    self._paper_seek_captured_tool_orientation
                ),
                tool0_to_pen_tip=self.tool0_to_pen_tip,
            )
        return tool_pose_from_pen_tip_pose(
            pen_pose=self._pen_state.pose,
            paper_origin=self._target_paper_origin(),
            pen_length=self.pen_length_m,
            tool0_to_pen_tip_xyz=self.tool0_to_pen_tip,
            orientation_override=target_orientation_for_command(
                configured_mode=self.servo_command_mode,
                diagnostic_orientation_mode=self.diagnostic_orientation_mode,
                fixed_vertical_orientation=self._fixed_vertical_orientation,
                frozen_orientation=self._linear_only_frozen_orientation,
                dynamic_orientation=self._pen_orientation.orientation,
            ),
        )

    def _paper_seek_target_tip_z(self) -> float:
        if self._paper_seek_state == PAPER_SEEK_RETRACTING:
            assert self._paper_seek_retract_target_z is not None
            return self._paper_seek_retract_target_z
        return self._paper_seek_start_tip_z + self._paper_seek_offset_m

    def _target_paper_origin(self) -> Point3:
        if self._paper_seek_state in (
            PAPER_SEEK_ZEROING,
            PAPER_SEEK_BASELINING,
            PAPER_SEEK_DESCENDING,
        ):
            return Point3(
                x=self._paper_origin.x,
                y=self._paper_origin.y,
                z=self._paper_seek_start_tip_z + self._paper_seek_offset_m,
            )
        if self._paper_seek_state == PAPER_SEEK_RETRACTING:
            assert self._paper_seek_retract_target_z is not None
            return Point3(
                x=self._paper_origin.x,
                y=self._paper_origin.y,
                z=self._paper_seek_retract_target_z,
            )
        return self._paper_origin

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

    def _on_stop_alignment_logging(self, _request, response):
        sample_count = self._alignment_error_logger.sample_count
        self._alignment_error_logger.stop()
        response.success = True
        response.message = f"stopped after {sample_count} samples"
        self.get_logger().info(
            f"Tool alignment error recording {response.message}."
        )
        return response

    def _publish_tf_markers_and_pose(
        self,
        velocity: PlanarVelocity,
        *,
        target_tool_pose: PoseTarget,
        current_tool_pose: PoseTarget | None,
        publish_pose: bool,
        dt_sec: float,
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
        target_msg = self._make_pose_stamped(stamp, target_tool_pose)
        self._target_pose_publisher.publish(target_msg)
        if publish_pose:
            if self._active_servo_command_mode == "pose":
                self._pose_publisher.publish(target_msg)
            elif self.servo_command_mode == "twist_constant_linear":
                self._twist_publisher.publish(
                    self._make_twist_stamped(
                        stamp,
                        twist_constant_linear_command(velocity),
                    )
                )
            else:
                command = twist_feedforward_command(
                    previous_target=self._previous_target_pose,
                    target=target_tool_pose,
                    current=current_tool_pose,
                    dt_sec=dt_sec,
                    position_gain=self.twist_position_gain,
                    orientation_gain=self.twist_orientation_gain,
                    linear_correction_limit_mps=(
                        self.twist_linear_correction_limit_mps
                    ),
                    angular_correction_limit_radps=(
                        self.twist_angular_correction_limit_radps
                    ),
                    angular_enabled=self.servo_command_mode != "twist_linear_only",
                )
                self._twist_publisher.publish(
                    self._make_twist_stamped(stamp, command)
                )
        self._previous_target_pose = target_tool_pose

    def _make_twist_stamped(self, stamp, command: CartesianTwist) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frames.base_frame
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = command.linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = command.angular
        return msg

    def _publish_zero_twist(self) -> None:
        if self._active_servo_command_mode == "pose":
            return
        self._twist_publisher.publish(
            self._make_twist_stamped(
                self.get_clock().now().to_msg(),
                CartesianTwist(
                    linear=(0.0, 0.0, 0.0),
                    angular=(0.0, 0.0, 0.0),
                ),
            )
        )

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
        self.get_logger().info(
            "Pen Servo node destroy requested. "
            f"shutdown_reason={self._shutdown_reason!r} "
            f"shutdown_requested={self._shutdown_requested_at_sec is not None} "
            f"pose_command_armed={self._pose_command_armed} "
            f"command_mode_ready={self._command_mode_ready} "
            f"servo_status_seen={self._servo_status_seen} "
            f"alignment_log_started={self._alignment_error_logger.started}"
        )
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
        node.get_logger().info(
            "rclpy.spin returned. "
            f"shutdown_reason={node._shutdown_reason!r} "
            f"rclpy_ok={rclpy.ok()}"
        )
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard interrupt received.")
    except (RuntimeError, ValueError) as exc:
        if node is not None:
            node._shutdown_reason = f"exception: {type(exc).__name__}: {exc}"
            node.get_logger().error(
                "Pen fake-hardware Servo node stopped after exception:\n"
                f"{traceback.format_exc()}"
            )
        else:
            print(
                "Pen fake-hardware Servo node refused to start after exception:\n"
                f"{traceback.format_exc()}"
            )
    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info("main() finalizer is calling rclpy.shutdown().")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
