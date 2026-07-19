import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import signal
import statistics
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Twist, WrenchStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_dashboard_msgs.msg import RobotMode, SafetyMode
from ur_dashboard_msgs.srv import GetRobotMode, GetSafetyMode
from ur_msgs.srv import SetForceMode, SetPayload

from .geometry import (
    Point3,
    Quaternion,
    pose_target_from_transform,
    projected_force_z_in_base,
    rotate_vector,
    transform_point,
)
from .handwriting_path import compile_strokes, load_handwriting, path_length


CONFIRMATION = "I_CONFIRM_REAL_Z_COMPLIANCE_TEST"
MOTION_CONTROLLERS = (
    "joint_trajectory_controller",
    "scaled_joint_trajectory_controller",
    "forward_position_controller",
    "forward_velocity_controller",
    "forward_effort_controller",
)
PASSTHROUGH = "passthrough_trajectory_controller"
FORCE = "force_mode_controller"
RETRACT_SETTLE_TIMEOUT_SEC = 2.0
RETRACT_STABLE_WINDOW_SEC = 0.2
RETRACT_STABLE_SPAN_M = 0.0001
PASSTHROUGH_MIN_START_TIME_SEC = 0.1
PASSTHROUGH_MIN_EXECUTION_RATIO = 0.9
LINE_REVERSE_TOLERANCE_M = 0.0001
CONTACT_PATH_MAX_UNDERSHOOT_N = 0.1
CONTACT_PATH_MIN_FORCE_COVERAGE = 0.9
MAX_BASELINE_ABS_N = 0.3
MAX_HANDWRITING_DIMENSION_M = 0.03
MAX_AIR_PATH_LENGTH_M = 0.2
MAX_CONTACT_PATH_LENGTH_M = 0.075
MAX_CONTACT_TOTAL_LENGTH_M = 0.12
MAX_CONTACT_EXECUTION_DISTANCE_M = 0.2
MAX_CONTACT_STROKE_COUNT = 12
MAX_CONTACT_RUN_SEC = 180.0
MAX_CONTACT_WRITING_SEC = 60.0
PATH_ENDPOINT_TOLERANCE_M = 0.0005
PATH_ENDPOINT_SETTLE_TIMEOUT_SEC = 0.5
PATH_ENDPOINT_STABLE_WINDOW_SEC = 0.1
PATH_LATERAL_TOLERANCE_M = 0.0005


@dataclass(frozen=True)
class ControllerDelta:
    activate: tuple[str, ...]
    deactivate: tuple[str, ...]


@dataclass(frozen=True)
class PathTracking:
    progress_m: float
    lateral_error_m: float
    total_length_m: float


def pen_axis_in_base_and_tilt(
    orientation: Quaternion,
    axis_tool: tuple[float, float, float],
) -> tuple[tuple[float, float, float], float]:
    axis_norm = math.sqrt(sum(value * value for value in axis_tool))
    if not math.isfinite(axis_norm) or axis_norm <= 1e-12:
        raise ValueError("pen_axis_tool_xyz must be finite and nonzero")
    unit_axis_tool = tuple(value / axis_norm for value in axis_tool)
    axis_base = rotate_vector(orientation, unit_axis_tool)
    rotated_norm = math.sqrt(sum(value * value for value in axis_base))
    if not math.isfinite(rotated_norm) or rotated_norm <= 1e-12:
        raise ValueError("tool orientation produced an invalid pen axis")
    unit_axis_base = tuple(value / rotated_norm for value in axis_base)
    alignment_with_base_down = max(-1.0, min(1.0, -unit_axis_base[2]))
    return unit_axis_base, math.acos(alignment_with_base_down)


def controller_delta(
    states: dict[str, str], *, activate: tuple[str, ...], deactivate: tuple[str, ...]
) -> ControllerDelta:
    return ControllerDelta(
        activate=tuple(name for name in activate if states.get(name) != "active"),
        deactivate=tuple(name for name in deactivate if states.get(name) == "active"),
    )


def controllers_match(
    states: dict[str, str], *, active: tuple[str, ...], inactive: tuple[str, ...]
) -> bool:
    return all(states.get(name) == "active" for name in active) and all(
        states.get(name) != "active" for name in inactive
    )


def retract_distance_is_stable(
    distances_m: list[float], *, minimum_m: float, maximum_m: float,
    maximum_span_m: float = RETRACT_STABLE_SPAN_M,
) -> bool:
    return bool(distances_m) and all(
        minimum_m <= distance <= maximum_m for distance in distances_m
    ) and max(distances_m) - min(distances_m) <= maximum_span_m


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def retime_passthrough_trajectory(
    trajectory, *, distance_m: float, speed_mps: float,
    start_delay_sec: float = PASSTHROUGH_MIN_START_TIME_SEC,
) -> tuple[float, float]:
    if distance_m <= 0.0 or speed_mps <= 0.0:
        raise ValueError("trajectory distance and speed must be positive")
    if len(trajectory.points) < 2:
        raise ValueError("trajectory must contain at least two points")
    original_start = duration_seconds(trajectory.points[0].time_from_start)
    original_duration = (
        duration_seconds(trajectory.points[-1].time_from_start) - original_start
    )
    if original_duration <= 0.0:
        raise ValueError("trajectory duration must be positive")
    motion_duration = distance_m / speed_mps
    time_scale = motion_duration / original_duration
    for point in trajectory.points:
        progress = (
            duration_seconds(point.time_from_start) - original_start
        ) / original_duration
        point.time_from_start = Duration(
            seconds=start_delay_sec + progress * motion_duration
        ).to_msg()
        if point.velocities:
            point.velocities = [
                velocity / time_scale for velocity in point.velocities
            ]
        if point.accelerations:
            point.accelerations = [
                acceleration / (time_scale * time_scale)
                for acceleration in point.accelerations
            ]
    return motion_duration, time_scale


def execution_completed_too_early(
    *, elapsed_sec: float, commanded_motion_sec: float,
    minimum_ratio: float = PASSTHROUGH_MIN_EXECUTION_RATIO,
) -> bool:
    return elapsed_sec < commanded_motion_sec * minimum_ratio


def line_motion_reversed(
    *, progress_m: float, furthest_progress_m: float,
    tolerance_m: float = LINE_REVERSE_TOLERANCE_M,
) -> bool:
    return progress_m < furthest_progress_m - tolerance_m


def path_contact_acquire_minimum(
    *, target_force_n: float, steady_force_min_n: float,
    maximum_undershoot_n: float = CONTACT_PATH_MAX_UNDERSHOOT_N,
) -> float:
    return max(steady_force_min_n, target_force_n - maximum_undershoot_n)


def contact_force_window_is_stable(
    samples: list[float], *, minimum_mean_n: float, steady_min_n: float,
    steady_max_n: float,
    minimum_coverage: float = CONTACT_PATH_MIN_FORCE_COVERAGE,
) -> bool:
    if not samples:
        return False
    coverage = sum(
        steady_min_n <= force <= steady_max_n for force in samples
    ) / len(samples)
    return statistics.fmean(samples) >= minimum_mean_n and coverage >= minimum_coverage


def validate_contact_strokes(
    strokes,
    *,
    speed_mps: float,
    maximum_stroke_length_m: float = MAX_CONTACT_PATH_LENGTH_M,
    maximum_total_length_m: float = MAX_CONTACT_TOTAL_LENGTH_M,
    maximum_stroke_count: int = MAX_CONTACT_STROKE_COUNT,
    maximum_writing_sec: float = MAX_CONTACT_WRITING_SEC,
):
    if len(strokes) > maximum_stroke_count:
        raise ValueError(
            f"contact stroke count exceeds {maximum_stroke_count}: {len(strokes)}"
        )
    for index, stroke in enumerate(strokes, start=1):
        length = path_length([stroke])
        if length > maximum_stroke_length_m + 1e-12:
            raise ValueError(
                f"contact stroke {index} length exceeds "
                f"{maximum_stroke_length_m * 1000.0:g}mm: {length:.6f}m"
            )
    total_length = path_length(strokes)
    if total_length > maximum_total_length_m + 1e-12:
        raise ValueError(
            "total contact path length exceeds "
            f"{maximum_total_length_m * 1000.0:g}mm: {total_length:.6f}m"
        )
    if total_length / speed_mps > maximum_writing_sec + 1e-12:
        raise ValueError(
            f"estimated contact writing time exceeds {maximum_writing_sec:g}s"
        )
    return strokes


def estimate_contact_run_sec(
    *,
    pen_down_length_m: float,
    execution_distance_m: float,
    stroke_count: int,
    contact_speed_mps: float,
    air_speed_mps: float,
    contact_clearance_m: float,
    retract_distance_m: float,
    max_z_speed_mps: float,
    baseline_settle_sec: float,
    baseline_duration_sec: float,
    contact_settle_sec: float,
) -> float:
    writing_sec = pen_down_length_m / contact_speed_mps
    planar_air_sec = max(0.0, execution_distance_m - pen_down_length_m) / air_speed_mps
    per_stroke_sec = (
        baseline_settle_sec
        + baseline_duration_sec
        + contact_clearance_m / max_z_speed_mps
        + contact_settle_sec
        + 2.0
    )
    vertical_air_sec = (
        max(0, stroke_count - 1) * contact_clearance_m + retract_distance_m
    ) / air_speed_mps
    return (
        writing_sec
        + planar_air_sec
        + stroke_count * per_stroke_sec
        + vertical_air_sec
        + 3.0
    )


def polyline_tracking(point: Point3, path: list[Point3]) -> PathTracking:
    if len(path) < 2:
        raise ValueError("tracking path must contain at least two points")
    best_lateral = math.inf
    best_progress = 0.0
    cumulative = 0.0
    for start, end in zip(path, path[1:]):
        delta_x = end.x - start.x
        delta_y = end.y - start.y
        segment_length = math.hypot(delta_x, delta_y)
        if segment_length <= 1e-12:
            continue
        projection = (
            (point.x - start.x) * delta_x + (point.y - start.y) * delta_y
        ) / (segment_length * segment_length)
        projection = max(0.0, min(1.0, projection))
        nearest_x = start.x + projection * delta_x
        nearest_y = start.y + projection * delta_y
        lateral = math.hypot(point.x - nearest_x, point.y - nearest_y)
        progress = cumulative + projection * segment_length
        if lateral < best_lateral - 1e-12 or (
            math.isclose(lateral, best_lateral, abs_tol=1e-12)
            and progress > best_progress
        ):
            best_lateral = lateral
            best_progress = progress
        cumulative += segment_length
    if not math.isfinite(best_lateral):
        raise ValueError("tracking path has no movement")
    return PathTracking(best_progress, best_lateral, cumulative)


def anchored_tip_strokes(
    strokes,
    *,
    anchor: Point3,
    tip_z: float,
) -> list[list[Point3]]:
    return [
        [Point3(anchor.x + x, anchor.y + y, tip_z) for x, y in stroke]
        for stroke in strokes
    ]


def tip_path_distance(start: Point3, targets: list[Point3]) -> float:
    points = [start, *targets]
    return sum(
        math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        for a, b in zip(points, points[1:])
    )


def stroke_execution_distance(start: Point3, strokes: list[list[Point3]]) -> float:
    distance = 0.0
    current = start
    for stroke in strokes:
        distance += tip_path_distance(current, stroke)
        current = stroke[-1]
    return distance


def tool_waypoints_for_tip_targets(
    current_pose: Pose,
    current_tip: Point3,
    targets: list[Point3],
    fixed_orientation=None,
) -> list[Pose]:
    waypoints = []
    for target in targets:
        waypoint = Pose()
        waypoint.position.x = current_pose.position.x + target.x - current_tip.x
        waypoint.position.y = current_pose.position.y + target.y - current_tip.y
        waypoint.position.z = current_pose.position.z + target.z - current_tip.z
        waypoint.orientation = (
            fixed_orientation
            if fixed_orientation is not None
            else current_pose.orientation
        )
        waypoints.append(waypoint)
    return waypoints


def pose_rotation_distance(first, second) -> float:
    dot = abs(
        first.x * second.x
        + first.y * second.y
        + first.z * second.z
        + first.w * second.w
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def validate_joint_trajectory(trajectory, *, max_joint_step_rad: float = 0.2) -> str | None:
    if len(trajectory.joint_names) != 6:
        return "trajectory must contain six UR joints"
    if len(trajectory.points) < 2:
        return "trajectory must contain at least two points"
    previous_time = -1.0
    previous_positions = None
    for index, point in enumerate(trajectory.points):
        if len(point.positions) != len(trajectory.joint_names):
            return "trajectory point has incomplete joint positions"
        if point.velocities and len(point.velocities) != len(trajectory.joint_names):
            return "trajectory point has incomplete joint velocities"
        if point.accelerations and len(point.accelerations) != len(
            trajectory.joint_names
        ):
            return "trajectory point has incomplete joint accelerations"
        stamp = duration_seconds(point.time_from_start)
        if index == 0 and stamp <= 0.0:
            return "trajectory first time_from_start must be positive"
        if stamp <= previous_time:
            return "trajectory time_from_start must be strictly increasing"
        if previous_positions is not None and any(
            abs(current - previous) > max_joint_step_rad
            for current, previous in zip(point.positions, previous_positions)
        ):
            return "trajectory contains a joint-space jump"
        previous_time = stamp
        previous_positions = point.positions
    return None


def relative_normal_force(*, projected_force_n: float, baseline_force_n: float) -> float:
    return projected_force_n - baseline_force_n


def baseline_compensated_force_target(
    *, relative_target_n: float, baseline_force_n: float,
    maximum_baseline_abs_n: float = MAX_BASELINE_ABS_N,
) -> float:
    if abs(baseline_force_n) > maximum_baseline_abs_n:
        raise ValueError("force baseline exceeds compensation limit")
    command_n = relative_target_n + baseline_force_n
    if command_n <= 0.0:
        raise ValueError("baseline-compensated force target must be positive")
    return command_n


def contact_lost(
    *, force_n: float, threshold_n: float, below_since: float | None, now: float, duration: float
) -> tuple[bool, float | None]:
    if force_n >= threshold_n:
        return False, None
    started = now if below_since is None else below_since
    return now - started >= duration, started


def force_mode_request(
    *, paper_point: PointStamped, target_force_n: float, speed_limit_mps: float,
    damping_factor: float, gain_scaling: float, xy_limit_m: float,
    rotation_limit_rad: float,
) -> SetForceMode.Request:
    request = SetForceMode.Request()
    request.task_frame.header.frame_id = paper_point.header.frame_id or "base_link"
    request.task_frame.pose.position.x = paper_point.point.x
    request.task_frame.pose.position.y = paper_point.point.y
    request.task_frame.pose.position.z = paper_point.point.z
    request.task_frame.pose.orientation.w = 1.0
    request.selection_vector_z = True
    request.wrench.force.z = -abs(target_force_n)
    request.type = SetForceMode.Request.NO_TRANSFORM
    request.speed_limits = Twist()
    request.speed_limits.linear.z = speed_limit_mps
    request.deviation_limits = [
        xy_limit_m,
        xy_limit_m,
        speed_limit_mps,
        rotation_limit_rad,
        rotation_limit_rad,
        rotation_limit_rad,
    ]
    request.damping_factor = damping_factor
    request.gain_scaling = gain_scaling
    return request


class RunStopped(RuntimeError):
    pass


class ZComplianceValidationNode(Node):
    def __init__(self) -> None:
        super().__init__("z_compliance_validation")
        if self.declare_parameter("human_confirmation", "").value != CONFIRMATION:
            raise ValueError(f"human_confirmation must equal {CONFIRMATION}")

        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.tool_frame = str(self.declare_parameter("tool_frame", "tool0").value)
        self.wrench_topic = str(
            self.declare_parameter(
                "wrench_topic", "/force_torque_sensor_broadcaster/wrench"
            ).value
        )
        self.detected_point_topic = str(
            self.declare_parameter(
                "detected_point_topic", "/pen_writing/detected_paper_point"
            ).value
        )
        self.payload_mass_kg = float(
            self.declare_parameter("payload_mass_kg", 0.085).value
        )
        self.payload_cog_xyz = tuple(
            float(value)
            for value in self.declare_parameter(
                "payload_cog_xyz", [0.0, 0.0, 0.0]
            ).value
        )
        self.tool0_to_pen_tip_xyz = Point3(
            *(
                float(value)
                for value in self.declare_parameter(
                    "tool0_to_pen_tip_xyz", [0.00079, -0.00076, 0.15172]
                ).value
            )
        )
        self.pen_axis_tool_xyz = tuple(
            float(value)
            for value in self.declare_parameter(
                "pen_axis_tool_xyz", [0.0, 0.0, 1.0]
            ).value
        )
        self.max_pen_tilt_rad = self._float_parameter(
            "max_pen_tilt_rad", math.radians(1.0)
        )
        self.target_force_n = self._float_parameter("target_force_n", 0.8)
        self.direction_force_n = self._float_parameter("direction_force_n", 0.2)
        self.max_force_filtered_n = self._float_parameter("max_force_filtered_n", 1.5)
        self.max_force_raw_n = self._float_parameter("max_force_raw_n", 2.0)
        self.max_z_speed_mps = self._float_parameter("max_z_speed_mps", 0.0005)
        self.damping_factor = self._float_parameter("damping_factor", 0.5)
        self.gain_scaling = self._float_parameter("gain_scaling", 0.3)
        self.max_acquire_travel_m = self._float_parameter("max_acquire_travel_m", 0.004)
        self.max_contact_z_offset_m = self._float_parameter(
            "max_contact_z_offset_m", 0.0015
        )
        self.max_xy_error_m = self._float_parameter("max_xy_error_m", 0.003)
        self.max_rotation_error_rad = self._float_parameter(
            "max_rotation_error_rad", math.radians(2.0)
        )
        self.steady_force_min_n = self._float_parameter("steady_force_min_n", 0.5)
        self.steady_force_max_n = self._float_parameter("steady_force_max_n", 1.1)
        self.lost_contact_force_n = self._float_parameter("lost_contact_force_n", 0.2)
        self.lost_contact_duration_sec = self._float_parameter(
            "lost_contact_duration_sec", 0.3
        )
        self.retract_distance_m = self._float_parameter("retract_distance_m", 0.003)
        self.contact_clearance_m = self._float_parameter(
            "contact_clearance_m", 0.002
        )
        self.line_length_m = self._float_parameter("line_length_m", 0.01)
        self.line_speed_mps = self._float_parameter("line_speed_mps", 0.003)
        self.air_speed_mps = self._float_parameter("air_speed_mps", 0.005)
        self.max_air_path_length_m = self._float_parameter(
            "max_air_path_length_m", MAX_AIR_PATH_LENGTH_M
        )
        self.max_contact_stroke_length_m = self._float_parameter(
            "max_contact_stroke_length_m", MAX_CONTACT_PATH_LENGTH_M
        )
        self.max_contact_total_length_m = self._float_parameter(
            "max_contact_total_length_m", MAX_CONTACT_TOTAL_LENGTH_M
        )
        self.max_contact_execution_distance_m = self._float_parameter(
            "max_contact_execution_distance_m", MAX_CONTACT_EXECUTION_DISTANCE_M
        )
        self.max_contact_stroke_count = int(
            self.declare_parameter(
                "max_contact_stroke_count", MAX_CONTACT_STROKE_COUNT
            ).value
        )
        self.max_contact_run_sec = self._float_parameter(
            "max_contact_run_sec", MAX_CONTACT_RUN_SEC
        )
        self.cartesian_step_m = self._float_parameter("cartesian_step_m", 0.0005)
        self.trajectory_file = str(
            self.declare_parameter("trajectory_file", "").value
        )
        self.writing_width_m = self._float_parameter("writing_width_m", 0.01)
        self.writing_height_m = self._float_parameter("writing_height_m", 0.01)
        self.path_simplify_tolerance_m = self._float_parameter(
            "path_simplify_tolerance_m", 0.00025
        )
        self.data_timeout_sec = self._float_parameter("data_timeout_sec", 0.2)
        self.baseline_duration_sec = self._float_parameter("baseline_duration_sec", 1.0)
        self.baseline_settle_sec = self._float_parameter("baseline_settle_sec", 0.5)
        self.max_baseline_stddev_n = self._float_parameter("max_baseline_stddev_n", 0.1)
        self.contact_settle_sec = self._float_parameter("contact_settle_sec", 1.0)
        self.hold_duration_sec = self._float_parameter("hold_duration_sec", 5.0)
        self.air_hold_duration_sec = self._float_parameter("air_hold_duration_sec", 2.0)
        self._validate_parameters()

        log_directory = str(self.declare_parameter("log_directory", "").value)
        self._run_directory = (
            Path(log_directory)
            if log_directory
            else Path.cwd()
            / "logs"
            / "force_pen_writing"
            / datetime.now().strftime("%Y%m%d-%H%M%S")
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._lock = threading.Lock()
        self._abort_event = threading.Event()
        self._abort_reason = "manual stop requested"
        self._worker: threading.Thread | None = None
        self._state = "IDLE"
        self._detail = "ready"
        self._paper_point: PointStamped | None = None
        self._latest_joint_state: JointState | None = None
        self._last_joint_time = 0.0
        self._last_wrench_time = 0.0
        self._raw_projected_force_n = 0.0
        self._filtered_projected_force_n = 0.0
        self._filter_initialized = False
        self._baseline_force_n = 0.0
        self._active_target_force_n = 0.0
        self._force_started = False
        self._controllers_switched = False
        self._original_controller_states: dict[str, str] = {}
        self._active_goal_handle = None
        self._profile = ""
        self._last_dashboard_check = 0.0
        self._force_start_tip: Point3 | None = None
        self._force_start_pose: PoseStamped | None = None
        self._contact_tip: Point3 | None = None
        self._line_start_tip: Point3 | None = None
        self._line_max_lateral_error_m = 0.0
        self._contact_path: list[Point3] | None = None
        self._path_max_lateral_error_m = 0.0
        self._active_stroke_index = 0
        self._stroke_count = 0
        self._pen_state = "pen_up"
        self._contact_run_deadline: float | None = None
        self._csv_file = None
        self._csv_writer = None
        self._csv_run_index = 0

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(
            String, "/pen_writing/z_compliance/status", latched
        )
        self._force_pub = self.create_publisher(
            Float64, "/pen_writing/z_compliance/normal_force", 10
        )
        self._offset_pub = self.create_publisher(
            Float64, "/pen_writing/z_compliance/z_offset", 10
        )
        self.create_subscription(WrenchStamped, self.wrench_topic, self._on_wrench, 20)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)
        self.create_subscription(
            PointStamped, self.detected_point_topic, self._on_paper_point, latched
        )

        self._list_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self._switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self._payload_client = self.create_client(
            SetPayload, "/io_and_status_controller/set_payload"
        )
        self._zero_client = self.create_client(
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
        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self._trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/passthrough_trajectory_controller/follow_joint_trajectory",
        )

        for service_name, profile in (
            ("start_switch_hold", "switch_hold"),
            ("start_direction", "direction"),
            ("start_contact_hold", "contact_hold"),
            ("start_line", "line"),
            ("start_path_air", "path_air"),
            ("start_path_contact", "path_contact"),
        ):
            self.create_service(
                Trigger,
                f"/pen_writing/z_compliance/{service_name}",
                lambda request, response, selected=profile: self._start_profile(
                    selected, request, response
                ),
            )
        self.create_service(Trigger, "/pen_writing/z_compliance/stop", self._stop)
        self.create_timer(0.02, self._publish_measurements)
        self._publish_status("IDLE", "ready; no motion starts until a start service is called")

    def _float_parameter(self, name: str, default: float) -> float:
        return float(self.declare_parameter(name, default).value)

    def _validate_parameters(self) -> None:
        if not 0.0 < self.target_force_n <= 1.0:
            raise ValueError("target_force_n must be in (0, 1]")
        if not 0.0 < self.max_z_speed_mps <= 0.0005:
            raise ValueError("max_z_speed_mps must be in (0, 0.0005]")
        if not 0.0 < self.max_acquire_travel_m <= 0.004:
            raise ValueError("max_acquire_travel_m must be in (0, 0.004]")
        if not 0.0 < self.max_contact_z_offset_m <= 0.0015:
            raise ValueError("max_contact_z_offset_m must be in (0, 0.0015]")
        if not 0.0 <= self.damping_factor <= 1.0:
            raise ValueError("damping_factor must be in [0, 1]")
        if not 0.0 < self.gain_scaling <= 1.0:
            raise ValueError("gain_scaling must be in (0, 1]")
        if len(self.payload_cog_xyz) != 3:
            raise ValueError("payload_cog_xyz must contain three values")
        if len(self.pen_axis_tool_xyz) != 3:
            raise ValueError("pen_axis_tool_xyz must contain three values")
        pen_axis_in_base_and_tilt(
            Quaternion(0.0, 0.0, 0.0, 1.0), self.pen_axis_tool_xyz
        )
        if not 0.0 < self.max_pen_tilt_rad <= math.radians(2.0):
            raise ValueError("max_pen_tilt_rad must be in (0, 2deg]")
        if not 0.0 < self.direction_force_n <= 0.5:
            raise ValueError("direction_force_n must be in (0, 0.5]")
        if not (
            0.0
            <= self.lost_contact_force_n
            < self.steady_force_min_n
            < self.steady_force_max_n
            <= self.max_force_filtered_n
            < self.max_force_raw_n
        ):
            raise ValueError("force thresholds are not strictly ordered")
        if self.max_force_filtered_n > 1.5 or self.max_force_raw_n > 2.0:
            raise ValueError("force hard limits exceed the validated bounds")
        if not 0.0 < self.max_xy_error_m <= 0.003:
            raise ValueError("max_xy_error_m must be in (0, 0.003]")
        if not 0.0 < self.max_rotation_error_rad <= math.radians(2.0):
            raise ValueError("max_rotation_error_rad must be in (0, 2deg]")
        if not 0.0 < self.retract_distance_m <= 0.003:
            raise ValueError("retract_distance_m must be in (0, 0.003]")
        if not 0.0 < self.contact_clearance_m <= self.retract_distance_m:
            raise ValueError(
                "contact_clearance_m must be in (0, retract_distance_m]"
            )
        if not 0.0 < self.line_length_m <= 0.01:
            raise ValueError("line_length_m must be in (0, 0.01]")
        if not 0.0 < self.line_speed_mps <= 0.004:
            raise ValueError("line_speed_mps must be in (0, 0.004]")
        if not 0.0 < self.air_speed_mps <= 0.01:
            raise ValueError("air_speed_mps must be in (0, 0.01]")
        if not 0.0 < self.max_air_path_length_m <= MAX_AIR_PATH_LENGTH_M:
            raise ValueError("max_air_path_length_m must be in (0, 0.2]")
        if not 0.0 < self.max_contact_stroke_length_m <= MAX_CONTACT_PATH_LENGTH_M:
            raise ValueError("max_contact_stroke_length_m must be in (0, 0.075]")
        if not 0.0 < self.max_contact_total_length_m <= MAX_CONTACT_TOTAL_LENGTH_M:
            raise ValueError("max_contact_total_length_m must be in (0, 0.12]")
        if not (
            0.0
            < self.max_contact_execution_distance_m
            <= MAX_CONTACT_EXECUTION_DISTANCE_M
        ):
            raise ValueError(
                "max_contact_execution_distance_m must be in (0, 0.2]"
            )
        if not 1 <= self.max_contact_stroke_count <= MAX_CONTACT_STROKE_COUNT:
            raise ValueError("max_contact_stroke_count must be in [1, 12]")
        if not 0.0 < self.max_contact_run_sec <= MAX_CONTACT_RUN_SEC:
            raise ValueError("max_contact_run_sec must be in (0, 180]")
        if not 0.0 < self.cartesian_step_m <= 0.0005:
            raise ValueError("cartesian_step_m must be in (0, 0.0005]")
        if not 0.0 < self.writing_width_m <= MAX_HANDWRITING_DIMENSION_M:
            raise ValueError(
                f"writing_width_m must be in (0, {MAX_HANDWRITING_DIMENSION_M}]"
            )
        if not 0.0 < self.writing_height_m <= MAX_HANDWRITING_DIMENSION_M:
            raise ValueError(
                f"writing_height_m must be in (0, {MAX_HANDWRITING_DIMENSION_M}]"
            )
        if not 0.0 <= self.path_simplify_tolerance_m <= 0.001:
            raise ValueError("path_simplify_tolerance_m must be in [0, 0.001]")

    def _on_paper_point(self, message: PointStamped) -> None:
        self._paper_point = message

    def _on_joint_state(self, message: JointState) -> None:
        self._latest_joint_state = message
        self._last_joint_time = time.monotonic()

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
        projected = projected_force_z_in_base(
            force_xyz=(
                float(message.wrench.force.x),
                float(message.wrench.force.y),
                float(message.wrench.force.z),
            ),
            source_orientation_in_base=orientation,
        )
        with self._lock:
            self._raw_projected_force_n = projected
            if not self._filter_initialized:
                self._filtered_projected_force_n = projected
                self._filter_initialized = True
            else:
                self._filtered_projected_force_n += 0.1 * (
                    projected - self._filtered_projected_force_n
                )
            self._last_wrench_time = time.monotonic()

    def _start_profile(self, profile: str, _request, response):
        if self._worker is not None and self._worker.is_alive():
            response.success = False
            response.message = f"already running: {self._state}"
            return response
        self._abort_event.clear()
        self._abort_reason = "manual stop requested"
        self._worker = threading.Thread(
            target=self._run_profile, args=(profile,), daemon=True
        )
        self._worker.start()
        response.success = True
        response.message = f"{profile} accepted"
        return response

    def _stop(self, _request, response):
        if self._worker is None or not self._worker.is_alive():
            response.success = False
            response.message = "no active Z-compliance run"
            return response
        self._abort_reason = "operator stop requested"
        self._abort_event.set()
        response.success = True
        response.message = "stop requested"
        return response

    def _run_profile(self, profile: str) -> None:
        self._profile = profile
        self._reset_run_tracking()
        succeeded = False
        failure = ""
        try:
            self._open_csv(profile)
            start_tip = self._precheck()
            if profile == "path_contact":
                self._run_contact_path()
            else:
                if profile not in ("switch_hold", "path_air"):
                    self._prepare_force_baseline()
                self._switch_to_passthrough_force()
                self._send_hold_current_joints()
                if profile == "switch_hold":
                    self._air_hold(start_tip)
                elif profile == "path_air":
                    self._run_air_path()
                else:
                    force = (
                        self.direction_force_n
                        if profile == "direction"
                        else self.target_force_n
                    )
                    self._start_force_mode(force)
                    if profile == "direction":
                        self._verify_direction(start_tip)
                    else:
                        self._acquire_contact(start_tip)
                        if profile == "contact_hold":
                            self._hold_contact()
                        else:
                            self._write_line()
            succeeded = True
        except Exception as exc:  # Safety state machine reports the exact failed gate.
            failure = str(exc)
            self.get_logger().error(f"Z-compliance {profile} failed: {failure}")
        finally:
            self._contact_run_deadline = None
            cleanup_error = self._safe_cleanup(
                allow_retract=profile not in ("switch_hold", "path_air")
            )
            if cleanup_error:
                failure = f"{failure}; {cleanup_error}" if failure else cleanup_error
                succeeded = False
            self._publish_status(
                "SUCCEEDED" if succeeded else "ABORTED",
                f"{profile} complete" if succeeded else failure or "stopped",
            )
            self._close_csv()

    def _reset_run_tracking(self) -> None:
        self._baseline_force_n = 0.0
        self._active_target_force_n = 0.0
        self._force_start_tip = None
        self._force_start_pose = None
        self._contact_tip = None
        self._line_start_tip = None
        self._line_max_lateral_error_m = 0.0
        self._contact_path = None
        self._path_max_lateral_error_m = 0.0
        self._active_stroke_index = 0
        self._stroke_count = 0
        self._pen_state = "pen_up"
        self._contact_run_deadline = None

    def _precheck(self) -> Point3:
        self._publish_status("PRECHECK", "checking paper, TF, robot and controllers")
        self._raise_if_stopped()
        if self._paper_point is None:
            raise RunStopped("no detected paper point")
        now = time.monotonic()
        if now - self._last_wrench_time > self.data_timeout_sec:
            raise RunStopped("wrench data is stale")
        if self._latest_joint_state is None or now - self._last_joint_time > self.data_timeout_sec:
            raise RunStopped("joint state is stale")
        if self._profile != "switch_hold":
            self._assert_pen_axis_tilt()
        tip = self._current_tip()
        gap = tip.z - self._paper_point.point.z
        if not 0.002 <= gap <= 0.004:
            raise RunStopped(f"pen-tip air gap must be 2-4mm, got {gap:.6f}m")
        self._assert_dashboard_safe(force=True)
        return tip

    def _assert_pen_axis_tilt(self) -> None:
        pose = self._current_tool_pose_stamped().pose
        orientation = Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        axis_base, tilt_rad = pen_axis_in_base_and_tilt(
            orientation, self.pen_axis_tool_xyz
        )
        tilt_deg = math.degrees(tilt_rad)
        limit_deg = math.degrees(self.max_pen_tilt_rad)
        if tilt_rad > self.max_pen_tilt_rad + 1e-12:
            raise RunStopped(
                "pen axis tilt exceeds absolute limit: "
                f"tilt={tilt_deg:.3f}deg limit={limit_deg:.3f}deg "
                f"axis_base=({axis_base[0]:.6f}, {axis_base[1]:.6f}, "
                f"{axis_base[2]:.6f})"
            )
        self.get_logger().info(
            "Pen-axis precheck passed: "
            f"tilt={tilt_deg:.3f}deg limit={limit_deg:.3f}deg "
            f"axis_base=({axis_base[0]:.6f}, {axis_base[1]:.6f}, "
            f"{axis_base[2]:.6f})"
        )

    def _prepare_force_baseline(self) -> None:
        self._publish_status("AIR_ZERO", "setting payload and zeroing F/T in air")
        payload = SetPayload.Request()
        payload.mass = self.payload_mass_kg
        (
            payload.center_of_gravity.x,
            payload.center_of_gravity.y,
            payload.center_of_gravity.z,
        ) = self.payload_cog_xyz
        result = self._call(self._payload_client, payload, 2.0)
        if not result.success:
            raise RunStopped("set_payload failed")
        result = self._call(self._zero_client, Trigger.Request(), 2.0)
        if not result.success:
            raise RunStopped("zero_ftsensor failed")
        with self._lock:
            self._filter_initialized = False
        self._sleep_checked(self.baseline_settle_sec)
        self._publish_status("BASELINE", "collecting relative-force baseline")
        samples = []
        end = time.monotonic() + self.baseline_duration_sec
        while time.monotonic() < end:
            self._assert_live_data()
            with self._lock:
                samples.append(self._filtered_projected_force_n)
            time.sleep(0.01)
        if len(samples) < 5:
            raise RunStopped("insufficient baseline samples")
        stddev = statistics.pstdev(samples)
        if stddev > self.max_baseline_stddev_n:
            raise RunStopped(f"baseline noise too high: {stddev:.3f}N")
        self._baseline_force_n = statistics.fmean(samples)
        self._publish_status(
            "BASELINE", f"mean={self._baseline_force_n:.3f}N stddev={stddev:.3f}N"
        )

    def _list_controllers(self) -> dict[str, str]:
        result = self._call(self._list_client, ListControllers.Request(), 2.0)
        return {
            controller.name: controller.state for controller in result.controller
        }

    def _switch_to_passthrough_force(self) -> None:
        self._publish_status("CONTROLLER_SWITCH", "activating passthrough + force")
        states = self._list_controllers()
        self._original_controller_states = states
        required = (PASSTHROUGH, FORCE)
        if any(name not in states for name in required):
            raise RunStopped("passthrough or force controller is not loaded")
        delta = controller_delta(states, activate=required, deactivate=MOTION_CONTROLLERS)
        self._switch(delta)
        self._controllers_switched = bool(delta.activate or delta.deactivate)
        states = self._list_controllers()
        if not controllers_match(states, active=required, inactive=MOTION_CONTROLLERS):
            raise RunStopped("controller state verification failed after switch")

    def _switch(self, delta: ControllerDelta) -> None:
        if not delta.activate and not delta.deactivate:
            return
        request = SwitchController.Request()
        request.activate_controllers = list(delta.activate)
        request.deactivate_controllers = list(delta.deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = 5
        result = self._call(self._switch_client, request, 6.0)
        if not result.ok:
            raise RunStopped("STRICT controller switch failed")

    def _send_hold_current_joints(self, *, publish_state: bool = True) -> None:
        if publish_state:
            self._publish_status("AIR_HOLD", "holding current joint position")
        joint_state = self._fresh_joint_state()
        trajectory = JointTrajectory()
        selected = dict(zip(joint_state.name, joint_state.position))
        trajectory.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [selected[name] for name in trajectory.joint_names]
        point.time_from_start.sec = 1
        trajectory.points = [point]
        self._execute_trajectory(trajectory, timeout=3.0, allow_single_point=True)

    def _air_hold(self, start_tip: Point3) -> None:
        end = time.monotonic() + self.air_hold_duration_sec
        while time.monotonic() < end:
            self._assert_live_data()
            current = self._current_tip()
            translation = math.dist(
                (start_tip.x, start_tip.y, start_tip.z),
                (current.x, current.y, current.z),
            )
            if translation > 0.001:
                raise RunStopped("air hold translation exceeded 1mm")
            time.sleep(0.02)

    def _run_air_path(self) -> None:
        if not self.trajectory_file:
            raise RunStopped("trajectory_file is required for path_air")
        assert self._paper_point is not None
        compiled = compile_strokes(
            load_handwriting(self.trajectory_file),
            writing_width_m=self.writing_width_m,
            writing_height_m=self.writing_height_m,
            simplify_tolerance_m=self.path_simplify_tolerance_m,
            cartesian_step_m=self.cartesian_step_m,
        )
        paper = self._paper_point.point
        strokes = anchored_tip_strokes(
            compiled,
            anchor=Point3(paper.x, paper.y, paper.z),
            tip_z=paper.z + self.retract_distance_m,
        )
        start_orientation = self._current_tool_pose_stamped().pose.orientation
        if (
            stroke_execution_distance(self._current_tip(), strokes)
            > self.max_air_path_length_m
        ):
            raise RunStopped(
                "air handwriting execution distance exceeds "
                f"{self.max_air_path_length_m * 1000:g}mm"
            )
        for index, stroke in enumerate(strokes, start=1):
            self._publish_status(
                "AIR_PATH_TRANSITION",
                f"moving above stroke {index}/{len(strokes)} start",
            )
            self._execute_air_tip_targets([stroke[0]], start_orientation)
            self._publish_status(
                "AIR_PATH_STROKE",
                f"executing stroke {index}/{len(strokes)} without contact",
            )
            self._execute_air_tip_targets(stroke[1:], start_orientation)
            rotation_error = pose_rotation_distance(
                self._current_tool_pose_stamped().pose.orientation,
                start_orientation,
            )
            if rotation_error > math.radians(1.0):
                raise RunStopped(
                    "air path rotation error exceeded 1deg: "
                    f"{rotation_error:.6f}rad"
                )

    def _compile_contact_strokes(self) -> list[list[Point3]]:
        if not self.trajectory_file:
            raise RunStopped("trajectory_file is required for path_contact")
        assert self._paper_point is not None
        try:
            compiled = compile_strokes(
                load_handwriting(self.trajectory_file),
                writing_width_m=self.writing_width_m,
                writing_height_m=self.writing_height_m,
                simplify_tolerance_m=self.path_simplify_tolerance_m,
                cartesian_step_m=self.cartesian_step_m,
            )
            strokes = validate_contact_strokes(
                compiled,
                speed_mps=self.line_speed_mps,
                maximum_stroke_length_m=self.max_contact_stroke_length_m,
                maximum_total_length_m=self.max_contact_total_length_m,
                maximum_stroke_count=self.max_contact_stroke_count,
            )
        except (OSError, ValueError) as exc:
            raise RunStopped(f"invalid contact path: {exc}") from exc
        paper = self._paper_point.point
        return anchored_tip_strokes(
            strokes,
            anchor=Point3(paper.x, paper.y, paper.z),
            tip_z=paper.z + self.contact_clearance_m,
        )

    def _run_contact_path(self) -> None:
        strokes = self._compile_contact_strokes()
        pen_down_length = sum(
            tip_path_distance(stroke[0], stroke[1:]) for stroke in strokes
        )
        execution_distance = stroke_execution_distance(self._current_tip(), strokes)
        if execution_distance > self.max_contact_execution_distance_m + 1e-12:
            raise RunStopped(
                "contact handwriting execution distance exceeds "
                f"{self.max_contact_execution_distance_m * 1000:g}mm: "
                f"{execution_distance:.6f}m"
            )
        estimated_run_sec = estimate_contact_run_sec(
            pen_down_length_m=pen_down_length,
            execution_distance_m=execution_distance,
            stroke_count=len(strokes),
            contact_speed_mps=self.line_speed_mps,
            air_speed_mps=self.air_speed_mps,
            contact_clearance_m=self.contact_clearance_m,
            retract_distance_m=self.retract_distance_m,
            max_z_speed_mps=self.max_z_speed_mps,
            baseline_settle_sec=self.baseline_settle_sec,
            baseline_duration_sec=self.baseline_duration_sec,
            contact_settle_sec=self.contact_settle_sec,
        )
        if estimated_run_sec > self.max_contact_run_sec + 1e-12:
            raise RunStopped(
                "estimated contact run time exceeds "
                f"{self.max_contact_run_sec:g}s: {estimated_run_sec:.1f}s"
            )
        self._contact_run_deadline = time.monotonic() + self.max_contact_run_sec
        self._publish_status(
            "CONTACT_PATH_READY",
            f"{len(strokes)} strokes, pen-down={pen_down_length:.3f}m, "
            f"route={execution_distance:.3f}m, estimate={estimated_run_sec:.1f}s",
        )
        start_orientation = self._current_tool_pose_stamped().pose.orientation
        self._stroke_count = len(strokes)
        self._switch_to_passthrough_force()
        self._send_hold_current_joints()
        for index, stroke in enumerate(strokes, start=1):
            self._active_stroke_index = index
            self._contact_path = stroke
            self._publish_status(
                "CONTACT_PATH_TRANSITION",
                f"moving above stroke {index}/{len(strokes)} start",
            )
            self._execute_air_tip_targets([stroke[0]], start_orientation)
            rotation_error = pose_rotation_distance(
                self._current_tool_pose_stamped().pose.orientation,
                start_orientation,
            )
            if rotation_error > math.radians(1.0):
                raise RunStopped(
                    "contact path air-transition rotation error exceeded 1deg: "
                    f"{rotation_error:.6f}rad"
                )
            self._prepare_force_baseline()
            contact_start = self._current_tip()
            self._start_force_mode(self.target_force_n)
            self._acquire_contact(
                contact_start,
                minimum_mean_force_n=path_contact_acquire_minimum(
                    target_force_n=self.target_force_n,
                    steady_force_min_n=self.steady_force_min_n,
                ),
            )
            self._pen_state = "pen_down"
            self._write_contact_path(stroke, start_orientation)
            if index < len(strokes):
                self._lift_between_contact_strokes()

    def _write_contact_path(self, stroke: list[Point3], orientation) -> None:
        assert self._contact_tip is not None
        contact_z = self._current_tip().z
        self._contact_path = [
            Point3(point.x, point.y, contact_z) for point in stroke
        ]
        self._path_max_lateral_error_m = 0.0
        self._publish_status(
            "CONTACT_PATH_WRITING",
            f"executing stroke {self._active_stroke_index}/{self._stroke_count}",
        )
        trajectory = self._plan_tip_targets(
            self._contact_path[1:],
            fixed_orientation=orientation,
            speed_mps=self.line_speed_mps,
        )
        distance = path_length(
            [[(point.x, point.y) for point in self._contact_path]]
        )
        timeout = max(10.0, distance / self.line_speed_mps + 5.0)
        self._execute_trajectory(
            trajectory, timeout=timeout, monitor_contact=True
        )
        current = self._current_tip()
        endpoint = self._contact_path[-1]
        endpoint_error = math.hypot(current.x - endpoint.x, current.y - endpoint.y)
        if endpoint_error > PATH_ENDPOINT_TOLERANCE_M:
            raise RunStopped(
                f"contact path endpoint error exceeded: {endpoint_error:.6f}m"
            )
        if self._path_max_lateral_error_m > PATH_LATERAL_TOLERANCE_M:
            raise RunStopped(
                "contact path lateral error exceeded: "
                f"{self._path_max_lateral_error_m:.6f}m"
            )
        _, rotation_error = self._tracking_errors(current)
        if rotation_error > math.radians(1.0):
            raise RunStopped(
                "contact path final rotation error exceeded 1deg: "
                f"{rotation_error:.6f}rad"
            )

    def _lift_between_contact_strokes(self) -> None:
        self._publish_status(
            "CONTACT_PATH_PEN_UP",
            f"lifting after stroke {self._active_stroke_index}/{self._stroke_count}",
        )
        self._send_hold_current_joints(publish_state=False)
        result = self._call(self._stop_force_client, Trigger.Request(), 3.0)
        if not result.success:
            raise RunStopped("stop_force_mode returned false between strokes")
        self._force_started = False
        self._active_target_force_n = 0.0
        self._pen_state = "pen_up"
        retract_start = self._current_tip()
        trajectory = self._plan_cartesian(
            delta_x=0.0,
            delta_z=self.contact_clearance_m,
            speed_mps=self.air_speed_mps,
        )
        self._execute_trajectory(trajectory, timeout=6.0)
        self._wait_for_stable_retract(
            retract_start, expected_distance_m=self.contact_clearance_m
        )
        self._contact_tip = None
        self._force_start_tip = None
        self._force_start_pose = None

    def _execute_air_tip_targets(self, targets: list[Point3], orientation) -> None:
        if not targets:
            return
        start = self._current_tip()
        distance = tip_path_distance(start, targets)
        if distance <= 0.00005:
            self._assert_tip_endpoint(targets[-1])
            return
        trajectory = self._plan_tip_targets(
            targets,
            fixed_orientation=orientation,
            speed_mps=self.air_speed_mps,
        )
        timeout = max(10.0, distance / self.air_speed_mps + 5.0)
        self._execute_trajectory(
            trajectory,
            timeout=timeout,
            monitor_live=True,
        )
        self._assert_tip_endpoint(targets[-1])

    def _assert_tip_endpoint(self, target: Point3) -> None:
        started = time.monotonic()
        deadline = started + PATH_ENDPOINT_SETTLE_TIMEOUT_SEC
        stable_since = None
        initial_error = None
        error = math.inf
        while True:
            self._assert_live_data()
            now = time.monotonic()
            current = self._current_tip()
            error = math.dist(
                (current.x, current.y, current.z),
                (target.x, target.y, target.z),
            )
            if initial_error is None:
                initial_error = error
            if error <= PATH_ENDPOINT_TOLERANCE_M:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= PATH_ENDPOINT_STABLE_WINDOW_SEC:
                    if initial_error > PATH_ENDPOINT_TOLERANCE_M:
                        self.get_logger().info(
                            "Air path endpoint settled: "
                            f"initial_error={initial_error:.6f}m "
                            f"final_error={error:.6f}m "
                            f"wait={now - started:.3f}s"
                        )
                    return
            else:
                stable_since = None
            if now >= deadline:
                raise RunStopped(
                    "path endpoint error exceeded after settling: "
                    f"{error:.6f}m"
                )
            time.sleep(0.01)

    def _start_force_mode(self, force_n: float) -> None:
        assert self._paper_point is not None
        self._force_start_tip = self._current_tip()
        self._force_start_pose = self._current_tool_pose_stamped()
        try:
            command_force_n = baseline_compensated_force_target(
                relative_target_n=force_n,
                baseline_force_n=self._baseline_force_n,
            )
        except ValueError as exc:
            raise RunStopped(str(exc)) from exc
        request = force_mode_request(
            paper_point=self._paper_point,
            target_force_n=command_force_n,
            speed_limit_mps=self.max_z_speed_mps,
            damping_factor=self.damping_factor,
            gain_scaling=self.gain_scaling,
            xy_limit_m=self.max_xy_error_m,
            rotation_limit_rad=self.max_rotation_error_rad,
        )
        result = self._call(self._start_force_client, request, 3.0)
        if not result.success:
            raise RunStopped("start_force_mode failed")
        self._active_target_force_n = force_n
        self._force_started = True

    def _verify_direction(self, start_tip: Point3) -> None:
        self._publish_status("DIRECTION_CHECK", "verifying motion toward -paper Z")
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            self._assert_live_data(check_contact_force=False)
            current = self._current_tip()
            downward = start_tip.z - current.z
            lateral = math.hypot(current.x - start_tip.x, current.y - start_tip.y)
            if lateral > 0.001:
                raise RunStopped("direction check lateral displacement exceeded 1mm")
            if downward >= 0.0001:
                return
            if abs(downward) > 0.0005:
                raise RunStopped("direction check displacement exceeded 0.5mm")
            time.sleep(0.01)
        raise RunStopped("direction check timed out")

    def _acquire_contact(
        self, start_tip: Point3, *, minimum_mean_force_n: float | None = None
    ) -> None:
        acquisition_minimum_n = (
            self.steady_force_min_n
            if minimum_mean_force_n is None
            else max(self.steady_force_min_n, minimum_mean_force_n)
        )
        if acquisition_minimum_n > self.steady_force_max_n:
            raise RunStopped("contact acquisition force band is invalid")
        if minimum_mean_force_n is None:
            detail = (
                "approaching paper under Force Mode; stable band="
                f"[{acquisition_minimum_n:.3f}, {self.steady_force_max_n:.3f}]N"
            )
        else:
            detail = (
                "approaching paper under Force Mode; stable window mean>="
                f"{acquisition_minimum_n:.3f}N coverage>="
                f"{CONTACT_PATH_MIN_FORCE_COVERAGE:.0%} in "
                f"[{self.steady_force_min_n:.3f}, "
                f"{self.steady_force_max_n:.3f}]N"
            )
        self._publish_status("CONTACT_ACQUIRE", detail)
        stable_since = None
        force_window: list[tuple[float, float]] = []
        deadline = (
            time.monotonic()
            + self.max_acquire_travel_m / self.max_z_speed_mps
            + self.contact_settle_sec
            + 2.0
        )
        while True:
            now = time.monotonic()
            if now > deadline:
                raise RunStopped("contact acquisition timed out")
            force = self._assert_live_data(check_contact_force=True)
            current = self._current_tip()
            travel = start_tip.z - current.z
            if travel > self.max_acquire_travel_m or travel < -0.0005:
                raise RunStopped(f"contact acquisition Z travel exceeded: {travel:.6f}m")
            if minimum_mean_force_n is not None:
                force_window.append((now, force))
                force_window = [
                    sample
                    for sample in force_window
                    if now - sample[0] <= self.contact_settle_sec
                ]
                window_forces = [sample[1] for sample in force_window]
                window_span = now - force_window[0][0]
                if (
                    window_span >= self.contact_settle_sec * 0.9
                    and contact_force_window_is_stable(
                        window_forces,
                        minimum_mean_n=acquisition_minimum_n,
                        steady_min_n=self.steady_force_min_n,
                        steady_max_n=self.steady_force_max_n,
                    )
                ):
                    mean_force = statistics.fmean(window_forces)
                    coverage = sum(
                        self.steady_force_min_n <= sample <= self.steady_force_max_n
                        for sample in window_forces
                    ) / len(window_forces)
                    self._contact_tip = current
                    self._publish_status(
                        "CONTACT_STABLE",
                        f"window_mean={mean_force:.3f}N coverage={coverage:.1%}",
                    )
                    return
            elif acquisition_minimum_n <= force <= self.steady_force_max_n:
                stable_since = stable_since or now
                if now - stable_since >= self.contact_settle_sec:
                    self._contact_tip = current
                    self._publish_status(
                        "CONTACT_STABLE",
                        f"force={force:.3f}N minimum={acquisition_minimum_n:.3f}N",
                    )
                    return
            else:
                stable_since = None
            time.sleep(0.01)

    def _hold_contact(self) -> None:
        self._publish_status("HOLDING", "holding 0.8N relative contact")
        self._monitor_contact_until(time.monotonic() + self.hold_duration_sec)

    def _write_line(self) -> None:
        self._publish_status("WRITING", "planning 10mm +X line")
        self._line_start_tip = self._current_tip()
        self._line_max_lateral_error_m = 0.0
        trajectory = self._plan_cartesian(
            delta_x=self.line_length_m,
            delta_z=0.0,
            speed_mps=self.line_speed_mps,
        )
        timeout = max(10.0, self.line_length_m / self.line_speed_mps + 5.0)
        self._execute_trajectory(trajectory, timeout=timeout, monitor_contact=True)
        current = self._current_tip()
        endpoint_error = math.hypot(
            current.x - (self._line_start_tip.x + self.line_length_m),
            current.y - self._line_start_tip.y,
        )
        if endpoint_error > 0.0005:
            raise RunStopped(f"line endpoint error exceeded: {endpoint_error:.6f}m")
        if self._line_max_lateral_error_m > 0.0005:
            raise RunStopped(
                "line lateral error exceeded: "
                f"{self._line_max_lateral_error_m:.6f}m"
            )
        _, rotation_error = self._tracking_errors(current)
        if rotation_error > math.radians(1.0):
            raise RunStopped(
                f"line final rotation error exceeded 1deg: {rotation_error:.6f}rad"
            )

    def _plan_cartesian(
        self, *, delta_x: float, delta_z: float, speed_mps: float
    ):
        tip = self._current_tip()
        return self._plan_tip_targets(
            [Point3(tip.x + delta_x, tip.y, tip.z + delta_z)],
            current_tip=tip,
            speed_mps=speed_mps,
        )

    def _plan_tip_targets(
        self,
        targets: list[Point3],
        *,
        fixed_orientation=None,
        current_tip: Point3 | None = None,
        speed_mps: float,
    ):
        if not targets:
            raise RunStopped("Cartesian target list must not be empty")
        pose = self._current_tool_pose_stamped()
        if current_tip is None:
            current_tip = self._current_tip()
        distance = tip_path_distance(current_tip, targets)
        if distance <= 0.0:
            raise RunStopped("Cartesian path distance must be positive")
        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.group_name = "ur_manipulator"
        request.link_name = self.tool_frame
        request.start_state = RobotState(joint_state=self._fresh_joint_state())
        request.waypoints = tool_waypoints_for_tip_targets(
            pose.pose,
            current_tip,
            targets,
            fixed_orientation,
        )
        request.max_step = self.cartesian_step_m
        request.jump_threshold = 2.0
        request.revolute_jump_threshold = 0.2
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.05
        request.cartesian_speed_limited_link = self.tool_frame
        request.max_cartesian_speed = speed_mps
        result = self._call(self._cartesian_client, request, 5.0)
        if result.error_code.val != MoveItErrorCodes.SUCCESS or result.fraction < 0.999:
            raise RunStopped(
                "Cartesian path incomplete: "
                f"code={result.error_code.val} fraction={result.fraction:.3f}"
            )
        trajectory = result.solution.joint_trajectory
        try:
            motion_duration, time_scale = retime_passthrough_trajectory(
                trajectory,
                distance_m=distance,
                speed_mps=speed_mps,
            )
        except ValueError as exc:
            raise RunStopped(str(exc)) from exc
        error = validate_joint_trajectory(trajectory)
        if error:
            raise RunStopped(error)
        first_time = duration_seconds(trajectory.points[0].time_from_start)
        final_time = duration_seconds(trajectory.points[-1].time_from_start)
        if not math.isclose(
            final_time - first_time, motion_duration, abs_tol=1e-6
        ):
            raise RunStopped("Cartesian trajectory retiming failed")
        self.get_logger().info(
            "Retimed Cartesian trajectory: "
            f"distance={distance:.6f}m speed={speed_mps:.6f}m/s "
            f"motion={motion_duration:.3f}s scale={time_scale:.3f} "
            f"first={first_time:.3f}s "
            f"final={final_time:.3f}s"
        )
        return trajectory

    def _execute_trajectory(
        self, trajectory, *, timeout: float, monitor_contact: bool = False,
        monitor_live: bool = False, allow_single_point: bool = False,
    ) -> None:
        if not allow_single_point:
            error = validate_joint_trajectory(trajectory)
            if error:
                raise RunStopped(error)
        if not self._trajectory_client.wait_for_server(timeout_sec=2.0):
            raise RunStopped("passthrough trajectory action is unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        execution_started = time.monotonic()
        handle = self._wait_future(
            self._trajectory_client.send_goal_async(goal), 3.0, "trajectory goal"
        )
        if handle is None or not handle.accepted:
            raise RunStopped("passthrough trajectory goal rejected")
        self._active_goal_handle = handle
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout
        below_since = None
        in_band = total = 0
        furthest_progress = 0.0
        while not result_future.done():
            if time.monotonic() > deadline:
                handle.cancel_goal_async()
                raise RunStopped("passthrough trajectory timed out")
            if monitor_contact:
                force = self._assert_live_data(check_contact_force=True)
                total += 1
                in_band += int(self.steady_force_min_n <= force <= self.steady_force_max_n)
                lost, below_since = contact_lost(
                    force_n=force,
                    threshold_n=self.lost_contact_force_n,
                    below_since=below_since,
                    now=time.monotonic(),
                    duration=self.lost_contact_duration_sec,
                )
                if lost:
                    handle.cancel_goal_async()
                    raise RunStopped(f"contact lost during {self._profile}")
                current_tip = self._current_tip()
                if self._profile == "path_contact":
                    assert self._contact_path is not None
                    tracking = polyline_tracking(current_tip, self._contact_path)
                    if tracking.lateral_error_m > PATH_LATERAL_TOLERANCE_M:
                        handle.cancel_goal_async()
                        raise RunStopped(
                            "contact path lateral error exceeded 0.5mm: "
                            f"{tracking.lateral_error_m:.6f}m"
                        )
                    progress = tracking.progress_m
                    motion_label = "contact path"
                else:
                    assert self._line_start_tip is not None
                    progress = current_tip.x - self._line_start_tip.x
                    motion_label = "line"
                if line_motion_reversed(
                    progress_m=progress,
                    furthest_progress_m=furthest_progress,
                ):
                    handle.cancel_goal_async()
                    raise RunStopped(
                        f"{motion_label} reversed beyond 0.1mm: "
                        f"progress={progress:.6f}m "
                        f"furthest={furthest_progress:.6f}m"
                    )
                furthest_progress = max(furthest_progress, progress)
                self._assert_contact_z_offset()
            elif monitor_live:
                self._assert_live_data()
            self._raise_if_stopped()
            time.sleep(0.01)
        wrapped = result_future.result()
        self._active_goal_handle = None
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RunStopped(f"trajectory action failed with status {wrapped.status}")
        if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RunStopped(f"trajectory controller error {wrapped.result.error_code}")
        elapsed = time.monotonic() - execution_started
        if not allow_single_point:
            commanded_motion = (
                duration_seconds(trajectory.points[-1].time_from_start)
                - duration_seconds(trajectory.points[0].time_from_start)
            )
            if execution_completed_too_early(
                elapsed_sec=elapsed, commanded_motion_sec=commanded_motion
            ):
                raise RunStopped(
                    "trajectory completed too early: "
                    f"elapsed={elapsed:.3f}s commanded_motion={commanded_motion:.3f}s"
                )
        if monitor_contact and (total == 0 or in_band / total < 0.9):
            raise RunStopped(f"steady-force coverage below 90%: {in_band}/{total}")

    def _monitor_contact_until(self, deadline: float) -> None:
        below_since = None
        in_band = total = 0
        while time.monotonic() < deadline:
            force = self._assert_live_data(check_contact_force=True)
            total += 1
            in_band += int(self.steady_force_min_n <= force <= self.steady_force_max_n)
            lost, below_since = contact_lost(
                force_n=force,
                threshold_n=self.lost_contact_force_n,
                below_since=below_since,
                now=time.monotonic(),
                duration=self.lost_contact_duration_sec,
            )
            if lost:
                raise RunStopped("contact lost during hold")
            self._assert_contact_z_offset()
            time.sleep(0.01)
        if total == 0 or in_band / total < 0.9:
            raise RunStopped(f"steady-force coverage below 90%: {in_band}/{total}")

    def _assert_live_data(self, *, check_contact_force: bool = False) -> float:
        self._raise_if_stopped()
        now = time.monotonic()
        if now - self._last_wrench_time > self.data_timeout_sec:
            raise RunStopped("wrench data timed out")
        if now - self._last_joint_time > self.data_timeout_sec:
            raise RunStopped("joint state timed out")
        self._assert_dashboard_safe()
        with self._lock:
            raw = relative_normal_force(
                projected_force_n=self._raw_projected_force_n,
                baseline_force_n=self._baseline_force_n,
            )
            filtered = relative_normal_force(
                projected_force_n=self._filtered_projected_force_n,
                baseline_force_n=self._baseline_force_n,
            )
        enforce_force_limits = check_contact_force or self._force_started
        if enforce_force_limits and abs(raw) > self.max_force_raw_n:
            raise RunStopped(f"raw force limit exceeded: {raw:.3f}N")
        if enforce_force_limits and abs(filtered) > self.max_force_filtered_n:
            raise RunStopped(f"filtered force limit exceeded: {filtered:.3f}N")
        if self._force_started:
            self._assert_noncompliant_limits()
        self._write_csv(raw, filtered)
        return filtered

    def _assert_dashboard_safe(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self._last_dashboard_check < 0.25:
            return
        robot = self._call(self._robot_mode_client, GetRobotMode.Request(), 1.0)
        safety = self._call(self._safety_mode_client, GetSafetyMode.Request(), 1.0)
        self._last_dashboard_check = time.monotonic()
        if not robot.success or robot.robot_mode.mode != RobotMode.RUNNING:
            raise RunStopped(f"robot mode is not RUNNING: {robot.robot_mode.mode}")
        if not safety.success or safety.safety_mode.mode != SafetyMode.NORMAL:
            raise RunStopped(f"safety mode is not NORMAL: {safety.safety_mode.mode}")

    def _assert_noncompliant_limits(self) -> None:
        if self._force_start_tip is None or self._force_start_pose is None:
            return
        tip = self._current_tip()
        xy_error, rotation_error = self._tracking_errors(tip)
        if xy_error > self.max_xy_error_m:
            raise RunStopped(f"noncompliant XY error exceeded: {xy_error:.6f}m")
        if rotation_error > self.max_rotation_error_rad:
            raise RunStopped(
                f"noncompliant rotation error exceeded: {rotation_error:.6f}rad"
            )

    def _tracking_errors(self, tip: Point3) -> tuple[float, float]:
        assert self._force_start_tip is not None
        assert self._force_start_pose is not None
        if self._profile == "path_contact" and self._contact_path is not None:
            tracking = polyline_tracking(tip, self._contact_path)
            self._path_max_lateral_error_m = max(
                self._path_max_lateral_error_m, tracking.lateral_error_m
            )
            xy_error = tracking.lateral_error_m
        elif self._profile == "line" and self._line_start_tip is not None:
            lateral_error = abs(tip.y - self._line_start_tip.y)
            self._line_max_lateral_error_m = max(
                self._line_max_lateral_error_m, lateral_error
            )
            x_error = max(
                self._line_start_tip.x - tip.x,
                tip.x - (self._line_start_tip.x + self.line_length_m),
                0.0,
            )
            xy_error = math.hypot(x_error, tip.y - self._line_start_tip.y)
        else:
            xy_error = math.hypot(
                tip.x - self._force_start_tip.x,
                tip.y - self._force_start_tip.y,
            )
        current = self._current_tool_pose_stamped().pose.orientation
        start = self._force_start_pose.pose.orientation
        rotation_error = pose_rotation_distance(current, start)
        return xy_error, rotation_error

    def _assert_contact_z_offset(self) -> None:
        if self._contact_tip is None:
            return
        current = self._current_tip()
        if abs(current.z - self._contact_tip.z) > self.max_contact_z_offset_m:
            raise RunStopped("contact Z offset exceeded")

    def _safe_cleanup(self, *, allow_retract: bool) -> str:
        errors = []
        self._abort_event.clear()
        self._publish_status("FORCE_STOP", "canceling trajectory and holding joints")
        self._cancel_active_goal(wait=True)
        force_was_started = self._force_started
        if self._controllers_switched:
            try:
                self._send_hold_current_joints(publish_state=False)
            except Exception as exc:
                errors.append(f"current-joint hold failed: {exc}")
        if self._force_started:
            try:
                result = self._call(
                    self._stop_force_client, Trigger.Request(), 3.0, allow_abort=False
                )
                if not result.success:
                    raise RunStopped("stop_force_mode returned false")
                self._force_started = False
                self._active_target_force_n = 0.0
                self._pen_state = "pen_up"
            except Exception as exc:
                errors.append(f"stop force failed; use robot stop: {exc}")
                return "; ".join(errors)
        if self._controllers_switched:
            if allow_retract and force_was_started and self._safety_is_normal():
                try:
                    self._publish_status("RETRACT", "retracting 3mm along base +Z")
                    retract_start = self._current_tip()
                    trajectory = self._plan_cartesian(
                        delta_x=0.0,
                        delta_z=self.retract_distance_m,
                        speed_mps=self.air_speed_mps,
                    )
                    self._execute_trajectory(trajectory, timeout=6.0)
                    self._wait_for_stable_retract(retract_start)
                except Exception as exc:
                    errors.append(f"retract failed: {exc}")
            elif allow_retract and force_was_started:
                errors.append("safety is not NORMAL; automatic retract skipped")
            try:
                self._publish_status(
                    "CONTROLLER_RESTORE", "restoring pre-run controller state"
                )
                self._restore_controllers()
            except Exception as exc:
                errors.append(f"controller restore failed; use robot stop: {exc}")
        return "; ".join(errors)

    def _wait_for_stable_retract(
        self,
        retract_start: Point3,
        *,
        expected_distance_m: float | None = None,
    ) -> float:
        expected = (
            self.retract_distance_m
            if expected_distance_m is None
            else expected_distance_m
        )
        minimum = max(0.0, expected - 0.001)
        maximum = expected + 0.001
        deadline = time.monotonic() + RETRACT_SETTLE_TIMEOUT_SEC
        samples: list[tuple[float, float]] = []
        last_distance = 0.0
        while time.monotonic() < deadline:
            self._assert_dashboard_safe()
            now = time.monotonic()
            last_distance = self._current_tip().z - retract_start.z
            if last_distance > maximum:
                raise RunStopped(
                    "retract distance exceeded expected+/-1mm: "
                    f"expected={expected:.6f}m actual={last_distance:.6f}m"
                )
            samples.append((now, last_distance))
            samples = [
                sample for sample in samples
                if now - sample[0] <= RETRACT_STABLE_WINDOW_SEC
            ]
            if (
                samples
                and now - samples[0][0] >= RETRACT_STABLE_WINDOW_SEC * 0.9
                and retract_distance_is_stable(
                    [distance for _, distance in samples],
                    minimum_m=minimum,
                    maximum_m=maximum,
                )
            ):
                return last_distance
            time.sleep(0.02)
        raise RunStopped(
            "retract did not reach and settle within expected+/-1mm: "
            f"expected={expected:.6f}m last={last_distance:.6f}m"
        )

    def _restore_controllers(self) -> None:
        states = self._list_controllers()
        originally_active = tuple(
            name for name, state in self._original_controller_states.items() if state == "active"
        )
        delta = controller_delta(
            states,
            activate=originally_active,
            deactivate=tuple(
                name for name in (PASSTHROUGH, FORCE) if name not in originally_active
            ),
        )
        self._switch(delta)
        restored = self._list_controllers()
        if not all(restored.get(name) == "active" for name in originally_active):
            raise RunStopped("original controller state was not restored")
        self._controllers_switched = False

    def _safety_is_normal(self) -> bool:
        try:
            result = self._call(
                self._safety_mode_client,
                GetSafetyMode.Request(),
                2.0,
                allow_abort=False,
            )
            return result.safety_mode.mode == SafetyMode.NORMAL
        except Exception:
            return False

    def _current_tool_pose_stamped(self) -> PoseStamped:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            raise RunStopped(f"tool TF unavailable: {exc}") from exc
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _current_tip(self, *, timeout_sec: float = 0.2) -> Point3:
        try:
            transform = self._tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException as exc:
            raise RunStopped(f"tool TF unavailable: {exc}") from exc
        pose = pose_target_from_transform(transform)
        return transform_point(pose, self.tool0_to_pen_tip_xyz)

    def _fresh_joint_state(self) -> JointState:
        if (
            self._latest_joint_state is None
            or time.monotonic() - self._last_joint_time > self.data_timeout_sec
        ):
            raise RunStopped("joint state unavailable")
        copied = JointState()
        copied.header = self._latest_joint_state.header
        copied.name = list(self._latest_joint_state.name)
        copied.position = list(self._latest_joint_state.position)
        copied.velocity = list(self._latest_joint_state.velocity)
        return copied

    def _call(self, client, request, timeout: float, *, allow_abort: bool = True):
        if not client.wait_for_service(timeout_sec=min(timeout, 1.0)):
            raise RunStopped(f"service unavailable: {client.srv_name}")
        return self._wait_future(
            client.call_async(request), timeout, client.srv_name, allow_abort=allow_abort
        )

    def _wait_future(
        self, future, timeout: float, label: str, *, allow_abort: bool = True
    ):
        deadline = time.monotonic() + timeout
        while not future.done():
            if allow_abort:
                self._raise_if_stopped()
            if time.monotonic() > deadline:
                raise RunStopped(f"{label} timed out")
            time.sleep(0.01)
        if future.exception() is not None:
            raise RunStopped(f"{label} failed: {future.exception()}")
        return future.result()

    def _sleep_checked(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._raise_if_stopped()
            time.sleep(0.01)

    def _raise_if_stopped(self) -> None:
        if self._abort_event.is_set():
            raise RunStopped(self._abort_reason)
        contact_deadline = getattr(self, "_contact_run_deadline", None)
        if contact_deadline is not None and time.monotonic() > contact_deadline:
            raise RunStopped(
                f"contact run time exceeded {self.max_contact_run_sec:g}s"
            )

    def _cancel_active_goal(self, *, wait: bool = False) -> None:
        handle = self._active_goal_handle
        if handle is not None:
            try:
                future = handle.cancel_goal_async()
                if wait:
                    self._wait_future(
                        future, 2.0, "trajectory cancel", allow_abort=False
                    )
            except Exception:
                pass
            self._active_goal_handle = None

    def _publish_measurements(self) -> None:
        with self._lock:
            force = relative_normal_force(
                projected_force_n=self._filtered_projected_force_n,
                baseline_force_n=self._baseline_force_n,
            )
        self._force_pub.publish(Float64(data=force))
        offset = 0.0
        if self._contact_tip is not None:
            try:
                offset = self._current_tip(timeout_sec=0.0).z - self._contact_tip.z
            except (TransformException, RunStopped):
                pass
        self._offset_pub.publish(Float64(data=offset))

    def _publish_status(self, state: str, detail: str) -> None:
        self._state = state
        self._detail = detail
        self._status_pub.publish(String(data=f"{state}: {detail}"))
        self.get_logger().info(f"Z compliance {state}: {detail}")

    def _open_csv(self, profile: str) -> None:
        self._run_directory.mkdir(parents=True, exist_ok=True)
        while True:
            self._csv_run_index += 1
            csv_path = (
                self._run_directory / f"{profile}_{self._csv_run_index:03d}.csv"
            )
            try:
                self._csv_file = csv_path.open(
                    "x", newline="", encoding="utf-8", buffering=1
                )
                break
            except FileExistsError:
                continue
        self.get_logger().info(f"Z compliance recording started: {csv_path}")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            (
                "monotonic_sec", "state", "target_force_n", "raw_force_n",
                "filtered_force_n", "tip_x", "tip_y", "tip_z", "z_offset_m",
                "xy_error_m", "rotation_error_rad", "controllers_switched",
                "force_started", "trajectory_action_state",
                "trajectory_file_id", "stroke_index", "stroke_count",
                "planned_progress_m", "lateral_error_m", "pen_state",
            )
        )

    def _write_csv(self, raw: float, filtered: float) -> None:
        if self._csv_writer is None:
            return
        try:
            tip = self._current_tip()
        except Exception:
            return
        z_offset = 0.0 if self._contact_tip is None else tip.z - self._contact_tip.z
        xy_error = rotation_error = 0.0
        planned_progress = lateral_error = 0.0
        if self._force_start_tip is not None and self._force_start_pose is not None:
            try:
                xy_error, rotation_error = self._tracking_errors(tip)
            except Exception:
                pass
        if self._contact_path:
            try:
                tracking = polyline_tracking(tip, self._contact_path)
                planned_progress = tracking.progress_m
                lateral_error = tracking.lateral_error_m
            except ValueError:
                pass
        self._csv_writer.writerow(
            (
                time.monotonic(), self._state, self._active_target_force_n, raw,
                filtered, tip.x, tip.y, tip.z, z_offset, xy_error,
                rotation_error, self._controllers_switched, self._force_started,
                "active" if self._active_goal_handle is not None else "idle",
                Path(self.trajectory_file).name if self.trajectory_file else "",
                self._active_stroke_index, self._stroke_count,
                planned_progress, lateral_error, self._pen_state,
            )
        )

    def _close_csv(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
        self._csv_file = None
        self._csv_writer = None


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ZComplianceValidationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    def request_safe_shutdown(_signum, _frame) -> None:
        node._abort_reason = "process shutdown requested"
        node._abort_event.set()
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_safe_shutdown)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node._abort_reason = "process shutdown requested"
        node._abort_event.set()
        deadline = time.monotonic() + 12.0
        while (
            node._worker is not None
            and node._worker.is_alive()
            and time.monotonic() < deadline
        ):
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
