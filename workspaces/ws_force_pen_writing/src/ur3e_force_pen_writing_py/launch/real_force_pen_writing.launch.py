import math
from datetime import datetime
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


REQUIRED_Z_COMPLIANCE_CONFIRMATION = "I_CONFIRM_REAL_Z_COMPLIANCE_TEST"
DEFAULT_TOOL0_TO_PEN_TIP_XYZ = [0.00079, -0.00076, 0.15172]
DEFAULT_PAYLOAD_MASS_KG = 0.085
DEFAULT_PAYLOAD_COG_XYZ = [0.0, 0.0, 0.0]


def _parse_float(context: LaunchContext, name: str):
    raw_value = context.perform_substitution(LaunchConfiguration(name))
    try:
        return float(raw_value), None
    except ValueError:
        return None, f"{name} must be a number, got {raw_value!r}"


def _parse_float_list(context: LaunchContext, name: str, size: int):
    raw_value = context.perform_substitution(LaunchConfiguration(name))
    try:
        values = [float(value) for value in yaml.safe_load(raw_value)]
    except (TypeError, ValueError, yaml.YAMLError):
        return None, f"{name} must be a list of {size} numbers"
    if len(values) != size:
        return None, f"{name} must contain {size} values"
    return values, None


def validate_z_compliance_configuration(values: dict[str, float | str]) -> str | None:
    if values["human_confirmation"] != REQUIRED_Z_COMPLIANCE_CONFIRMATION:
        return (
            "missing real Z-compliance confirmation; pass "
            f"human_confirmation:={REQUIRED_Z_COMPLIANCE_CONFIRMATION}"
        )

    bounds = {
        "payload_mass_kg": (0.0, 0.5),
        "target_force_n": (0.0, 1.0),
        "direction_force_n": (0.0, 0.5),
        "max_force_filtered_n": (0.0, 1.5),
        "max_force_raw_n": (0.0, 2.0),
        "max_z_speed_mps": (0.0, 0.0005),
        "max_acquire_travel_m": (0.0, 0.004),
        "max_contact_z_offset_m": (0.0, 0.0015),
        "max_xy_error_m": (0.0, 0.003),
        "max_rotation_error_rad": (0.0, math.radians(2.0)),
        "retract_distance_m": (0.0, 0.003),
        "line_length_m": (0.0, 0.01),
        "line_speed_mps": (0.0, 0.002),
        "cartesian_step_m": (0.0, 0.0005),
        "writing_width_m": (0.0, 0.01),
        "writing_height_m": (0.0, 0.01),
        "data_timeout_sec": (0.0, 1.0),
        "lost_contact_duration_sec": (0.0, 1.0),
        "baseline_duration_sec": (0.0, 5.0),
        "baseline_settle_sec": (0.0, 2.0),
        "max_baseline_stddev_n": (0.0, 0.1),
        "contact_settle_sec": (0.0, 2.0),
        "hold_duration_sec": (0.0, 5.0),
        "air_hold_duration_sec": (0.0, 2.0),
    }
    for name, (lower, upper) in bounds.items():
        value = float(values[name])
        if not lower < value <= upper:
            return f"{name} must be in ({lower}, {upper}], got {value}"

    if not 0.0 <= float(values["damping_factor"]) <= 1.0:
        return "damping_factor must be in [0, 1]"
    if not 0.0 <= float(values["path_simplify_tolerance_m"]) <= 0.001:
        return "path_simplify_tolerance_m must be in [0, 0.001]"
    if not 0.0 < float(values["gain_scaling"]) <= 1.0:
        return "gain_scaling must be in (0, 1]"
    if not (
        0.0
        <= float(values["lost_contact_force_n"])
        < float(values["steady_force_min_n"])
        < float(values["steady_force_max_n"])
        <= float(values["max_force_filtered_n"])
        < float(values["max_force_raw_n"])
    ):
        return (
            "force thresholds must satisfy lost_contact < steady_min < steady_max "
            "<= filtered_limit < raw_limit"
        )
    return None


def launch_setup(context: LaunchContext, *_args, **_kwargs):
    scalar_names = (
        "payload_mass_kg",
        "target_force_n",
        "direction_force_n",
        "max_force_filtered_n",
        "max_force_raw_n",
        "max_z_speed_mps",
        "damping_factor",
        "gain_scaling",
        "max_acquire_travel_m",
        "max_contact_z_offset_m",
        "max_xy_error_m",
        "max_rotation_error_rad",
        "steady_force_min_n",
        "steady_force_max_n",
        "lost_contact_force_n",
        "lost_contact_duration_sec",
        "retract_distance_m",
        "line_length_m",
        "line_speed_mps",
        "cartesian_step_m",
        "writing_width_m",
        "writing_height_m",
        "path_simplify_tolerance_m",
        "data_timeout_sec",
        "baseline_duration_sec",
        "baseline_settle_sec",
        "max_baseline_stddev_n",
        "contact_settle_sec",
        "hold_duration_sec",
        "air_hold_duration_sec",
    )
    values: dict[str, float | str] = {
        "human_confirmation": context.perform_substitution(
            LaunchConfiguration("human_confirmation")
        )
    }
    error = None
    for name in scalar_names:
        values[name], error = _parse_float(context, name)
        if error is not None:
            break

    payload_cog_xyz = None
    tool0_to_pen_tip_xyz = None
    if error is None:
        payload_cog_xyz, error = _parse_float_list(context, "payload_cog_xyz", 3)
    if error is None:
        tool0_to_pen_tip_xyz, error = _parse_float_list(
            context, "tool0_to_pen_tip_xyz", 3
        )
    if error is None:
        error = validate_z_compliance_configuration(values)
    if error is not None:
        return [
            LogInfo(msg=f"Refusing real Z-compliance launch: {error}"),
            EmitEvent(event=Shutdown(reason="Invalid real Z-compliance safety argument")),
        ]

    configured_log_directory = context.perform_substitution(
        LaunchConfiguration("log_directory")
    )
    session_log_directory = Path(configured_log_directory) if configured_log_directory else (
        Path.cwd()
        / "logs"
        / "force_pen_writing"
        / datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    session_log_directory.mkdir(parents=True, exist_ok=True)

    force_writing_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_force_pen_writing_py"),
                    "launch",
                    "real_force_writing_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "launch_rviz": "false",
            "launch_paper_seek": "true",
            "payload_mass_kg": str(values["payload_mass_kg"]),
            "payload_cog_xyz": str(payload_cog_xyz),
            "tool0_to_pen_tip_xyz": str(tool0_to_pen_tip_xyz),
            "wrench_topic": LaunchConfiguration("wrench_topic"),
            "log_directory": str(session_log_directory),
        }.items(),
    )

    validator = Node(
        package="ur3e_force_pen_writing_py",
        executable="z_compliance_validation_node",
        name="z_compliance_validation",
        output="screen",
        parameters=[
            {
                "human_confirmation": REQUIRED_Z_COMPLIANCE_CONFIRMATION,
                "base_frame": "base_link",
                "tool_frame": "tool0",
                "wrench_topic": LaunchConfiguration("wrench_topic"),
                "detected_point_topic": LaunchConfiguration(
                    "detected_point_topic"
                ),
                "payload_mass_kg": values["payload_mass_kg"],
                "payload_cog_xyz": payload_cog_xyz,
                "tool0_to_pen_tip_xyz": tool0_to_pen_tip_xyz,
                "log_directory": str(session_log_directory),
                "trajectory_file": LaunchConfiguration("trajectory_file"),
                **{
                    name: values[name]
                    for name in scalar_names
                    if name != "payload_mass_kg"
                },
            }
        ],
    )
    return [
        LogInfo(
            msg=(
                "Real Z-compliance validation confirmation accepted. No motion starts "
                "until a /pen_writing/z_compliance/start_* service is called."
            )
        ),
        SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(session_log_directory)),
        LogInfo(msg=f"Force-writing logs: {session_log_directory}"),
        force_writing_bringup,
        validator,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("reverse_ip", default_value="192.168.56.2"),
            DeclareLaunchArgument("human_confirmation", default_value=""),
            DeclareLaunchArgument(
                "wrench_topic",
                default_value="/force_torque_sensor_broadcaster/wrench",
            ),
            DeclareLaunchArgument(
                "detected_point_topic",
                default_value="/pen_writing/detected_paper_point",
            ),
            DeclareLaunchArgument(
                "payload_mass_kg", default_value=str(DEFAULT_PAYLOAD_MASS_KG)
            ),
            DeclareLaunchArgument(
                "payload_cog_xyz", default_value=str(DEFAULT_PAYLOAD_COG_XYZ)
            ),
            DeclareLaunchArgument(
                "tool0_to_pen_tip_xyz",
                default_value=str(DEFAULT_TOOL0_TO_PEN_TIP_XYZ),
            ),
            DeclareLaunchArgument("log_directory", default_value=""),
            DeclareLaunchArgument("target_force_n", default_value="0.8"),
            DeclareLaunchArgument("direction_force_n", default_value="0.2"),
            DeclareLaunchArgument("max_force_filtered_n", default_value="1.5"),
            DeclareLaunchArgument("max_force_raw_n", default_value="2.0"),
            DeclareLaunchArgument("max_z_speed_mps", default_value="0.0005"),
            DeclareLaunchArgument("damping_factor", default_value="0.5"),
            DeclareLaunchArgument("gain_scaling", default_value="0.3"),
            DeclareLaunchArgument("max_acquire_travel_m", default_value="0.004"),
            DeclareLaunchArgument(
                "max_contact_z_offset_m", default_value="0.0015"
            ),
            DeclareLaunchArgument("max_xy_error_m", default_value="0.003"),
            DeclareLaunchArgument(
                "max_rotation_error_rad", default_value=str(math.radians(2.0))
            ),
            DeclareLaunchArgument("steady_force_min_n", default_value="0.5"),
            DeclareLaunchArgument("steady_force_max_n", default_value="1.1"),
            DeclareLaunchArgument("lost_contact_force_n", default_value="0.2"),
            DeclareLaunchArgument(
                "lost_contact_duration_sec", default_value="0.3"
            ),
            DeclareLaunchArgument("retract_distance_m", default_value="0.003"),
            DeclareLaunchArgument("line_length_m", default_value="0.01"),
            DeclareLaunchArgument("line_speed_mps", default_value="0.002"),
            DeclareLaunchArgument("cartesian_step_m", default_value="0.0005"),
            DeclareLaunchArgument("trajectory_file", default_value=""),
            DeclareLaunchArgument("writing_width_m", default_value="0.01"),
            DeclareLaunchArgument("writing_height_m", default_value="0.01"),
            DeclareLaunchArgument(
                "path_simplify_tolerance_m", default_value="0.00025"
            ),
            DeclareLaunchArgument("data_timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("baseline_duration_sec", default_value="1.0"),
            DeclareLaunchArgument("baseline_settle_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "max_baseline_stddev_n", default_value="0.1"
            ),
            DeclareLaunchArgument("contact_settle_sec", default_value="1.0"),
            DeclareLaunchArgument("hold_duration_sec", default_value="5.0"),
            DeclareLaunchArgument("air_hold_duration_sec", default_value="2.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
