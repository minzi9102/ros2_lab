from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


REQUIRED_FORCE_CONFIRMATION = "I_CONFIRM_REAL_FORCE_MODE_TEST"


def validate_force_launch(context, *_args, **_kwargs):
    confirmation = context.perform_substitution(
        LaunchConfiguration("human_confirmation")
    )
    if confirmation != REQUIRED_FORCE_CONFIRMATION:
        return [
            LogInfo(
                msg=(
                    "Refusing force test: human_confirmation must be "
                    f"{REQUIRED_FORCE_CONFIRMATION}"
                )
            ),
            EmitEvent(event=Shutdown(reason="Missing real Force Mode confirmation")),
        ]
    configured = context.perform_substitution(LaunchConfiguration("log_directory"))
    session = Path(configured) if configured else (
        Path.cwd()
        / "logs"
        / "force_pen_writing"
        / datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    session.mkdir(parents=True, exist_ok=True)
    return [
        SetLaunchConfiguration("session_log_directory", str(session)),
        SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(session)),
        LogInfo(msg=f"Force-mode validation logs: {session}"),
    ]


def generate_launch_description() -> LaunchDescription:
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
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_paper_seek": "false",
            "log_directory": LaunchConfiguration("session_log_directory"),
        }.items(),
    )
    validator = Node(
        package="ur3e_force_pen_writing_py",
        executable="force_mode_validation_node",
        name="force_mode_validation",
        output="screen",
        parameters=[
            {
                "human_confirmation": REQUIRED_FORCE_CONFIRMATION,
                "base_frame": "base",
                "tool_frame": "tool0_controller",
                "servo_base_frame": "base_link",
                "servo_tool_frame": "tool0",
                "max_speed_mps": 0.002,
                "max_force_n": 10.0,
                "retract_distance_m": 0.003,
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("reverse_ip", default_value="192.168.56.2"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("human_confirmation", default_value=""),
            DeclareLaunchArgument("log_directory", default_value=""),
            OpaqueFunction(function=validate_force_launch),
            force_writing_bringup,
            validator,
        ]
    )
