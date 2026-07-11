from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
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
    if confirmation == REQUIRED_FORCE_CONFIRMATION:
        return []
    return [
        LogInfo(
            msg=(
                "Refusing force test: human_confirmation must be "
                f"{REQUIRED_FORCE_CONFIRMATION}"
            )
        ),
        EmitEvent(event=Shutdown(reason="Missing real Force Mode confirmation")),
    ]


def generate_launch_description() -> LaunchDescription:
    real_pen_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_pen_writing_control_py"),
                    "launch",
                    "stage3_real_air_pen_servo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "human_confirmation": "I_CONFIRM_REAL_PEN_AIR_MOTION",
            "real_pen_air_launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_pen_node": "false",
            "launch_joy_node": "false",
            "launch_pen_tip_plane_monitor": "false",
            "max_session_duration_sec": "60.0",
        }.items(),
    )
    validator = Node(
        package="ur3e_pen_writing_control_py",
        executable="force_mode_validation_node",
        name="force_mode_validation",
        output="screen",
        parameters=[
            {
                "human_confirmation": LaunchConfiguration("human_confirmation"),
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
            OpaqueFunction(function=validate_force_launch),
            real_pen_launch,
            validator,
        ]
    )
