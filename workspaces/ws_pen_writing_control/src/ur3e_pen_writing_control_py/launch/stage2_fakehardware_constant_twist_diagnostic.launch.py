from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    run_log_dir = (
        Path.cwd()
        / "logs"
        / "stage2_fakehardware_constant_twist_diagnostic"
        / datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    run_log_dir.mkdir(parents=True, exist_ok=True)

    twist_profile_arg = DeclareLaunchArgument(
        "twist_profile",
        default_value="pure_x",
        description="Constant Twist profile: pure_x, pure_y, or pure_yaw.",
    )
    launch_rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="false")
    verbose_runtime_logs_arg = DeclareLaunchArgument(
        "verbose_runtime_logs",
        default_value="false",
    )
    joint_state_relay_period_arg = DeclareLaunchArgument(
        "joint_state_relay_period_sec",
        default_value="0.004",
    )
    servo_use_smoothing_arg = DeclareLaunchArgument(
        "servo_use_smoothing",
        default_value="true",
    )
    servo_butterworth_filter_coeff_arg = DeclareLaunchArgument(
        "servo_butterworth_filter_coeff",
        default_value="1.5",
    )
    duration_arg = DeclareLaunchArgument("duration_sec", default_value="5.0")
    publish_rate_arg = DeclareLaunchArgument("publish_rate_hz", default_value="125.0")

    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("ur3_moveit_servo_lab_cpp"),
                        "urdf",
                        "task7E_ur.urdf.xacro",
                    ]
                ),
                " ",
                "name:=ur",
                " ",
                "ur_type:=ur3",
                " ",
                "robot_ip:=192.168.56.101",
                " ",
                "use_mock_hardware:=true",
            ]
        )
    }

    stage2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_pen_writing_control_py"),
                    "launch",
                    "stage2_fakehardware_pen_servo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "verbose_runtime_logs": LaunchConfiguration("verbose_runtime_logs"),
            "launch_joy_node": "false",
            "launch_pen_node": "false",
            "run_log_dir": str(run_log_dir),
            "joint_state_relay_period_sec": LaunchConfiguration(
                "joint_state_relay_period_sec"
            ),
            "servo_use_smoothing": LaunchConfiguration("servo_use_smoothing"),
            "servo_butterworth_filter_coeff": LaunchConfiguration(
                "servo_butterworth_filter_coeff"
            ),
        }.items(),
    )

    diagnostic_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="constant_twist_diagnostic_node",
        name="constant_twist_diagnostic",
        output="screen",
        parameters=[
            robot_description,
            {
                "twist_profile": LaunchConfiguration("twist_profile"),
                "duration_sec": ParameterValue(
                    LaunchConfiguration("duration_sec"),
                    value_type=float,
                ),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"),
                    value_type=float,
                ),
                "report_json_path": str(run_log_dir / "constant_twist_report.json"),
                "report_markdown_path": str(run_log_dir / "constant_twist_report.md"),
            },
        ],
    )

    def on_diagnostic_exit(event, _context):
        return [
            LogInfo(
                msg=(
                    "Constant Twist diagnostic exited with return code "
                    f"{event.returncode}. Results: "
                    f"{run_log_dir / 'constant_twist_report.md'}"
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "Constant Twist diagnostic completed with return code "
                        f"{event.returncode}."
                    )
                )
            ),
        ]

    return LaunchDescription(
        [
            twist_profile_arg,
            launch_rviz_arg,
            verbose_runtime_logs_arg,
            joint_state_relay_period_arg,
            servo_use_smoothing_arg,
            servo_butterworth_filter_coeff_arg,
            duration_arg,
            publish_rate_arg,
            SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(run_log_dir)),
            LogInfo(
                msg=f"Constant Twist diagnostic logs will be written to: {run_log_dir}"
            ),
            stage2_launch,
            diagnostic_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=diagnostic_node,
                    on_exit=on_diagnostic_exit,
                )
            ),
        ]
    )
