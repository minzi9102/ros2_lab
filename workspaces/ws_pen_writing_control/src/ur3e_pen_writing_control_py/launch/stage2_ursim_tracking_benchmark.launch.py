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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    run_log_dir = (
        Path.cwd()
        / "logs"
        / "stage2_ursim_tracking_benchmark"
        / datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    run_log_dir.mkdir(parents=True, exist_ok=True)

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz during the tracking benchmark.",
    )
    verbose_runtime_logs_arg = DeclareLaunchArgument(
        "verbose_runtime_logs",
        default_value="false",
        description="Forward verbose runtime logs to terminal when true.",
    )
    benchmark_profile_arg = DeclareLaunchArgument(
        "benchmark_profile",
        default_value="eight_direction",
        description="Benchmark profile aligned with the real-robot benchmark.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="172.17.0.2",
        description="URSim robot IP address.",
    )
    external_control_program_arg = DeclareLaunchArgument(
        "external_control_program",
        default_value="/ursim/programs/123.urp",
        description="Dashboard path of the URSim External Control program to load automatically.",
    )
    servo_low_pass_filter_coeff_arg = DeclareLaunchArgument(
        "servo_low_pass_filter_coeff",
        default_value="10.0",
        description="MoveIt Servo joint-state low-pass filter coefficient.",
    )

    joy_topic = "/pen_writing/benchmark/joy"
    alignment_csv = str(run_log_dir / "tool_alignment_error.csv")
    summary_json = str(run_log_dir / "tracking_summary.json")
    summary_md = str(run_log_dir / "tracking_summary.md")
    result_json = str(run_log_dir / "benchmark_result.json")

    stage2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_pen_writing_control_py"),
                    "launch",
                    "stage2_ursim_pen_servo.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "verbose_runtime_logs": LaunchConfiguration("verbose_runtime_logs"),
            "auto_start_external_control": "true",
            "external_control_program": LaunchConfiguration("external_control_program"),
            "launch_joy_node": "false",
            "joy_topic": joy_topic,
            "run_log_dir": str(run_log_dir),
            "joint_states_wait_timeout_sec": "60.0",
            "servo_status_wait_timeout_sec": "30.0",
            "servo_low_pass_filter_coeff": LaunchConfiguration(
                "servo_low_pass_filter_coeff"
            ),
        }.items(),
    )

    benchmark_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="pen_real_tracking_benchmark_node",
        name="pen_ursim_tracking_benchmark",
        output="screen",
        parameters=[
            {
                "command_joy_topic": joy_topic,
                "operator_joy_topic": "/joy",
                "servo_status_topic": "/servo_node/status",
                "alignment_error_log_path": alignment_csv,
                "summary_json_path": summary_json,
                "summary_markdown_path": summary_md,
                "result_path": result_json,
                "publish_rate_hz": 50.0,
                "benchmark_profile": LaunchConfiguration("benchmark_profile"),
                "ready_timeout_sec": 120.0,
                "arm_timeout_sec": 10.0,
                "alignment_ready_timeout_sec": 30.0,
            }
        ],
    )

    def on_benchmark_exit(event, _context):
        return [
            LogInfo(
                msg=(
                    "URSim tracking benchmark exited with return code "
                    f"{event.returncode}. Results: {summary_md}"
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "URSim tracking benchmark completed with return code "
                        f"{event.returncode}."
                    )
                )
            ),
        ]

    return LaunchDescription(
        [
            launch_rviz_arg,
            verbose_runtime_logs_arg,
            benchmark_profile_arg,
            robot_ip_arg,
            external_control_program_arg,
            servo_low_pass_filter_coeff_arg,
            SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(run_log_dir)),
            LogInfo(msg=f"URSim tracking benchmark logs will be written to: {run_log_dir}"),
            stage2_launch,
            benchmark_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=benchmark_node,
                    on_exit=on_benchmark_exit,
                )
            ),
        ]
    )
