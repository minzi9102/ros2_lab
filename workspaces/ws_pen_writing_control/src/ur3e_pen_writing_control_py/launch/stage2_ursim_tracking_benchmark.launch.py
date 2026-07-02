import signal
from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import SignalProcess
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
    chain_split_bag_dir = (
        Path.cwd()
        / "logs"
        / f"tmp_chain_split_ursim_{datetime.now().strftime('%Y%m%d-%H%M%S')}"
        / "bag"
    )
    chain_split_bag_dir.parent.mkdir(parents=True, exist_ok=True)

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
    servo_publish_period_arg = DeclareLaunchArgument(
        "servo_publish_period_sec",
        default_value="0.004",
        description="MoveIt Servo output publish period in seconds.",
    )
    servo_linear_scale_arg = DeclareLaunchArgument(
        "servo_linear_scale",
        default_value="0.6",
        description="MoveIt Servo linear scale for Cartesian and pose tracking commands.",
    )
    servo_output_controller_arg = DeclareLaunchArgument(
        "servo_output_controller",
        default_value="forward_position_controller",
        description=(
            "Servo output controller: forward_position_controller or "
            "joint_trajectory_controller."
        ),
    )
    pose_target_publish_rate_arg = DeclareLaunchArgument(
        "pose_target_publish_rate_hz",
        default_value="60.0",
        description="Publish rate of the virtual pen pose target.",
    )
    max_planar_speed_arg = DeclareLaunchArgument(
        "max_planar_speed_mps",
        default_value="0.03",
        description="Maximum virtual pen-tip planar speed in meters per second.",
    )
    paper_width_arg = DeclareLaunchArgument(
        "paper_width_m",
        default_value="0.24",
        description="Virtual paper width in meters.",
    )
    paper_height_arg = DeclareLaunchArgument(
        "paper_height_m",
        default_value="0.16",
        description="Virtual paper height in meters.",
    )
    initial_tip_x_arg = DeclareLaunchArgument(
        "initial_tip_x_m",
        default_value="0.0",
        description="Initial virtual pen-tip X coordinate in the paper frame.",
    )
    initial_tip_y_arg = DeclareLaunchArgument(
        "initial_tip_y_m",
        default_value="0.0",
        description="Initial virtual pen-tip Y coordinate in the paper frame.",
    )
    fixed_tilt_deg_arg = DeclareLaunchArgument(
        "fixed_tilt_deg",
        default_value="20.0",
        description="Virtual pen tilt angle used outside fixed-vertical diagnostics.",
    )
    diagnostic_freeze_tip_xy_arg = DeclareLaunchArgument(
        "diagnostic_freeze_tip_xy",
        default_value="false",
        description="Freeze pen tip XY while still updating orientation diagnostics.",
    )
    diagnostic_orientation_mode_arg = DeclareLaunchArgument(
        "diagnostic_orientation_mode",
        default_value="dynamic",
        description="Pen orientation mode: dynamic or fixed_vertical.",
    )
    joint_state_relay_period_arg = DeclareLaunchArgument(
        "joint_state_relay_period_sec",
        default_value="0.020",
    )
    servo_command_mode_arg = DeclareLaunchArgument(
        "servo_command_mode",
        default_value="pose",
        description="Servo command mode: pose, twist_feedforward, or twist_linear_only.",
    )
    twist_position_gain_arg = DeclareLaunchArgument(
        "twist_position_gain",
        default_value="2.0",
    )
    twist_orientation_gain_arg = DeclareLaunchArgument(
        "twist_orientation_gain",
        default_value="2.0",
    )
    twist_linear_correction_limit_arg = DeclareLaunchArgument(
        "twist_linear_correction_limit_mps",
        default_value="0.03",
    )
    twist_angular_correction_limit_arg = DeclareLaunchArgument(
        "twist_angular_correction_limit_radps",
        default_value="0.3",
    )

    joy_topic = "/pen_writing/benchmark/joy"
    alignment_csv = str(run_log_dir / "tool_alignment_error.csv")
    summary_json = str(run_log_dir / "tracking_summary.json")
    summary_md = str(run_log_dir / "tracking_summary.md")
    result_json = str(run_log_dir / "benchmark_result.json")
    chain_split_json = str(run_log_dir / "chain_split_fk_report.json")
    chain_split_md = str(run_log_dir / "chain_split_fk_report.md")
    urdf_xacro = str(
        Path.cwd()
        / "workspaces"
        / "ws_stage3"
        / "src"
        / "ur3_moveit_servo_lab_cpp"
        / "urdf"
        / "task7E_ur.urdf.xacro"
    )

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
            "stop_external_control_on_shutdown": "true",
            "external_control_program": LaunchConfiguration("external_control_program"),
            "launch_joy_node": "false",
            "joy_topic": joy_topic,
            "run_log_dir": str(run_log_dir),
            "joint_states_wait_timeout_sec": "60.0",
            "servo_status_wait_timeout_sec": "30.0",
            "servo_linear_scale": LaunchConfiguration("servo_linear_scale"),
            "servo_output_controller": LaunchConfiguration(
                "servo_output_controller"
            ),
            "servo_low_pass_filter_coeff": LaunchConfiguration(
                "servo_low_pass_filter_coeff"
            ),
            "servo_publish_period_sec": LaunchConfiguration(
                "servo_publish_period_sec"
            ),
            "pose_target_publish_rate_hz": LaunchConfiguration(
                "pose_target_publish_rate_hz"
            ),
            "max_planar_speed_mps": LaunchConfiguration("max_planar_speed_mps"),
            "paper_width_m": LaunchConfiguration("paper_width_m"),
            "paper_height_m": LaunchConfiguration("paper_height_m"),
            "initial_tip_x_m": LaunchConfiguration("initial_tip_x_m"),
            "initial_tip_y_m": LaunchConfiguration("initial_tip_y_m"),
            "fixed_tilt_deg": LaunchConfiguration("fixed_tilt_deg"),
            "diagnostic_freeze_tip_xy": LaunchConfiguration(
                "diagnostic_freeze_tip_xy"
            ),
            "diagnostic_orientation_mode": LaunchConfiguration(
                "diagnostic_orientation_mode"
            ),
            "joint_state_relay_period_sec": LaunchConfiguration(
                "joint_state_relay_period_sec"
            ),
            "servo_command_mode": LaunchConfiguration("servo_command_mode"),
            "twist_position_gain": LaunchConfiguration("twist_position_gain"),
            "twist_orientation_gain": LaunchConfiguration(
                "twist_orientation_gain"
            ),
            "twist_linear_correction_limit_mps": LaunchConfiguration(
                "twist_linear_correction_limit_mps"
            ),
            "twist_angular_correction_limit_radps": LaunchConfiguration(
                "twist_angular_correction_limit_radps"
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

    rosbag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--storage",
            "mcap",
            "--output",
            str(chain_split_bag_dir),
            "/pen_writing/target_pose",
            "/servo_node/pose_target_cmds",
            "/servo_node/delta_twist_cmds",
            "/forward_position_controller/commands",
            "/joint_trajectory_controller/joint_trajectory",
            "/joint_states",
        ],
        output="log",
    )
    chain_split_started = {"value": False}

    def make_chain_split_report():
        return ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "ur3e_pen_writing_control_py",
                "chain_split_fk_report",
                "--bag",
                str(chain_split_bag_dir),
                "--summary-json",
                summary_json,
                "--urdf-xacro",
                urdf_xacro,
                "--ur-type",
                "ur3",
                "--output-json",
                chain_split_json,
                "--output-md",
                chain_split_md,
            ],
            output="screen",
        )

    def on_benchmark_exit(event, _context):
        return [
            LogInfo(
                msg=(
                    "URSim tracking benchmark exited with return code "
                    f"{event.returncode}. Stopping chain-split rosbag at "
                    f"{chain_split_bag_dir}"
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rosbag_record,
                    on_exit=on_rosbag_exit,
                )
            ),
            EmitEvent(
                event=SignalProcess(
                    signal_number=signal.SIGINT,
                    process_matcher=lambda process: process == rosbag_record,
                )
            ),
        ]

    def on_rosbag_exit(event, _context):
        if chain_split_started["value"]:
            return []
        chain_split_started["value"] = True
        chain_split_report = make_chain_split_report()
        return [
            LogInfo(
                msg=(
                    "Chain-split rosbag exited with return code "
                    f"{event.returncode}. Generating FK report."
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=chain_split_report,
                    on_exit=on_chain_split_exit,
                )
            ),
            chain_split_report,
        ]

    def on_chain_split_exit(event, _context):
        return [
            LogInfo(
                msg=(
                    "URSim tracking benchmark artifacts ready. "
                    f"Summary: {summary_md} Chain split: {chain_split_md}"
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "URSim tracking benchmark and chain-split postprocess "
                        f"completed with return code {event.returncode}."
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
            servo_linear_scale_arg,
            servo_output_controller_arg,
            servo_low_pass_filter_coeff_arg,
            servo_publish_period_arg,
            pose_target_publish_rate_arg,
            max_planar_speed_arg,
            paper_width_arg,
            paper_height_arg,
            initial_tip_x_arg,
            initial_tip_y_arg,
            fixed_tilt_deg_arg,
            diagnostic_freeze_tip_xy_arg,
            diagnostic_orientation_mode_arg,
            joint_state_relay_period_arg,
            servo_command_mode_arg,
            twist_position_gain_arg,
            twist_orientation_gain_arg,
            twist_linear_correction_limit_arg,
            twist_angular_correction_limit_arg,
            SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(run_log_dir)),
            LogInfo(msg=f"URSim tracking benchmark logs will be written to: {run_log_dir}"),
            LogInfo(msg=f"URSim chain-split rosbag will be written to: {chain_split_bag_dir}"),
            rosbag_record,
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
