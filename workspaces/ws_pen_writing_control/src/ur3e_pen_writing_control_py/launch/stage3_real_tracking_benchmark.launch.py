import json
import math
import yaml

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


REQUIRED_CONFIRMATION = "I_CONFIRM_REAL_PEN_AIR_MOTION"
INITIAL_CONTROLLER = "scaled_joint_trajectory_controller"
SERVO_CONTROLLER = "forward_position_controller"
COMMAND_JOY_TOPIC = "/pen_writing/real_benchmark/joy"
JOINT_STATE_RELAY_PERIOD_SEC = 0.004
RAW_JOINT_STATES_TOPIC = "/joint_states"
FRESH_JOINT_STATES_TOPIC = "/task7e/joint_states_fresh"
MAX_SESSION_DURATION_SEC = 60.0
RVIZ_LAUNCH_CONFIG = "real_benchmark_launch_rviz"
STAGE3_SERVO_ROTATIONAL_SCALE_RADPS = math.tau


def load_yaml(package_name: str, relative_path: str):
    path = Path(get_package_share_directory(package_name)) / relative_path
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def reviewed_home_parameters() -> dict:
    document = load_yaml(
        "ur3_real_guarded_motion_lab_cpp",
        "config/task8D_guarded_targets.yaml",
    )
    parameters = document["task8d_guarded_joint_motion"]["ros__parameters"]
    return {
        "home_joint_names": list(parameters["home_joint_names"]),
        "home_positions_rad": list(parameters["home_positions_rad"]),
        "home_reviewed_by": str(parameters["home_reviewed_by"]),
        "max_velocity_scaling": 0.10,
        "max_acceleration_scaling": 0.10,
        "final_position_tolerance_rad": 0.02,
    }


def validate_benchmark_configuration(
    *,
    human_confirmation: str,
    max_session_duration_sec: float,
) -> str | None:
    if human_confirmation != REQUIRED_CONFIRMATION:
        return f"human_confirmation must be {REQUIRED_CONFIRMATION}"
    if max_session_duration_sec <= 0.0:
        return "max_session_duration_sec must be greater than zero"
    if max_session_duration_sec > MAX_SESSION_DURATION_SEC:
        return f"max_session_duration_sec must be <= {MAX_SESSION_DURATION_SEC}"
    home = reviewed_home_parameters()
    if len(home["home_joint_names"]) != 6 or len(home["home_positions_rad"]) != 6:
        return "reviewed Task8D home must contain exactly six joints"
    if not home["home_reviewed_by"]:
        return "reviewed Task8D home is missing home_reviewed_by"
    return None


def joint_state_relay_parameters() -> dict:
    return {
        "source_topic": RAW_JOINT_STATES_TOPIC,
        "target_topic": FRESH_JOINT_STATES_TOPIC,
        "publish_period_sec": JOINT_STATE_RELAY_PERIOD_SEC,
    }


def trajectory_gate_parameters(timeout_sec) -> dict:
    return {
        "topic": FRESH_JOINT_STATES_TOPIC,
        "timeout_sec": timeout_sec,
        "reliability": "best_effort",
        "required_active_controllers": [
            "joint_state_broadcaster",
            INITIAL_CONTROLLER,
        ],
    }


def move_group_parameters(moveit_config) -> list:
    moveit_parameters = (
        moveit_config.to_dict() if hasattr(moveit_config, "to_dict") else moveit_config
    )
    return [
        moveit_parameters,
        {
            "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
            "warehouse_host": str(Path.home() / ".ros" / "warehouse_ros.sqlite"),
        },
        {
            "use_sim_time": False,
            "publish_robot_description_semantic": True,
        },
    ]


def _bool_value(context: LaunchContext, name: str) -> bool:
    return context.perform_substitution(LaunchConfiguration(name)).lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _append_lifecycle(path: Path, event: str, **details) -> None:
    if path.exists():
        document = json.loads(path.read_text(encoding="utf-8"))
    else:
        document = {"events": []}
    document["events"].append(
        {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "event": event,
            **details,
        }
    )
    path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("robot_ip"),
        DeclareLaunchArgument("reverse_ip", default_value="192.168.56.2"),
        DeclareLaunchArgument("human_confirmation", default_value=""),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("prehome_only", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            "external_control_program",
            default_value="/programs/external_control.urp",
        ),
        DeclareLaunchArgument("servo_log_level", default_value="warn"),
        DeclareLaunchArgument("max_session_duration_sec", default_value="60.0"),
        DeclareLaunchArgument("hardware_ready_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("joint_states_wait_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("servo_status_wait_timeout_sec", default_value="30.0"),
        DeclareLaunchArgument("dashboard_receive_timeout", default_value="20.0"),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("joy_device_name", default_value=""),
        DeclareLaunchArgument("joy_deadzone", default_value="0.08"),
        DeclareLaunchArgument("joy_autorepeat_rate", default_value="100.0"),
        OpaqueFunction(function=launch_setup),
    ]
    return LaunchDescription(arguments)


def launch_setup(context: LaunchContext, *_args, **_kwargs):
    try:
        max_session_duration_sec = float(
            context.perform_substitution(
                LaunchConfiguration("max_session_duration_sec")
            )
        )
    except ValueError:
        max_session_duration_sec = -1.0
    error = validate_benchmark_configuration(
        human_confirmation=context.perform_substitution(
            LaunchConfiguration("human_confirmation")
        ),
        max_session_duration_sec=max_session_duration_sec,
    )
    if error is not None:
        return [
            LogInfo(msg=f"Refusing real tracking benchmark: {error}"),
            EmitEvent(event=Shutdown(reason="Invalid real benchmark configuration")),
        ]

    dry_run = _bool_value(context, "dry_run")
    prehome_only = _bool_value(context, "prehome_only")
    run_log_dir = Path.cwd().joinpath(
        "logs",
        "stage3_real_tracking_benchmark",
        datetime.now().strftime("%Y%m%d-%H%M%S"),
    )
    run_log_dir.mkdir(parents=True, exist_ok=True)
    lifecycle_path = run_log_dir / "benchmark_lifecycle.json"
    runtime_state = {"benchmark_active": False, "returning_home": False}
    _append_lifecycle(
        lifecycle_path,
        "launch_started",
        dry_run=dry_run,
        prehome_only=prehome_only,
    )

    home = reviewed_home_parameters()
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro",
            {"name": LaunchConfiguration("ur_type")},
        )
        .to_moveit_configs()
    )
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_yaml["joint_topic"] = FRESH_JOINT_STATES_TOPIC
    servo_yaml["scale"]["rotational"] = STAGE3_SERVO_ROTATIONAL_SCALE_RADPS

    description_launchfile = PathJoinSubstitution(
        [
            FindPackageShare("ur3_real_bringup_lab"),
            "launch",
            "task8B_real_calibrated_rsp.launch.py",
        ]
    )
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "use_mock_hardware": "false",
            "initial_joint_controller": INITIAL_CONTROLLER,
            "activate_joint_controller": "true",
            "launch_rviz": "false",
            "launch_dashboard_client": "false",
            "description_launchfile": description_launchfile,
        }.items(),
    )
    hardware_gate = Node(
        package="ur3_real_bringup_lab",
        executable="wait_for_hardware_ready.py",
        name="real_benchmark_hardware_gate",
        output="both",
        parameters=[
            {
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("hardware_ready_timeout_sec"),
                    value_type=float,
                ),
                "expected_joint_names": home["home_joint_names"],
            }
        ],
    )
    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_dashboard_client.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "dashboard_receive_timeout": LaunchConfiguration(
                "dashboard_receive_timeout"
            ),
        }.items(),
    )
    external_control_manager = Node(
        package="ur3_real_bringup_lab",
        executable="manage_external_control.py",
        name="real_benchmark_external_control_manager",
        output="both",
        parameters=[
            {
                "program_path": LaunchConfiguration("external_control_program"),
                "require_remote_control": True,
                "startup_timeout_sec": ParameterValue(
                    LaunchConfiguration("dashboard_receive_timeout"),
                    value_type=float,
                ),
                "stop_on_shutdown": True,
            }
        ],
    )
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        name="real_benchmark_wait_robot_description",
        output="screen",
    )
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=move_group_parameters(moveit_config),
        remappings=[(RAW_JOINT_STATES_TOPIC, FRESH_JOINT_STATES_TOPIC)],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_pen_real_benchmark",
        condition=IfCondition(LaunchConfiguration(RVIZ_LAUNCH_CONFIG)),
        output="both",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_pen_writing_control_py"),
                    "rviz",
                    "stage2_fakehardware_pen_servo.rviz",
                ]
            ),
        ],
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
    )
    trajectory_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_joint_states.py",
        name="real_benchmark_trajectory_gate",
        output="both",
        parameters=[
            trajectory_gate_parameters(
                LaunchConfiguration("joint_states_wait_timeout_sec")
            )
        ],
    )

    def home_node(phase: str, result_name: str):
        parameters = {
            **home,
            "phase": phase,
            "execute": not dry_run,
            "human_confirmation": LaunchConfiguration("human_confirmation"),
            "planning_group": "ur_manipulator",
            "planning_time_sec": 5.0,
            "planning_attempts": 5,
            "current_state_timeout_sec": 3.0,
            "result_path": str(run_log_dir / result_name),
        }
        return Node(
            package="ur3_real_guarded_motion_lab_cpp",
            executable="planned_home_motion_node",
            name=f"real_benchmark_{phase}",
            output="both",
            parameters=[moveit_config.to_dict(), parameters],
            remappings=[(RAW_JOINT_STATES_TOPIC, FRESH_JOINT_STATES_TOPIC)],
        )

    pre_home = home_node("pre_home", "pre_home_result.json")
    post_home = home_node("post_home", "post_home_result.json")

    def switch_node(activate: str, deactivate: str, result_name: str):
        return Node(
            package="ur3e_pen_writing_control_py",
            executable="controller_switch_once_node",
            name=f"switch_{deactivate}_to_{activate}",
            output="screen",
            parameters=[
                {
                    "activate_controllers": [activate],
                    "deactivate_controllers": [deactivate],
                    "timeout_sec": 10.0,
                    "result_path": str(run_log_dir / result_name),
                }
            ],
        )

    switch_to_servo = switch_node(
        SERVO_CONTROLLER,
        INITIAL_CONTROLLER,
        "switch_to_servo_result.json",
    )
    switch_to_trajectory = switch_node(
        INITIAL_CONTROLLER,
        SERVO_CONTROLLER,
        "switch_to_trajectory_result.json",
    )
    joint_state_relay = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="joint_state_stamp_relay_node",
        name="real_benchmark_joint_state_stamp_relay",
        output="both",
        parameters=[joint_state_relay_parameters()],
    )
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="both",
        parameters=[moveit_config.to_dict(), {"moveit_servo": servo_yaml}],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("servo_log_level"),
        ],
    )
    servo_status_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_servo_status.py",
        name="real_benchmark_servo_status_gate",
        output="both",
        parameters=[
            {
                "topic": "/servo_node/status",
                "timeout_sec": LaunchConfiguration("servo_status_wait_timeout_sec"),
            }
        ],
    )
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="real_benchmark_operator_joy",
        output="both",
        parameters=[
            {
                "device_id": ParameterValue(
                    LaunchConfiguration("joy_device_id"),
                    value_type=int,
                ),
                "device_name": LaunchConfiguration("joy_device_name"),
                "deadzone": ParameterValue(
                    LaunchConfiguration("joy_deadzone"),
                    value_type=float,
                ),
                "autorepeat_rate": ParameterValue(
                    LaunchConfiguration("joy_autorepeat_rate"),
                    value_type=float,
                ),
                "sticky_buttons": False,
            }
        ],
    )
    pen_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="pen_fakehardware_servo_node",
        name="pen_real_benchmark_servo",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_pen_writing_control_py"),
                    "config",
                    "pen_tool_model.yaml",
                ]
            ),
            {
                "base_frame": "base_link",
                "paper_frame": "paper_frame",
                "tool_frame": "tool0",
                "start_from_current_tool0": True,
                "require_motion_before_pose_command": True,
                "paper_origin_xyz": [0.45, 0.0, 0.12],
                "tool0_to_pen_tip_xyz": [0.0, 0.0, 0.14],
                "servo_status_topic": "/servo_node/status",
                "servo_status_timeout_sec": 1.0,
                "max_session_duration_sec": max_session_duration_sec,
                "max_planar_speed_mps": 0.03,
                "tilt_rate_degps": 10.0,
                "untilt_rate_degps": 12.0,
                "max_pen_axis_angular_speed_degps": 12.0,
                "joy_topic": COMMAND_JOY_TOPIC,
                "alignment_error_log_path": str(
                    run_log_dir / "tool_alignment_error.csv"
                ),
            },
        ],
    )
    benchmark_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="pen_real_tracking_benchmark_node",
        name="pen_real_tracking_benchmark",
        output="screen",
        parameters=[
            {
                "command_joy_topic": COMMAND_JOY_TOPIC,
                "operator_joy_topic": "/joy",
                "servo_status_topic": "/servo_node/status",
                "alignment_error_log_path": str(
                    run_log_dir / "tool_alignment_error.csv"
                ),
                "summary_json_path": str(run_log_dir / "tracking_summary.json"),
                "summary_markdown_path": str(run_log_dir / "tracking_summary.md"),
                "result_path": str(run_log_dir / "benchmark_result.json"),
                "publish_rate_hz": 50.0,
            }
        ],
    )

    def shutdown(reason: str, event: str, returncode=None):
        _append_lifecycle(
            lifecycle_path,
            event,
            returncode=returncode,
            action="shutdown_without_return_home",
        )
        return [EmitEvent(event=Shutdown(reason=reason))]

    def on_hardware_exit(event, _context):
        if event.returncode != 0:
            return shutdown("Hardware gate failed.", "hardware_gate_failed", event.returncode)
        _append_lifecycle(lifecycle_path, "hardware_gate_passed")
        return [
            dashboard_launch,
            external_control_manager,
            joint_state_relay,
            wait_robot_description,
            rviz_node,
            TimerAction(period=3.0, actions=[trajectory_gate]),
        ]

    def on_trajectory_gate_exit(event, _context):
        if event.returncode != 0:
            return shutdown(
                "Trajectory controller gate failed.",
                "trajectory_gate_failed",
                event.returncode,
            )
        _append_lifecycle(lifecycle_path, "pre_home_started")
        return [pre_home]

    def on_pre_home_exit(event, _context):
        if event.returncode != 0:
            return shutdown("Pre-home failed.", "pre_home_failed", event.returncode)
        _append_lifecycle(lifecycle_path, "pre_home_completed")
        if dry_run or prehome_only:
            return [EmitEvent(event=Shutdown(reason="Pre-home validation completed."))]
        return [switch_to_servo]

    def on_switch_to_servo_exit(event, _context):
        if event.returncode != 0:
            return shutdown(
                "Switch to Servo controller failed.",
                "switch_to_servo_failed",
                event.returncode,
            )
        _append_lifecycle(lifecycle_path, "servo_controller_active")
        return [
            servo_node,
            servo_status_gate,
        ]

    def on_servo_gate_exit(event, _context):
        if event.returncode != 0:
            return shutdown(
                "Servo status gate failed.",
                "servo_status_gate_failed",
                event.returncode,
            )
        _append_lifecycle(lifecycle_path, "benchmark_started")
        runtime_state["benchmark_active"] = True
        return [joy_node, pen_node, benchmark_node]

    def on_benchmark_exit(event, _context):
        if event.returncode != 0:
            runtime_state["benchmark_active"] = False
            return shutdown(
                "Benchmark safety or infrastructure abort.",
                "benchmark_aborted",
                event.returncode,
            )
        _append_lifecycle(lifecycle_path, "benchmark_completed")
        runtime_state["benchmark_active"] = False
        runtime_state["returning_home"] = True
        return [switch_to_trajectory]

    def on_switch_to_trajectory_exit(event, _context):
        if event.returncode != 0:
            return shutdown(
                "Switch to trajectory controller failed.",
                "switch_to_trajectory_failed",
                event.returncode,
            )
        _append_lifecycle(lifecycle_path, "post_home_started")
        return [post_home]

    def on_post_home_exit(event, _context):
        if event.returncode != 0:
            return shutdown("Post-home failed.", "post_home_failed", event.returncode)
        _append_lifecycle(
            lifecycle_path,
            "post_home_completed",
            action="normal_shutdown",
        )
        runtime_state["returning_home"] = False
        return [EmitEvent(event=Shutdown(reason="Real benchmark completed and returned home."))]

    def on_runtime_process_exit(label: str):
        def callback(event, _context):
            if runtime_state["benchmark_active"]:
                runtime_state["benchmark_active"] = False
                return shutdown(
                    f"{label} exited during benchmark.",
                    f"{label}_exited",
                    event.returncode,
                )
            return []

        return callback

    return [
        SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(run_log_dir)),
        SetEnvironmentVariable(name="RCUTILS_LOGGING_BUFFERED_STREAM", value="1"),
        SetLaunchConfiguration(
            name=RVIZ_LAUNCH_CONFIG,
            value=LaunchConfiguration("launch_rviz"),
        ),
        LogInfo(msg=f"Real tracking benchmark logs: {run_log_dir}"),
        LogInfo(
            msg=(
                "Real benchmark confirmed. Keep E-stop reachable. "
                "A freezes without return-home; B requests controlled return-home."
            )
        ),
        driver_launch,
        hardware_gate,
        RegisterEventHandler(
            OnProcessExit(target_action=hardware_gate, on_exit=on_hardware_exit)
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=trajectory_gate, on_exit=on_trajectory_gate_exit)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=pre_home, on_exit=on_pre_home_exit)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=switch_to_servo, on_exit=on_switch_to_servo_exit)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=servo_status_gate, on_exit=on_servo_gate_exit)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=benchmark_node, on_exit=on_benchmark_exit)
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=switch_to_trajectory,
                on_exit=on_switch_to_trajectory_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=post_home, on_exit=on_post_home_exit)
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=servo_node,
                on_exit=on_runtime_process_exit("servo_node"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=pen_node,
                on_exit=on_runtime_process_exit("pen_node"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joy_node,
                on_exit=on_runtime_process_exit("operator_joy"),
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=external_control_manager,
                on_exit=lambda event, _context: (
                    []
                    if event.returncode == 0
                    else shutdown(
                        "External Control manager failed.",
                        "external_control_failed",
                        event.returncode,
                    )
                ),
            )
        ),
    ]
