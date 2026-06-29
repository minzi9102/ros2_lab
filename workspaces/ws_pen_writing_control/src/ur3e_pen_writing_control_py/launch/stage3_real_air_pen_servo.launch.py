import math
import os
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


REQUIRED_REAL_AIR_CONFIRMATION = "I_CONFIRM_REAL_PEN_AIR_MOTION"
STAGE3_SERVO_ROTATIONAL_SCALE_RADPS = math.tau
STAGE3_REAL_DEFAULT_MAX_SESSION_DURATION_SEC = 30.0
STAGE3_REAL_MAX_SESSION_DURATION_SEC = 60.0
STAGE3_REAL_DEFAULT_USE_MOCK_HARDWARE = "false"
STAGE3_REAL_INITIAL_JOINT_CONTROLLER = "forward_position_controller"
STAGE3_REAL_DESCRIPTION_LAUNCHFILE_NAME = "task8B_real_calibrated_rsp.launch.py"
STAGE3_REAL_PAPER_ORIGIN_XYZ = [0.45, 0.0, 0.12]
STAGE3_REAL_TOOL0_TO_PEN_TIP_XYZ = [0.0, 0.0, 0.14]
STAGE3_JOINT_STATE_RELAY_PERIOD_SEC = 0.004
STAGE3_REAL_FAKEHARDWARE_MATCHED_PEN_PARAMS = {
    "max_planar_speed_mps": 0.03,
    "tilt_rate_degps": 10.0,
    "untilt_rate_degps": 12.0,
    "max_pen_axis_angular_speed_degps": 12.0,
}


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def parse_positive_bounded_float(
    *,
    context: LaunchContext,
    argument_name: str,
    max_value: float,
):
    raw_value = context.perform_substitution(LaunchConfiguration(argument_name))
    try:
        value = float(raw_value)
    except ValueError:
        return None, f"{argument_name} must be a number, got {raw_value!r}"

    if value <= 0.0:
        return None, f"{argument_name} must be greater than 0.0, got {value}"
    if value > max_value:
        return None, f"{argument_name} must be <= {max_value}, got {value}"
    return value, None


def validate_real_air_configuration(
    *,
    human_confirmation: str,
    max_session_duration_sec: float,
) -> str | None:
    if human_confirmation != REQUIRED_REAL_AIR_CONFIRMATION:
        return (
            "missing real robot pen air-motion confirmation; pass "
            f"human_confirmation:={REQUIRED_REAL_AIR_CONFIRMATION}"
        )
    if max_session_duration_sec <= 0.0:
        return "max_session_duration_sec must be greater than 0.0"
    if max_session_duration_sec > STAGE3_REAL_MAX_SESSION_DURATION_SEC:
        return (
            "max_session_duration_sec must be <= "
            f"{STAGE3_REAL_MAX_SESSION_DURATION_SEC}"
        )
    return None


def configured_stage3_servo_yaml():
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_yaml["joint_topic"] = "/task7e/joint_states_fresh"
    servo_yaml["scale"]["rotational"] = STAGE3_SERVO_ROTATIONAL_SCALE_RADPS
    return servo_yaml


def pen_real_air_node_parameters(max_session_duration_sec: float):
    parameters = {
        "base_frame": "base_link",
        "paper_frame": "paper_frame",
        "tool_frame": "tool0",
        "start_from_current_tool0": True,
        "require_motion_before_pose_command": True,
        "paper_origin_xyz": STAGE3_REAL_PAPER_ORIGIN_XYZ,
        "tool0_to_pen_tip_xyz": STAGE3_REAL_TOOL0_TO_PEN_TIP_XYZ,
        "servo_status_topic": "/servo_node/status",
        "servo_status_timeout_sec": 1.0,
        "max_session_duration_sec": max_session_duration_sec,
    }
    parameters.update(STAGE3_REAL_FAKEHARDWARE_MATCHED_PEN_PARAMS)
    return parameters


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                description="Real UR robot type.",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                description="Real robot IP address confirmed by preflight checks.",
            ),
            DeclareLaunchArgument(
                "reverse_ip",
                default_value="192.168.56.2",
                description="ROS PC IP on the robot network.",
            ),
            DeclareLaunchArgument(
                "human_confirmation",
                default_value="",
                description=(
                    "Must be I_CONFIRM_REAL_PEN_AIR_MOTION before real pen Servo starts."
                ),
            ),
            DeclareLaunchArgument(
                "external_control_program",
                default_value="/programs/external_control.urp",
                description="Program path passed to dashboard load_program.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Optionally launch RViz after real driver startup.",
            ),
            DeclareLaunchArgument(
                "servo_log_level",
                default_value="warn",
                description="Log level for moveit_servo/servo_node.",
            ),
            DeclareLaunchArgument(
                "joint_states_wait_timeout_sec",
                default_value="30.0",
                description="Maximum time to wait for /joint_states and active controllers.",
            ),
            DeclareLaunchArgument(
                "servo_startup_settle_sec",
                default_value="5.0",
                description="Settle time after joint state gate passes before launching Servo.",
            ),
            DeclareLaunchArgument(
                "servo_status_wait_timeout_sec",
                default_value="30.0",
                description="Maximum time to wait for /servo_node/status before pen input.",
            ),
            DeclareLaunchArgument(
                "dashboard_receive_timeout",
                default_value="20.0",
                description="Timeout passed to dashboard / External Control manager.",
            ),
            DeclareLaunchArgument(
                "hardware_ready_timeout_sec",
                default_value="30.0",
                description="Timeout for waiting controller manager and /joint_states.",
            ),
            DeclareLaunchArgument(
                "stop_external_control_on_shutdown",
                default_value="true",
                description="Stop External Control on shutdown when this launch started it.",
            ),
            DeclareLaunchArgument(
                "max_session_duration_sec",
                default_value=f"{STAGE3_REAL_DEFAULT_MAX_SESSION_DURATION_SEC:.1f}",
                description="Maximum real pen Servo session duration. Hard limit: 60.0s.",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="/joy",
                description="sensor_msgs/Joy topic used by the real pen Servo node.",
            ),
            DeclareLaunchArgument(
                "launch_joy_node",
                default_value="true",
                description="Launch the physical joy_node when true.",
            ),
            DeclareLaunchArgument(
                "joy_device_id",
                default_value="0",
                description="Joystick device id passed to joy_node.",
            ),
            DeclareLaunchArgument(
                "joy_device_name",
                default_value="",
                description="Optional joystick device name passed to joy_node.",
            ),
            DeclareLaunchArgument(
                "joy_deadzone",
                default_value="0.08",
                description="Joystick deadzone used by joy_node and pen command mapping.",
            ),
            DeclareLaunchArgument(
                "joy_autorepeat_rate",
                default_value="100.0",
                description="Joystick autorepeat rate passed to joy_node.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )


def launch_setup(context: LaunchContext, *_args, **_kwargs):
    max_session_duration_sec, error = parse_positive_bounded_float(
        context=context,
        argument_name="max_session_duration_sec",
        max_value=STAGE3_REAL_MAX_SESSION_DURATION_SEC,
    )
    if error is None:
        error = validate_real_air_configuration(
            human_confirmation=context.perform_substitution(
                LaunchConfiguration("human_confirmation")
            ),
            max_session_duration_sec=max_session_duration_sec,
        )
    if error is not None:
        return [
            LogInfo(msg=f"Refusing real robot pen Servo launch: {error}"),
            EmitEvent(event=Shutdown(reason="Invalid real pen Servo safety argument")),
        ]

    log_root_dir = Path.cwd() / "logs" / "stage3_real_air_pen_servo"
    run_log_dir = log_root_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    run_log_dir.mkdir(parents=True, exist_ok=True)

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro",
            {"name": LaunchConfiguration("ur_type")},
        )
        .to_moveit_configs()
    )

    servo_yaml = configured_stage3_servo_yaml()
    servo_params = {"moveit_servo": servo_yaml}

    description_launchfile = PathJoinSubstitution(
        [
            FindPackageShare("ur3_real_bringup_lab"),
            "launch",
            STAGE3_REAL_DESCRIPTION_LAUNCHFILE_NAME,
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
            "use_mock_hardware": STAGE3_REAL_DEFAULT_USE_MOCK_HARDWARE,
            "initial_joint_controller": STAGE3_REAL_INITIAL_JOINT_CONTROLLER,
            "activate_joint_controller": "true",
            "launch_rviz": "false",
            "launch_dashboard_client": "false",
            "description_launchfile": description_launchfile,
        }.items(),
    )

    hardware_ready_gate = Node(
        package="ur3_real_bringup_lab",
        executable="wait_for_hardware_ready.py",
        name="real_pen_air_hardware_ready_gate",
        output="both",
        parameters=[
            {
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("hardware_ready_timeout_sec"),
                    value_type=float,
                ),
                "expected_joint_names": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            }
        ],
    )

    dashboard_client_launch = IncludeLaunchDescription(
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
            "dashboard_receive_timeout": LaunchConfiguration("dashboard_receive_timeout"),
        }.items(),
    )

    external_control_manager = Node(
        package="ur3_real_bringup_lab",
        executable="manage_external_control.py",
        name="real_pen_air_external_control_manager",
        output="both",
        parameters=[
            {
                "program_path": LaunchConfiguration("external_control_program"),
                "require_remote_control": True,
                "startup_timeout_sec": ParameterValue(
                    LaunchConfiguration("dashboard_receive_timeout"),
                    value_type=float,
                ),
                "stop_on_shutdown": LaunchConfiguration("stop_external_control_on_shutdown"),
            }
        ],
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_pen_real_air",
        condition=IfCondition(LaunchConfiguration("real_pen_air_launch_rviz")),
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
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    joint_state_relay = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="joint_state_stamp_relay_node",
        name="real_pen_air_joint_state_stamp_relay",
        output="both",
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            {
                "source_topic": "/joint_states",
                "target_topic": "/task7e/joint_states_fresh",
                "publish_period_sec": STAGE3_JOINT_STATE_RELAY_PERIOD_SEC,
            }
        ],
    )

    joint_states_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_joint_states.py",
        name="real_pen_air_joint_states_gate",
        output="both",
        parameters=[
            {
                "topic": "/joint_states",
                "timeout_sec": LaunchConfiguration("joint_states_wait_timeout_sec"),
                "required_active_controllers": [
                    "joint_state_broadcaster",
                    "forward_position_controller",
                ],
            }
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="both",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("servo_log_level")],
    )

    servo_status_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_servo_status.py",
        name="real_pen_air_servo_status_gate",
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
        name="real_pen_air_joy_node",
        condition=IfCondition(LaunchConfiguration("launch_joy_node")),
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
        remappings=[("/joy", LaunchConfiguration("joy_topic"))],
    )

    pen_node_parameters = pen_real_air_node_parameters(max_session_duration_sec)
    pen_node_parameters.update(
        {
            "joy_topic": LaunchConfiguration("joy_topic"),
            "joy_deadzone": ParameterValue(
                LaunchConfiguration("joy_deadzone"),
                value_type=float,
            ),
            "alignment_error_log_path": str(run_log_dir / "tool_alignment_error.csv"),
        }
    )
    pen_servo_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="pen_fakehardware_servo_node",
        name="pen_real_air_servo",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_pen_writing_control_py"),
                    "config",
                    "pen_tool_model.yaml",
                ]
            ),
            pen_node_parameters,
        ],
    )

    def on_hardware_ready_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Real robot hardware ready gate passed; launching dashboard "
                        "client and External Control manager."
                    )
                ),
                dashboard_client_launch,
                external_control_manager,
                moveit_launch,
                rviz_node,
                joint_state_relay,
                TimerAction(
                    period=2.0,
                    actions=[
                        LogInfo(
                            msg=(
                                "Waiting for /joint_states and forward_position_controller "
                                "before starting MoveIt Servo..."
                            )
                        ),
                        joint_states_gate,
                    ],
                ),
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Real pen hardware ready gate failed before dashboard startup."
                )
            )
        ]

    def on_external_control_manager_exit(event, _context):
        if event.returncode == 0:
            return []
        return [
            EmitEvent(
                event=Shutdown(reason="Real pen External Control manager failed.")
            )
        ]

    def on_joint_states_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg="Real robot joint states and forward_position_controller are ready."
                ),
                TimerAction(
                    period=LaunchConfiguration("servo_startup_settle_sec"),
                    actions=[
                        LogInfo(msg="Starting MoveIt Servo node for real pen control."),
                        servo_node,
                        LogInfo(msg="Waiting for /servo_node/status before pen input."),
                        servo_status_gate,
                    ],
                ),
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Real pen Servo launch timed out before Servo startup."
                )
            )
        ]

    def on_servo_status_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Detected Servo status traffic. Starting real pen Servo node. "
                        f"session={max_session_duration_sec:.1f}s."
                    )
                ),
                joy_node,
                pen_servo_node,
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Real pen Servo launch timed out before pen startup."
                )
            )
        ]

    def on_servo_node_exit(event, _context):
        return [
            EmitEvent(
                event=Shutdown(
                    reason=f"Real pen Servo node exited with return code {event.returncode}."
                )
            )
        ]

    def on_pen_servo_node_exit(event, _context):
        return [
            EmitEvent(
                event=Shutdown(
                    reason=f"Real pen node exited with return code {event.returncode}."
                )
            )
        ]

    return [
        SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(run_log_dir)),
        SetEnvironmentVariable(name="RCUTILS_LOGGING_BUFFERED_STREAM", value="1"),
        SetEnvironmentVariable(name="RCUTILS_LOGGING_USE_STDOUT", value="1"),
        SetLaunchConfiguration(
            name="real_pen_air_launch_rviz",
            value=LaunchConfiguration("launch_rviz"),
        ),
        LogInfo(msg=f"Real pen air Servo logs will be written to: {run_log_dir}"),
        LogInfo(
            msg=(
                "Real pen air Servo confirmation accepted. "
                "Keep the emergency stop reachable, keep the pen clear of the table, "
                "press A to freeze, and press B to exit."
            )
        ),
        driver_launch,
        LogInfo(
            msg=(
                "Waiting for hardware ready gate before launching dashboard and "
                "External Control manager..."
            )
        ),
        hardware_ready_gate,
        RegisterEventHandler(
            OnProcessExit(
                target_action=hardware_ready_gate,
                on_exit=on_hardware_ready_gate_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=external_control_manager,
                on_exit=on_external_control_manager_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_states_gate,
                on_exit=on_joint_states_gate_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=servo_status_gate,
                on_exit=on_servo_status_gate_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=servo_node,
                on_exit=on_servo_node_exit,
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=pen_servo_node,
                on_exit=on_pen_servo_node_exit,
            )
        ),
    ]
