import os
import yaml

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
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


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def validate_fakehardware_arguments(context: LaunchContext, *_args, **_kwargs):
    use_mock_hardware = context.perform_substitution(
        LaunchConfiguration("use_mock_hardware")
    )
    if use_mock_hardware.lower() not in ("1", "true", "yes", "on"):
        return _refuse_launch(
            "stage2_fakehardware_pen_servo only supports use_mock_hardware:=true"
        )

    for argument_name in (
        "joy_deadzone",
        "joy_autorepeat_rate",
        "joint_states_wait_timeout_sec",
        "servo_startup_settle_sec",
        "servo_status_wait_timeout_sec",
    ):
        raw_value = context.perform_substitution(LaunchConfiguration(argument_name))
        try:
            value = float(raw_value)
        except ValueError:
            return _refuse_launch(f"{argument_name} must be a number, got {raw_value!r}")
        if argument_name == "joy_deadzone":
            if value < 0.0 or value >= 1.0:
                return _refuse_launch(f"{argument_name} must be in [0.0, 1.0)")
            continue
        if value <= 0.0:
            return _refuse_launch(f"{argument_name} must be greater than 0.0")

    return [SetLaunchConfiguration(name="pen_fakehardware_args_valid", value="true")]


def set_runtime_log_output(context: LaunchContext, *_args, **_kwargs):
    verbose_runtime_logs = _as_bool(
        context.perform_substitution(LaunchConfiguration("verbose_runtime_logs"))
    )
    return [
        SetLaunchConfiguration(
            name="pen_runtime_output",
            value="screen" if verbose_runtime_logs else "log",
        ),
        SetLaunchConfiguration(
            name="pen_runtime_output_both",
            value="both" if verbose_runtime_logs else "log",
        ),
    ]


def _refuse_launch(reason: str):
    return [
        SetLaunchConfiguration(name="pen_fakehardware_args_valid", value="false"),
        LogInfo(msg=f"Refusing pen fake-hardware Servo launch: {reason}"),
        EmitEvent(event=Shutdown(reason="Invalid pen fake-hardware Servo argument")),
    ]


def _as_bool(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def generate_launch_description() -> LaunchDescription:
    log_root_dir = Path.cwd() / "logs" / "stage2_fakehardware_pen_servo"
    default_run_log_dir = log_root_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    default_run_log_dir.mkdir(parents=True, exist_ok=True)

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3",
        description="UR robot type used by the pen fake-hardware Servo scene.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.56.101",
        description="Kept for UR driver launch compatibility; fake hardware does not connect.",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Must remain true for this virtual/fake-hardware stage.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz with the robot, paper, pen, TF, and marker displays.",
    )
    servo_log_level_arg = DeclareLaunchArgument(
        "servo_log_level",
        default_value="warn",
        description="Log level for moveit_servo/servo_node.",
    )
    verbose_runtime_logs_arg = DeclareLaunchArgument(
        "verbose_runtime_logs",
        default_value="false",
        description="Print verbose runtime node logs to terminal when true.",
    )
    run_log_dir_arg = DeclareLaunchArgument(
        "run_log_dir",
        default_value=str(default_run_log_dir),
        description="Directory used for Stage2 runtime logs and diagnostic CSV files.",
    )
    joint_states_wait_timeout_arg = DeclareLaunchArgument(
        "joint_states_wait_timeout_sec",
        default_value="15.0",
        description="Maximum time to wait for /joint_states and active controllers.",
    )
    servo_startup_settle_arg = DeclareLaunchArgument(
        "servo_startup_settle_sec",
        default_value="5.0",
        description="Settle time after joint state gate passes before launching Servo.",
    )
    servo_status_wait_timeout_arg = DeclareLaunchArgument(
        "servo_status_wait_timeout_sec",
        default_value="15.0",
        description="Maximum time to wait for /servo_node/status before starting pen input.",
    )
    joy_topic_arg = DeclareLaunchArgument(
        "joy_topic",
        default_value="/joy",
        description="sensor_msgs/Joy topic used by the pen fake-hardware Servo node.",
    )
    launch_joy_node_arg = DeclareLaunchArgument(
        "launch_joy_node",
        default_value="true",
        description="Launch the physical joy_node when true.",
    )
    joy_device_id_arg = DeclareLaunchArgument(
        "joy_device_id",
        default_value="0",
        description="Joystick device id passed to joy_node.",
    )
    joy_device_name_arg = DeclareLaunchArgument(
        "joy_device_name",
        default_value="",
        description="Optional joystick device name passed to joy_node.",
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        "joy_deadzone",
        default_value="0.08",
        description="Joystick deadzone used by joy_node and pen command mapping.",
    )
    joy_autorepeat_rate_arg = DeclareLaunchArgument(
        "joy_autorepeat_rate",
        default_value="100.0",
        description="Joystick autorepeat rate passed to joy_node.",
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro",
            {"name": LaunchConfiguration("ur_type")},
        )
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_yaml["joint_topic"] = "/task7e/joint_states_fresh"
    servo_yaml["scale"]["rotational"] = 0.5
    servo_params = {"moveit_servo": servo_yaml}

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": "true",
            "initial_joint_controller": "forward_position_controller",
            "launch_rviz": "false",
            "description_launchfile": PathJoinSubstitution(
                [
                    FindPackageShare("ur3_moveit_servo_lab_cpp"),
                    "launch",
                    "task7E_ur_rsp.launch.py",
                ]
            ),
        }.items(),
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
        name="rviz2_pen_fakehardware",
        condition=IfCondition(LaunchConfiguration("pen_fakehardware_launch_rviz")),
        output=LaunchConfiguration("pen_runtime_output"),
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
        name="pen_joint_state_stamp_relay",
        output=LaunchConfiguration("pen_runtime_output"),
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            {
                "source_topic": "/joint_states",
                "target_topic": "/task7e/joint_states_fresh",
                "publish_period_sec": 0.02,
            }
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output=LaunchConfiguration("pen_runtime_output"),
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("servo_log_level")],
    )

    joint_states_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_joint_states.py",
        name="pen_joint_states_gate",
        output="screen",
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

    servo_status_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_servo_status.py",
        name="pen_servo_status_gate",
        output="screen",
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
        name="pen_joy_node",
        condition=IfCondition(LaunchConfiguration("launch_joy_node")),
        output=LaunchConfiguration("pen_runtime_output_both"),
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

    pen_servo_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="pen_fakehardware_servo_node",
        name="pen_fakehardware_servo",
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
                "joy_topic": LaunchConfiguration("joy_topic"),
                "joy_deadzone": ParameterValue(
                    LaunchConfiguration("joy_deadzone"),
                    value_type=float,
                ),
                "start_from_current_tool0": True,
                "require_motion_before_pose_command": True,
                "paper_origin_xyz": [0.45, 0.0, 0.12],
                "tool0_to_pen_tip_xyz": [0.0, 0.0, 0.14],
                "servo_status_topic": "/servo_node/status",
                "servo_status_timeout_sec": 1.0,
                "alignment_error_log_path": PathJoinSubstitution(
                    [
                        LaunchConfiguration("run_log_dir"),
                        "tool_alignment_error.csv",
                    ]
                ),
            },
        ],
    )

    def on_joint_states_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Detected /joint_states and active controllers. "
                        "Starting MoveIt Servo soon."
                    )
                ),
                TimerAction(
                    period=LaunchConfiguration("servo_startup_settle_sec"),
                    actions=[
                        LogInfo(msg="Starting MoveIt Servo node."),
                        servo_node,
                        LogInfo(
                            msg=(
                                "Waiting for /servo_node/status before starting "
                                "pen fake-hardware input."
                            )
                        ),
                        servo_status_gate,
                    ],
                ),
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Pen fake-hardware launch timed out before Servo startup."
                )
            )
        ]

    def on_servo_status_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg="Detected Servo status traffic. Starting joy and pen POSE node."
                ),
                joy_node,
                pen_servo_node,
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Pen fake-hardware launch timed out before pen node startup."
                )
            )
        ]

    def on_servo_node_exit(event, _context):
        return [
            LogInfo(
                msg=(
                    "MoveIt Servo node exited with return code "
                    f"{event.returncode}. Shutting down pen fake-hardware launch."
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=f"MoveIt Servo exited with return code {event.returncode}."
                )
            ),
        ]

    def on_pen_servo_node_exit(event, _context):
        return [
            EmitEvent(
                event=Shutdown(
                    reason=f"Pen fake-hardware node exited with return code {event.returncode}."
                )
            )
        ]

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_mock_hardware_arg,
            launch_rviz_arg,
            servo_log_level_arg,
            verbose_runtime_logs_arg,
            run_log_dir_arg,
            joint_states_wait_timeout_arg,
            servo_startup_settle_arg,
            servo_status_wait_timeout_arg,
            joy_topic_arg,
            launch_joy_node_arg,
            joy_device_id_arg,
            joy_device_name_arg,
            joy_deadzone_arg,
            joy_autorepeat_rate_arg,
            SetLaunchConfiguration(name="pen_fakehardware_args_valid", value="false"),
            OpaqueFunction(function=validate_fakehardware_arguments),
            GroupAction(
                scoped=False,
                condition=IfCondition(LaunchConfiguration("pen_fakehardware_args_valid")),
                actions=[
                    SetEnvironmentVariable(
                        name="ROS_LOG_DIR",
                        value=LaunchConfiguration("run_log_dir"),
                    ),
                    SetEnvironmentVariable(
                        name="RCUTILS_LOGGING_BUFFERED_STREAM",
                        value="1",
                    ),
                    SetEnvironmentVariable(
                        name="RCUTILS_LOGGING_USE_STDOUT",
                        value="1",
                    ),
                    SetLaunchConfiguration(
                        name="pen_fakehardware_launch_rviz",
                        value=LaunchConfiguration("launch_rviz"),
                    ),
                    SetLaunchConfiguration(name="pen_runtime_output", value="log"),
                    SetLaunchConfiguration(name="pen_runtime_output_both", value="log"),
                    OpaqueFunction(function=set_runtime_log_output),
                    LogInfo(
                        msg=[
                            "Pen fake-hardware Servo logs will be written to: ",
                            LaunchConfiguration("run_log_dir"),
                        ]
                    ),
                    driver_launch,
                    moveit_launch,
                    rviz_node,
                    joint_state_relay,
                    LogInfo(
                        msg=(
                            "Waiting for /joint_states and active controllers before "
                            "starting MoveIt Servo..."
                        )
                    ),
                    joint_states_gate,
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
                ],
            ),
        ]
    )
