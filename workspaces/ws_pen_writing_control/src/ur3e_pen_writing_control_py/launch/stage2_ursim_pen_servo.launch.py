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
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


STAGE2_SERVO_ROTATIONAL_SCALE_RADPS = math.tau
STAGE2_URSIM_DEFAULT_ROBOT_IP = "172.17.0.2"
STAGE2_URSIM_DEFAULT_USE_MOCK_HARDWARE = "false"
STAGE2_URSIM_DEFAULT_EXTERNAL_CONTROL_PROGRAM = "/ursim/programs/123.urp"
STAGE2_URSIM_DEFAULT_PAPER_ORIGIN_XYZ = [0.45, 0.0, 0.12]
STAGE2_URSIM_DEFAULT_TOOL0_TO_PEN_TIP_XYZ = [0.0, 0.0, 0.14]


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def validate_ursim_arguments(context: LaunchContext, *_args, **_kwargs):
    use_mock_hardware = context.perform_substitution(
        LaunchConfiguration("use_mock_hardware")
    )
    if use_mock_hardware.lower() in ("1", "true", "yes", "on"):
        return _refuse_launch(
            "stage2_ursim_pen_servo requires use_mock_hardware:=false"
        )

    robot_ip = context.perform_substitution(LaunchConfiguration("robot_ip")).strip()
    if not robot_ip:
        return _refuse_launch("robot_ip must not be empty for URSim")

    for argument_name in (
        "joy_deadzone",
        "joy_autorepeat_rate",
        "joint_states_wait_timeout_sec",
        "servo_startup_settle_sec",
        "servo_status_wait_timeout_sec",
        "servo_linear_scale",
        "servo_low_pass_filter_coeff",
        "dashboard_receive_timeout_sec",
        "script_sender_port",
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

    return [SetLaunchConfiguration(name="pen_ursim_args_valid", value="true")]


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
        SetLaunchConfiguration(name="pen_ursim_args_valid", value="false"),
        LogInfo(msg=f"Refusing pen URSim Servo launch: {reason}"),
        EmitEvent(event=Shutdown(reason="Invalid pen URSim Servo argument")),
    ]


def _as_bool(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def configured_stage2_servo_yaml(*, linear_scale=0.6, low_pass_filter_coeff=10.0):
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_yaml["joint_topic"] = "/task7e/joint_states_fresh"
    servo_yaml["scale"]["linear"] = linear_scale
    servo_yaml["scale"]["rotational"] = STAGE2_SERVO_ROTATIONAL_SCALE_RADPS
    servo_yaml["low_pass_filter_coeff"] = low_pass_filter_coeff
    return servo_yaml


def pen_ursim_node_parameters():
    return {
        "base_frame": "base_link",
        "paper_frame": "paper_frame",
        "tool_frame": "tool0",
        "start_from_current_tool0": True,
        "require_motion_before_pose_command": True,
        "paper_origin_xyz": STAGE2_URSIM_DEFAULT_PAPER_ORIGIN_XYZ,
        "tool0_to_pen_tip_xyz": STAGE2_URSIM_DEFAULT_TOOL0_TO_PEN_TIP_XYZ,
        "servo_status_topic": "/servo_node/status",
        "servo_status_timeout_sec": 1.0,
    }


def generate_launch_description() -> LaunchDescription:
    log_root_dir = Path.cwd() / "logs" / "stage2_ursim_pen_servo"
    default_run_log_dir = log_root_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    default_run_log_dir.mkdir(parents=True, exist_ok=True)

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3",
        description="UR robot type used by the pen URSim Servo scene.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value=STAGE2_URSIM_DEFAULT_ROBOT_IP,
        description="URSim IP address.",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value=STAGE2_URSIM_DEFAULT_USE_MOCK_HARDWARE,
        description="Must remain false for this URSim stage.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
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
        description="Directory used for Stage2 URSim runtime logs and diagnostic CSV files.",
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
    servo_linear_scale_arg = DeclareLaunchArgument(
        "servo_linear_scale",
        default_value="0.6",
        description="MoveIt Servo linear scale for Cartesian and pose tracking commands.",
    )
    servo_low_pass_filter_coeff_arg = DeclareLaunchArgument(
        "servo_low_pass_filter_coeff",
        default_value="10.0",
        description="MoveIt Servo joint-state low-pass filter coefficient.",
    )
    auto_start_external_control_arg = DeclareLaunchArgument(
        "auto_start_external_control",
        default_value="true",
        description="Automatically load and play the URSim External Control program.",
    )
    external_control_program_arg = DeclareLaunchArgument(
        "external_control_program",
        default_value=STAGE2_URSIM_DEFAULT_EXTERNAL_CONTROL_PROGRAM,
        description="Dashboard path of the URSim External Control program to load.",
    )
    dashboard_receive_timeout_arg = DeclareLaunchArgument(
        "dashboard_receive_timeout_sec",
        default_value="20.0",
        description="Timeout used while auto-starting the URSim External Control program.",
    )
    script_sender_port_arg = DeclareLaunchArgument(
        "script_sender_port",
        default_value="50002",
        description="Driver port that must be listening before URSim External Control starts.",
    )
    joy_topic_arg = DeclareLaunchArgument(
        "joy_topic",
        default_value="/joy",
        description="sensor_msgs/Joy topic used by the pen URSim Servo node.",
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

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "initial_joint_controller": "forward_position_controller",
            "script_sender_port": LaunchConfiguration("script_sender_port"),
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_pen_ursim",
        condition=IfCondition(LaunchConfiguration("pen_ursim_launch_rviz")),
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
        name="pen_ursim_joint_state_stamp_relay",
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

    external_control_autostart = ExecuteProcess(
        cmd=[
            "python3",
            "-c",
            (
                "import socket, sys, time\n"
                "host = sys.argv[1]\n"
                "program = sys.argv[2]\n"
                "timeout = float(sys.argv[3])\n"
                "script_sender_port = int(sys.argv[4])\n"
                "deadline = time.monotonic() + timeout\n"
                "def issue(command):\n"
                "    sock = socket.create_connection((host, 29999), timeout=3.0)\n"
                "    sock.settimeout(3.0)\n"
                "    greeting = sock.recv(4096).decode('utf-8', errors='replace').strip()\n"
                "    print(greeting, flush=True)\n"
                "    sock.sendall((command + '\\n').encode('utf-8'))\n"
                "    response = sock.recv(4096).decode('utf-8', errors='replace').strip()\n"
                "    sock.close()\n"
                "    print(response, flush=True)\n"
                "    return response\n"
                "def wait_for_script_sender():\n"
                "    while time.monotonic() < deadline:\n"
                "        try:\n"
                "            sock = socket.create_connection(('127.0.0.1', script_sender_port), timeout=1.0)\n"
                "            sock.close()\n"
                "            print(f'script_sender_port_ready={script_sender_port}', flush=True)\n"
                "            return\n"
                "        except OSError:\n"
                "            time.sleep(0.2)\n"
                "    raise SystemExit(f'script sender port {script_sender_port} did not open before timeout')\n"
                "wait_for_script_sender()\n"
                "state = issue('programState')\n"
                "if state.startswith('PLAYING'):\n"
                "    raise SystemExit(0)\n"
                "loaded = issue('get loaded program')\n"
                "if program not in loaded:\n"
                "    reply = issue(f'load {program}')\n"
                "    if 'Loading program:' not in reply and 'File loaded' not in reply:\n"
                "        raise SystemExit(f'Failed to load program: {reply}')\n"
                "reply = issue('play')\n"
                "if 'Starting program' not in reply and 'Failed to execute: play' not in reply:\n"
                "    raise SystemExit(f'Failed to start program: {reply}')\n"
                "while time.monotonic() < deadline:\n"
                "    state = issue('programState')\n"
                "    if state.startswith('PLAYING'):\n"
                "        raise SystemExit(0)\n"
                "    time.sleep(0.5)\n"
                "raise SystemExit('External Control did not reach PLAYING before timeout')\n"
            ),
            LaunchConfiguration("robot_ip"),
            LaunchConfiguration("external_control_program"),
            LaunchConfiguration("dashboard_receive_timeout_sec"),
            LaunchConfiguration("script_sender_port"),
        ],
        condition=IfCondition(LaunchConfiguration("auto_start_external_control")),
        output=LaunchConfiguration("pen_runtime_output"),
    )

    joint_states_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_joint_states.py",
        name="pen_ursim_joint_states_gate",
        output="screen",
        parameters=[
            {
                "topic": "/joint_states",
                "timeout_sec": LaunchConfiguration("joint_states_wait_timeout_sec"),
                "required_active_controllers": [
                    "joint_state_broadcaster",
                ],
            }
        ],
    )

    activate_forward_position_controller = Node(
        package="ur3e_pen_writing_control_py",
        executable="controller_switch_once_node",
        name="activate_ursim_forward_position_controller",
        output="screen",
        parameters=[
            {
                "activate_controllers": ["forward_position_controller"],
                "deactivate_controllers": [""],
                "timeout_sec": 20.0,
                "result_path": PathJoinSubstitution(
                    [
                        LaunchConfiguration("run_log_dir"),
                        "activate_forward_position_controller.json",
                    ]
                ),
            }
        ],
    )

    servo_status_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_servo_status.py",
        name="pen_ursim_servo_status_gate",
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
        name="pen_ursim_joy_node",
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

    pen_node_parameters = pen_ursim_node_parameters()
    pen_node_parameters.update(
        {
            "joy_topic": LaunchConfiguration("joy_topic"),
            "joy_deadzone": ParameterValue(
                LaunchConfiguration("joy_deadzone"),
                value_type=float,
            ),
            "alignment_error_log_path": PathJoinSubstitution(
                [
                    LaunchConfiguration("run_log_dir"),
                    "tool_alignment_error.csv",
                ]
            ),
        }
    )
    pen_servo_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="pen_fakehardware_servo_node",
        name="pen_ursim_servo",
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

    def on_joint_states_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Detected /joint_states and base controllers. "
                        "Re-activating forward_position_controller before Servo."
                    )
                ),
                TimerAction(
                    period=LaunchConfiguration("servo_startup_settle_sec"),
                    actions=[
                        activate_forward_position_controller,
                    ],
                ),
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Pen URSim launch timed out before Servo startup."
                )
            )
        ]

    servo_yaml = configured_stage2_servo_yaml()
    servo_yaml["scale"]["linear"] = ParameterValue(
        LaunchConfiguration("servo_linear_scale"),
        value_type=float,
    )
    servo_yaml["low_pass_filter_coeff"] = ParameterValue(
        LaunchConfiguration("servo_low_pass_filter_coeff"),
        value_type=float,
    )
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        prefix="prlimit --rtprio=0:0 --",
        output=LaunchConfiguration("pen_runtime_output"),
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("servo_log_level"),
        ],
    )

    def on_activate_forward_position_controller_exit(event, _context):
        if event.returncode == 0:
            return [
                TimerAction(
                    period=LaunchConfiguration("servo_startup_settle_sec"),
                    actions=[
                        LogInfo(
                            msg=(
                                "forward_position_controller is active. "
                                "Starting MoveIt Servo node."
                            )
                        ),
                        servo_node,
                        LogInfo(
                            msg=(
                                "Waiting for /servo_node/status before starting "
                                "pen URSim input."
                            )
                        ),
                        servo_status_gate,
                    ],
                ),
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Pen URSim launch timed out before Servo startup."
                )
            )
        ]

    def on_external_control_autostart_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "URSim External Control is playing. Waiting for "
                        "/joint_states and base controllers before Servo..."
                    )
                ),
                joint_states_gate,
            ]
        return [
            LogInfo(
                msg=(
                    "URSim External Control auto-start failed with return code "
                    f"{event.returncode}."
                )
            ),
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "URSim External Control auto-start failed before Servo startup."
                    )
                )
            ),
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
                    reason="Pen URSim launch timed out before pen node startup."
                )
            )
        ]

    def on_servo_node_exit(event, _context):
        return [
            LogInfo(
                msg=(
                    "MoveIt Servo node exited with return code "
                    f"{event.returncode}. Shutting down pen URSim launch."
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
                    reason=f"Pen URSim node exited with return code {event.returncode}."
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
            servo_linear_scale_arg,
            servo_low_pass_filter_coeff_arg,
            auto_start_external_control_arg,
            external_control_program_arg,
            dashboard_receive_timeout_arg,
            script_sender_port_arg,
            joy_topic_arg,
            launch_joy_node_arg,
            joy_device_id_arg,
            joy_device_name_arg,
            joy_deadzone_arg,
            joy_autorepeat_rate_arg,
            SetLaunchConfiguration(name="pen_ursim_args_valid", value="false"),
            OpaqueFunction(function=validate_ursim_arguments),
            GroupAction(
                scoped=False,
                condition=IfCondition(LaunchConfiguration("pen_ursim_args_valid")),
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
                        name="pen_ursim_launch_rviz",
                        value=LaunchConfiguration("launch_rviz"),
                    ),
                    SetLaunchConfiguration(name="pen_runtime_output", value="log"),
                    SetLaunchConfiguration(name="pen_runtime_output_both", value="log"),
                    OpaqueFunction(function=set_runtime_log_output),
                    LogInfo(
                        msg=[
                            "Pen URSim Servo logs will be written to: ",
                            LaunchConfiguration("run_log_dir"),
                        ]
                    ),
                    driver_launch,
                    external_control_autostart,
                    rviz_node,
                    joint_state_relay,
                    LogInfo(
                        msg=(
                            "Waiting for URSim External Control before starting "
                            "MoveIt Servo..."
                        )
                    ),
                    TimerAction(
                        period=0.0,
                        condition=UnlessCondition(
                            LaunchConfiguration("auto_start_external_control")
                        ),
                        actions=[joint_states_gate],
                    ),
                    RegisterEventHandler(
                        OnProcessExit(
                            target_action=external_control_autostart,
                            on_exit=on_external_control_autostart_exit,
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
                            target_action=activate_forward_position_controller,
                            on_exit=on_activate_forward_position_controller_exit,
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
