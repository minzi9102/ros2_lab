import math
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
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
import yaml


JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _float_list(context: LaunchContext, name: str, size: int) -> list[float]:
    raw = context.perform_substitution(LaunchConfiguration(name))
    values = [float(value) for value in yaml.safe_load(raw)]
    if len(values) != size:
        raise ValueError(f"{name} must contain {size} values")
    return values


def _load_servo_yaml() -> dict:
    from ament_index_python.packages import get_package_share_directory

    path = Path(get_package_share_directory("ur_moveit_config")) / "config/ur_servo.yaml"
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    config["joint_topic"] = "/force_pen_writing/joint_states_fresh"
    config["command_out_type"] = "trajectory_msgs/JointTrajectory"
    config["command_out_topic"] = "/joint_trajectory_controller/joint_trajectory"
    config["scale"]["rotational"] = math.tau
    config["use_smoothing"] = True
    return config


def launch_setup(context: LaunchContext, *_args, **_kwargs):
    payload_cog = _float_list(context, "payload_cog_xyz", 3)
    pen_tip = _float_list(context, "tool0_to_pen_tip_xyz", 3)
    log_directory = context.perform_substitution(LaunchConfiguration("log_directory"))
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(
            Path("srdf") / "ur.srdf.xacro",
            {"name": "ur3e"},
        )
        .to_moveit_configs()
    )
    calibrated_description = PathJoinSubstitution(
        [
            FindPackageShare("ur3_real_bringup_lab"),
            "launch",
            "task8B_real_calibrated_rsp.launch.py",
        ]
    )
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "use_mock_hardware": "false",
            "initial_joint_controller": "joint_trajectory_controller",
            "activate_joint_controller": "true",
            "launch_rviz": "false",
            "launch_dashboard_client": "false",
            "description_launchfile": calibrated_description,
        }.items(),
    )
    hardware_gate = Node(
        package="ur3_real_bringup_lab",
        executable="wait_for_hardware_ready.py",
        name="force_writing_hardware_ready_gate",
        output="both",
        parameters=[
            {
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("hardware_ready_timeout_sec"), value_type=float
                ),
                "expected_joint_names": JOINTS,
            }
        ],
    )
    dashboard = IncludeLaunchDescription(
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
    external_control = Node(
        package="ur3_real_bringup_lab",
        executable="manage_external_control.py",
        name="force_writing_external_control_manager",
        output="both",
        parameters=[
            {
                "program_path": LaunchConfiguration("external_control_program"),
                "require_remote_control": True,
                "startup_timeout_sec": ParameterValue(
                    LaunchConfiguration("dashboard_receive_timeout"), value_type=float
                ),
                "stop_on_shutdown": True,
            }
        ],
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
        }.items(),
    )
    relay = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="joint_state_stamp_relay_node",
        name="force_writing_joint_state_stamp_relay",
        output="both",
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            {
                "source_topic": "/joint_states",
                "target_topic": "/force_pen_writing/joint_states_fresh",
                "publish_period_sec": 0.004,
            }
        ],
    )
    joint_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_joint_states.py",
        name="force_writing_joint_states_gate",
        output="both",
        parameters=[
            {
                "topic": "/joint_states",
                "timeout_sec": LaunchConfiguration("joint_states_wait_timeout_sec"),
                "required_active_controllers": [
                    "joint_state_broadcaster",
                    "joint_trajectory_controller",
                ],
            }
        ],
    )
    servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="both",
        parameters=[moveit_config.to_dict(), {"moveit_servo": _load_servo_yaml()}],
        arguments=["--ros-args", "--log-level", "warn"],
    )
    servo_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_servo_status.py",
        name="force_writing_servo_status_gate",
        output="both",
        parameters=[
            {
                "topic": "/servo_node/status",
                "timeout_sec": LaunchConfiguration("servo_status_wait_timeout_sec"),
            }
        ],
    )
    paper_seek = Node(
        package="ur3e_force_pen_writing_py",
        executable="paper_seek_servo_node",
        name="paper_seek_servo",
        condition=IfCondition(LaunchConfiguration("launch_paper_seek")),
        output="screen",
        parameters=[
            {
                "payload_mass_kg": ParameterValue(
                    LaunchConfiguration("payload_mass_kg"), value_type=float
                ),
                "payload_cog_xyz": payload_cog,
                "tool0_to_pen_tip_xyz": pen_tip,
                "wrench_topic": LaunchConfiguration("wrench_topic"),
            }
        ],
    )

    def after_hardware(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Hardware ready gate failed"))]
        return [dashboard, external_control, moveit, relay, joint_gate]

    def after_joints(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Joint state gate failed"))]
        return [TimerAction(period=5.0, actions=[servo, servo_gate])]

    def after_servo(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Servo status gate failed"))]
        return [
            LogInfo(msg="Force-writing bringup ready; no motion has been commanded."),
            paper_seek,
        ]

    def external_failed(event, _context):
        if event.returncode == 0:
            return []
        return [EmitEvent(event=Shutdown(reason="External Control manager failed"))]

    return [
        SetEnvironmentVariable(name="ROS_LOG_DIR", value=log_directory),
        SetEnvironmentVariable(name="RCUTILS_LOGGING_BUFFERED_STREAM", value="1"),
        driver,
        hardware_gate,
        RegisterEventHandler(
            OnProcessExit(target_action=hardware_gate, on_exit=after_hardware)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=joint_gate, on_exit=after_joints)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=servo_gate, on_exit=after_servo)
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=external_control, on_exit=external_failed)
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("reverse_ip", default_value="192.168.56.2"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("launch_paper_seek", default_value="false"),
            DeclareLaunchArgument("payload_mass_kg", default_value="0.085"),
            DeclareLaunchArgument("payload_cog_xyz", default_value="[0, 0, 0]"),
            DeclareLaunchArgument(
                "tool0_to_pen_tip_xyz", default_value="[0.00079, -0.00076, 0.15172]"
            ),
            DeclareLaunchArgument(
                "wrench_topic",
                default_value="/force_torque_sensor_broadcaster/wrench",
            ),
            DeclareLaunchArgument("log_directory"),
            DeclareLaunchArgument("hardware_ready_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("joint_states_wait_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("servo_status_wait_timeout_sec", default_value="30.0"),
            DeclareLaunchArgument("dashboard_receive_timeout", default_value="20.0"),
            DeclareLaunchArgument(
                "external_control_program",
                default_value="/programs/external_control.urp",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
