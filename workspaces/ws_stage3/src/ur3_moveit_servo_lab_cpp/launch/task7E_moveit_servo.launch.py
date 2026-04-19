import os
import yaml

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description() -> LaunchDescription:
    log_root_dir = Path.cwd() / "logs" / "task7E"
    run_log_dir = log_root_dir / datetime.now().strftime("%Y%m%d-%H%M%S")
    run_log_dir.mkdir(parents=True, exist_ok=True)

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3",
        description="UR robot type for Task 7E.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.56.101",
        description="Robot or URSim IP address. Fake hardware keeps this only for alignment.",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Use fake hardware for the Task 7E Servo baseline.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Optionally launch RViz while Servo is running.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="tool0",
        description="Reference frame used by the Task 7E twist commander.",
    )
    linear_x_arg = DeclareLaunchArgument(
        "linear_x",
        default_value="0.02",
        description="Linear X command sent by the Task 7E twist commander.",
    )
    linear_y_arg = DeclareLaunchArgument(
        "linear_y",
        default_value="0.0",
        description="Linear Y command sent by the Task 7E twist commander.",
    )
    linear_z_arg = DeclareLaunchArgument(
        "linear_z",
        default_value="0.0",
        description="Linear Z command sent by the Task 7E twist commander.",
    )
    servo_log_level_arg = DeclareLaunchArgument(
        "servo_log_level",
        default_value="info",
        description="Log level for the standalone Servo node started by Task 7E.",
    )
    joint_states_wait_timeout_arg = DeclareLaunchArgument(
        "joint_states_wait_timeout_sec",
        default_value="15.0",
        description="Maximum time to wait for /joint_states before starting Task 7E Servo nodes.",
    )
    servo_startup_settle_arg = DeclareLaunchArgument(
        "servo_startup_settle_sec",
        default_value="2.0",
        description="Extra settle time after MoveIt and controllers are ready before launching the Task 7E Servo container.",
    )
    servo_status_wait_timeout_arg = DeclareLaunchArgument(
        "servo_status_wait_timeout_sec",
        default_value="15.0",
        description="Maximum time to wait for /servo_node/status before starting the Task 7E commander.",
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": LaunchConfiguration("ur_type")})
        .to_moveit_configs()
    )
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
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
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "initial_joint_controller": "forward_position_controller",
            "launch_rviz": "false",
            "description_launchfile": PathJoinSubstitution(
                [FindPackageShare("ur3_moveit_servo_lab_cpp"), "launch", "task7E_ur_rsp.launch.py"]
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

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        condition=IfCondition(LaunchConfiguration("task7e_launch_rviz")),
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": False,
            },
        ],
    )

    servo_container = ComposableNodeContainer(
        name="task7e_servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    moveit_config.to_dict(),
                    servo_params,
                ],
                extra_arguments=[{"use_intra_process_comms": False}],
            )
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("servo_log_level")],
    )

    servo_commander = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="servo_twist_commander_node",
        name="ur3_servo_twist_commander",
        output="screen",
        parameters=[
            {
                "frame_id": LaunchConfiguration("frame_id"),
                "linear_x": LaunchConfiguration("linear_x"),
                "linear_y": LaunchConfiguration("linear_y"),
                "linear_z": LaunchConfiguration("linear_z"),
            }
        ],
    )
    joint_states_gate = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="wait_for_joint_states.py",
        name="task7e_joint_states_gate",
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
        name="task7e_servo_status_gate",
        output="screen",
        parameters=[
            {
                "topic": "/servo_node/status",
                "timeout_sec": LaunchConfiguration("servo_status_wait_timeout_sec"),
            }
        ],
    )

    def on_joint_states_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg="Detected /joint_states traffic. Waiting briefly for MoveIt startup to settle before launching Task 7E Servo."
                ),
                TimerAction(
                    period=LaunchConfiguration("servo_startup_settle_sec"),
                    actions=[
                        LogInfo(msg="Starting Task 7E Servo container."),
                        servo_container,
                        LogInfo(
                            msg="Waiting for /servo_node/status before starting the Task 7E commander."
                        ),
                        servo_status_gate,
                    ],
                ),
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Task 7E joint state gate timed out before Servo startup."
                )
            )
        ]

    def on_servo_status_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg="Detected Servo status traffic. Starting the Task 7E commander."),
                servo_commander,
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason="Task 7E Servo status gate timed out before commander startup."
                )
            )
        ]

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_mock_hardware_arg,
            launch_rviz_arg,
            frame_id_arg,
            linear_x_arg,
            linear_y_arg,
            linear_z_arg,
            servo_log_level_arg,
            joint_states_wait_timeout_arg,
            servo_startup_settle_arg,
            servo_status_wait_timeout_arg,
            SetEnvironmentVariable(name="ROS_LOG_DIR", value=str(run_log_dir)),
            SetLaunchConfiguration(
                name="task7e_launch_rviz",
                value=LaunchConfiguration("launch_rviz"),
            ),
            LogInfo(msg=f"Task 7E logs will be written to: {run_log_dir}"),
            driver_launch,
            moveit_launch,
            rviz_node,
            LogInfo(
                msg="Waiting for /joint_states and active controllers before starting Task 7E Servo nodes..."
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
        ]
    )
