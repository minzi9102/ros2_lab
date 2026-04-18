import os
import yaml

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    servo_log_level_arg = DeclareLaunchArgument(
        "servo_log_level",
        default_value="debug",
        description="Log level for the standalone Servo node started by Task 7E.",
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
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "launch_servo": "false",
        }.items(),
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("servo_log_level")],
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
    )

    servo_commander = Node(
        package="ur3_moveit_servo_lab_cpp",
        executable="servo_twist_commander_node",
        name="ur3_servo_twist_commander",
        output="screen",
    )

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_mock_hardware_arg,
            launch_rviz_arg,
            servo_log_level_arg,
            driver_launch,
            moveit_launch,
            servo_node,
            servo_commander,
        ]
    )
