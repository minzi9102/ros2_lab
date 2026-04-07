import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("ur3_ros2_control_lab_py")
    xacro_path = os.path.join(package_share, "urdf", "ur3_simplified_ros2_control.urdf.xacro")
    controllers_yaml_path = os.path.join(package_share, "config", "ur3_controllers_minimal.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    link_2_length = LaunchConfiguration("link_2_length")
    tool0_offset_z = LaunchConfiguration("tool0_offset_z")
    upper_joint_limit_scale = LaunchConfiguration("upper_joint_limit_scale")
    lower_joint_limit_scale = LaunchConfiguration("lower_joint_limit_scale")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_path,
                " ",
                "link_2_length:=",
                link_2_length,
                " ",
                "tool0_offset_z:=",
                tool0_offset_z,
                " ",
                "upper_joint_limit_scale:=",
                upper_joint_limit_scale,
                " ",
                "lower_joint_limit_scale:=",
                lower_joint_limit_scale,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            },
            controllers_yaml_path,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("link_2_length", default_value="0.34"),
            DeclareLaunchArgument("tool0_offset_z", default_value="0.12"),
            DeclareLaunchArgument("upper_joint_limit_scale", default_value="1.0"),
            DeclareLaunchArgument("lower_joint_limit_scale", default_value="1.0"),
            robot_state_publisher_node,
            ros2_control_node,
            TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        ]
    )
