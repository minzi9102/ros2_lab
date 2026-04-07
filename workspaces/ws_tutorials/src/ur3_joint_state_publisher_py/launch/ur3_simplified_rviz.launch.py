import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("ur3_joint_state_publisher_py")
    xacro_path = os.path.join(package_share, "urdf", "ur3_simplified.urdf.xacro")
    rviz_config_path = os.path.join(package_share, "rviz", "ur3_simplified.rviz")

    joint_state_topic = LaunchConfiguration("joint_state_topic")
    robot_description_topic = LaunchConfiguration("robot_description_topic")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    max_abs_position_rad = LaunchConfiguration("max_abs_position_rad")
    base_frequency_hz = LaunchConfiguration("base_frequency_hz")
    link_2_length = LaunchConfiguration("link_2_length")
    tool0_offset_z = LaunchConfiguration("tool0_offset_z")
    upper_joint_limit_scale = LaunchConfiguration("upper_joint_limit_scale")
    lower_joint_limit_scale = LaunchConfiguration("lower_joint_limit_scale")
    use_rviz = LaunchConfiguration("use_rviz")

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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joint_state_topic",
                default_value="/joint_states_demo",
                description="Topic used by the demo JointState publisher.",
            ),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/robot_description_demo",
                description="Robot description topic for RViz RobotModel.",
            ),
            DeclareLaunchArgument(
                "tf_topic",
                default_value="/tf_demo",
                description="TF topic for demo isolation.",
            ),
            DeclareLaunchArgument(
                "tf_static_topic",
                default_value="/tf_static_demo",
                description="Static TF topic for demo isolation.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="20.0",
                description="JointState publish rate (Hz).",
            ),
            DeclareLaunchArgument(
                "max_abs_position_rad",
                default_value="3.14",
                description="Absolute bound for generated joint position.",
            ),
            DeclareLaunchArgument(
                "base_frequency_hz",
                default_value="0.1",
                description="Base frequency for sinusoidal joint motion.",
            ),
            DeclareLaunchArgument(
                "link_2_length",
                default_value="0.34",
                description="Length of link_2 cylinder in xacro model.",
            ),
            DeclareLaunchArgument(
                "tool0_offset_z",
                default_value="0.12",
                description="Fixed-joint Z offset from link_6 to tool0.",
            ),
            DeclareLaunchArgument(
                "upper_joint_limit_scale",
                default_value="1.0",
                description="Scale factor applied to symmetric +/- joint limits.",
            ),
            DeclareLaunchArgument(
                "lower_joint_limit_scale",
                default_value="1.0",
                description="Scale factor applied to symmetric +/- joint limits.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2 if true.",
            ),
            Node(
                package="ur3_joint_state_publisher_py",
                executable="joint_state_publisher_node",
                name="ur3_joint_state_publisher_node",
                output="screen",
                parameters=[
                    {
                        "publish_topic": joint_state_topic,
                        "publish_rate_hz": publish_rate_hz,
                        "max_abs_position_rad": max_abs_position_rad,
                        "base_frequency_hz": base_frequency_hz,
                    }
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("/joint_states", joint_state_topic),
                    ("/robot_description", robot_description_topic),
                    ("/tf", tf_topic),
                    ("/tf_static", tf_static_topic),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_path],
                remappings=[
                    ("/robot_description", robot_description_topic),
                    ("/tf", tf_topic),
                    ("/tf_static", tf_static_topic),
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
