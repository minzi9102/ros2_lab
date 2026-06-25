from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("ur3e_pen_writing_control_py")
    config_path = f"{package_share}/config/pen_tool_model.yaml"
    rviz_path = f"{package_share}/rviz/stage1_pen_visualization.rviz"

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz for virtual pen writing visualization.",
    )
    joy_topic_arg = DeclareLaunchArgument(
        "joy_topic",
        default_value="/joy",
        description="sensor_msgs/Joy topic used by the pen writing visualizer.",
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
        description="Joystick deadzone used by joy_node and the visualizer mapping.",
    )
    joy_autorepeat_rate_arg = DeclareLaunchArgument(
        "joy_autorepeat_rate",
        default_value="100.0",
        description="Joystick autorepeat rate passed to joy_node.",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="pen_writing_joy_node",
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

    visualizer_node = Node(
        package="ur3e_pen_writing_control_py",
        executable="pen_writing_visualizer_node",
        name="pen_writing_visualizer_node",
        output="both",
        parameters=[
            config_path,
            {
                "joy_topic": LaunchConfiguration("joy_topic"),
                "joy_deadzone": ParameterValue(
                    LaunchConfiguration("joy_deadzone"),
                    value_type=float,
                ),
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="pen_writing_rviz",
        output="both",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            launch_rviz_arg,
            joy_topic_arg,
            joy_device_id_arg,
            joy_device_name_arg,
            joy_deadzone_arg,
            joy_autorepeat_rate_arg,
            joy_node,
            visualizer_node,
            rviz_node,
        ]
    )
