from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("ur3e_pen2_control_py")
    config_path = f"{package_share}/config/virtual_pen.yaml"
    rviz_path = f"{package_share}/rviz/virtual_pen.rviz"

    arguments = [
        DeclareLaunchArgument("launch_joy", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("joy_topic", default_value="/joy"),
        DeclareLaunchArgument("joy_device_id", default_value="0"),
        DeclareLaunchArgument("joy_device_name", default_value=""),
        DeclareLaunchArgument("joy_deadzone", default_value="0.08"),
    ]
    joy = Node(
        package="joy",
        executable="joy_node",
        name="virtual_pen_joy_node",
        output="both",
        condition=IfCondition(LaunchConfiguration("launch_joy")),
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
                "autorepeat_rate": 100.0,
                "sticky_buttons": False,
            }
        ],
        remappings=[("/joy", LaunchConfiguration("joy_topic"))],
    )
    virtual_pen = Node(
        package="ur3e_pen2_control_py",
        executable="virtual_pen_node",
        name="virtual_pen_node",
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
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="virtual_pen_rviz",
        output="both",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )
    return LaunchDescription([*arguments, joy, virtual_pen, rviz])
