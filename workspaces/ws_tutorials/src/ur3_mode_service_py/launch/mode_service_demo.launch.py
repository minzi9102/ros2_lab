from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3_mode_service_py',
            executable='mode_service_server',
            output='screen',
        ),
    ])
