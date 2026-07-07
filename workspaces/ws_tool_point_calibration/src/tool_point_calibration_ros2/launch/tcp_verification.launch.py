from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('parent_frame', default_value='tool0'),
            DeclareLaunchArgument('tcp_frame', default_value='calibrated_tcp'),
            DeclareLaunchArgument('tcp_x', default_value='0.00121417'),
            DeclareLaunchArgument('tcp_y', default_value='0.0311535'),
            DeclareLaunchArgument('tcp_z', default_value='0.173598'),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='calibrated_tcp_static_tf',
                output='screen',
                arguments=[
                    '--x',
                    LaunchConfiguration('tcp_x'),
                    '--y',
                    LaunchConfiguration('tcp_y'),
                    '--z',
                    LaunchConfiguration('tcp_z'),
                    '--roll',
                    '0',
                    '--pitch',
                    '0',
                    '--yaw',
                    '0',
                    '--frame-id',
                    LaunchConfiguration('parent_frame'),
                    '--child-frame-id',
                    LaunchConfiguration('tcp_frame'),
                ],
            ),
        ]
    )
