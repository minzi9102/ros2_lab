from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument('base_frame', default_value='base'),
            DeclareLaunchArgument('tcp_frame', default_value='calibrated_tcp'),
            DeclareLaunchArgument('num_samples', default_value='4'),
            DeclareLaunchArgument('lookup_timeout_sec', default_value='1.0'),
            Node(
                package='tool_point_calibration_ros2',
                executable='console_paper_calibration_node',
                name='console_paper_calibration',
                output='screen',
                emulate_tty=True,
                parameters=[
                    {
                        'base_frame': LaunchConfiguration('base_frame'),
                        'tcp_frame': LaunchConfiguration('tcp_frame'),
                        'num_samples': ParameterValue(
                            LaunchConfiguration('num_samples'), value_type=int
                        ),
                        'lookup_timeout_sec': ParameterValue(
                            LaunchConfiguration('lookup_timeout_sec'), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
