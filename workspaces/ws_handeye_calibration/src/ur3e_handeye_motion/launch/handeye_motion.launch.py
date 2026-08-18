from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ip = LaunchConfiguration('robot_ip')
    reverse_ip = LaunchConfiguration('reverse_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    real_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ur3_real_bringup_lab'),
                    'launch',
                    'task8B_readonly_bringup.launch.py',
                ]
            )
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'robot_ip': robot_ip,
            'reverse_ip': reverse_ip,
            'launch_rviz': 'false',
            'activate_joint_controller': 'true',
            'launch_dashboard_client': 'true',
            'manage_external_control': 'true',
            'stop_external_control_on_shutdown': 'true',
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'launch_rviz': launch_rviz,
            'launch_servo': 'false',
        }.items(),
    )

    motion_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ur3e_handeye_motion'),
                    'launch',
                    'motion_server.launch.py',
                ]
            )
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('robot_ip', default_value='192.168.56.101'),
            DeclareLaunchArgument('reverse_ip', default_value='192.168.56.2'),
            DeclareLaunchArgument('launch_rviz', default_value='true'),
            real_bringup,
            moveit,
            motion_server,
        ]
    )
