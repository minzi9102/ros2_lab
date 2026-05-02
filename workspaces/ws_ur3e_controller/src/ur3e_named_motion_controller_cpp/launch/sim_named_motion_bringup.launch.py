from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur3e',
        description='UR robot type for the named motion controller.',
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='Robot or URSim IP. Fake hardware keeps this only for launch compatibility.',
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use fake hardware by default. Set false only for a reviewed real run.',
    )
    runtime_mode_arg = DeclareLaunchArgument(
        'runtime_mode',
        default_value='sim',
        description='Pass sim or real to the named controller.',
    )
    execute_arg = DeclareLaunchArgument(
        'execute',
        default_value='false',
        description='Allow service requests with execute=true. Default is plan-only.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz with MoveIt.',
    )
    moveit_launch_rviz_arg = SetLaunchConfiguration(
        'ur3e_controller_moveit_launch_rviz',
        LaunchConfiguration('launch_rviz'),
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'launch_rviz': 'false',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': LaunchConfiguration('ur3e_controller_moveit_launch_rviz'),
            'launch_servo': 'false',
        }.items(),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ur3e_named_motion_controller_cpp'),
                    'launch',
                    'named_motion_controller.launch.py',
                ]
            )
        ),
        launch_arguments={
            'runtime_mode': LaunchConfiguration('runtime_mode'),
            'execute': LaunchConfiguration('execute'),
        }.items(),
    )

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_mock_hardware_arg,
            runtime_mode_arg,
            execute_arg,
            launch_rviz_arg,
            moveit_launch_rviz_arg,
            driver_launch,
            moveit_launch,
            controller_launch,
        ]
    )
