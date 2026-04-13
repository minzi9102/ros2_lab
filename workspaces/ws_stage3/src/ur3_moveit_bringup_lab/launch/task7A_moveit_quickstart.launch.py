from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur3",
        description="UR robot type for Task 7A.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.56.101",
        description="Robot or URSim IP address. Ignored by FakeSystem, but kept aligned with the official launch.",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Use fake hardware for the 7A Quickstart by default.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch RViz with the MoveIt motion planning panel.",
    )
    launch_servo_arg = DeclareLaunchArgument(
        "launch_servo",
        default_value="false",
        description="Keep Servo disabled during the 7A Quickstart.",
    )

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
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller",
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
            "launch_servo": LaunchConfiguration("launch_servo"),
        }.items(),
    )

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_mock_hardware_arg,
            launch_rviz_arg,
            launch_servo_arg,
            driver_launch,
            moveit_launch,
        ]
    )
