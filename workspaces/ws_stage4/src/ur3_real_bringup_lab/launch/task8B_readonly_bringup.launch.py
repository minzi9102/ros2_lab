from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        description="Real robot type, for example ur3 or ur3e.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="Real robot IP address confirmed by Task 8A.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Keep RViz off by default during readonly bringup.",
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
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
        }.items(),
    )

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            launch_rviz_arg,
            driver_launch,
        ]
    )
