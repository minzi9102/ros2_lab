from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        description="Real robot type, for example ur3e.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="Real robot IP address confirmed by Task 8A.",
    )

    kinematics_params_file = PathJoinSubstitution(
        [
            FindPackageShare("ur3_real_bringup_lab"),
            "config",
            "ur3e_real_calibration.yaml",
        ]
    )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "kinematics_params_file": kinematics_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            rsp_launch,
        ]
    )
