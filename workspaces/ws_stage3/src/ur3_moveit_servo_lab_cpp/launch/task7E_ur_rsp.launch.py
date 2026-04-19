from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        description="UR robot type forwarded to the upstream robot_state_publisher launch.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="Robot IP forwarded to the upstream robot_state_publisher launch.",
    )

    upstream_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "description_file": PathJoinSubstitution(
                [FindPackageShare("ur3_moveit_servo_lab_cpp"), "urdf", "task7E_ur.urdf.xacro"]
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            upstream_rsp,
        ]
    )
