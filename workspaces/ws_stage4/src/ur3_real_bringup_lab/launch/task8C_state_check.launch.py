from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    require_external_control_arg = DeclareLaunchArgument(
        "require_external_control",
        default_value="true",
        description="Record whether External Control is required before motion tasks.",
    )
    report_only_arg = DeclareLaunchArgument(
        "report_only",
        default_value="true",
        description="Task 8C scaffold only reports state expectations and never recovers faults.",
    )

    state_check = Node(
        package="ur3_real_bringup_lab",
        executable="check_real_robot_state.py",
        name="task8c_state_check",
        output="screen",
        parameters=[
            {
                "require_external_control": LaunchConfiguration("require_external_control"),
                "report_only": LaunchConfiguration("report_only"),
            }
        ],
    )

    return LaunchDescription(
        [
            require_external_control_arg,
            report_only_arg,
            state_check,
        ]
    )
