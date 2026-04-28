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
    require_trajectory_controller_active_arg = DeclareLaunchArgument(
        "require_trajectory_controller_active",
        default_value="false",
        description=(
            "Set true only for pre-motion checks; Task 8C readonly checks keep "
            "trajectory controllers inactive."
        ),
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
                "require_trajectory_controller_active": LaunchConfiguration(
                    "require_trajectory_controller_active"
                ),
                "report_only": LaunchConfiguration("report_only"),
            }
        ],
    )

    return LaunchDescription(
        [
            require_external_control_arg,
            require_trajectory_controller_active_arg,
            report_only_arg,
            state_check,
        ]
    )
