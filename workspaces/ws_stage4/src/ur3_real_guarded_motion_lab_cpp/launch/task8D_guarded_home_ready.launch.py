from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    execute_arg = DeclareLaunchArgument(
        "execute",
        default_value="false",
        description="Keep false for dry-run. True still requires human_confirmation.",
    )
    require_confirmation_arg = DeclareLaunchArgument(
        "require_confirmation",
        default_value="true",
        description="Require explicit confirmation token before any future real motion implementation.",
    )
    human_confirmation_arg = DeclareLaunchArgument(
        "human_confirmation",
        default_value="",
        description="Use I_CONFIRM_REAL_ROBOT_MOTION only after现场确认.",
    )
    target_name_arg = DeclareLaunchArgument(
        "target_name",
        default_value="ready",
        description="Guarded target name from task8D_guarded_targets.yaml.",
    )
    max_joint_delta_arg = DeclareLaunchArgument(
        "max_joint_delta_rad",
        default_value="0.10",
        description="Task 8D default conservative joint delta gate.",
    )
    final_position_tolerance_arg = DeclareLaunchArgument(
        "final_position_tolerance_rad",
        default_value="0.02",
        description="Allowed final joint error after the action result.",
    )

    targets_file = PathJoinSubstitution(
        [
            FindPackageShare("ur3_real_guarded_motion_lab_cpp"),
            "config",
            "task8D_guarded_targets.yaml",
        ]
    )

    guarded_node = Node(
        package="ur3_real_guarded_motion_lab_cpp",
        executable="guarded_joint_motion_node",
        name="task8d_guarded_joint_motion",
        output="screen",
        parameters=[
            targets_file,
            {
                "execute": LaunchConfiguration("execute"),
                "require_confirmation": LaunchConfiguration("require_confirmation"),
                "human_confirmation": LaunchConfiguration("human_confirmation"),
                "target_name": LaunchConfiguration("target_name"),
                "max_joint_delta_rad": LaunchConfiguration("max_joint_delta_rad"),
                "final_position_tolerance_rad": LaunchConfiguration(
                    "final_position_tolerance_rad"
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            execute_arg,
            require_confirmation_arg,
            human_confirmation_arg,
            target_name_arg,
            max_joint_delta_arg,
            final_position_tolerance_arg,
            guarded_node,
        ]
    )
