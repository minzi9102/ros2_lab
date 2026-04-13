from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    planning_group_arg = DeclareLaunchArgument(
        "planning_group",
        default_value="ur_manipulator",
        description="MoveIt planning group used by the Task 7B node.",
    )
    target_mode_arg = DeclareLaunchArgument(
        "target_mode",
        default_value="joint",
        description="Planning target mode: joint or pose.",
    )
    execute_plan_arg = DeclareLaunchArgument(
        "execute_plan",
        default_value="false",
        description="Execute the plan after planning succeeds.",
    )

    planner_node = Node(
        package="ur3_moveit_move_group_lab_cpp",
        executable="move_group_planner_node",
        name="ur3_move_group_planner",
        output="screen",
        parameters=[
            {
                "planning_group": LaunchConfiguration("planning_group"),
                "target_mode": LaunchConfiguration("target_mode"),
                "execute_plan": LaunchConfiguration("execute_plan"),
            }
        ],
    )

    return LaunchDescription(
        [
            planning_group_arg,
            target_mode_arg,
            execute_plan_arg,
            planner_node,
        ]
    )
