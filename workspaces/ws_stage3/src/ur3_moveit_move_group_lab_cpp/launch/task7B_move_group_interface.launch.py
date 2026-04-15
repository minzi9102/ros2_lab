import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


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
    kinematics_params = {
        # 最小修复：给 7B 自己的 MoveGroupInterface 本地模型加载过程补上 kinematics 参数。
        "robot_description_kinematics": load_yaml("ur_moveit_config", "config/kinematics.yaml"),
    }

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
            },
            kinematics_params,
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
