import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def generate_launch_description() -> LaunchDescription:
    demo_mode_arg = DeclareLaunchArgument(
        "demo_mode",
        default_value="both",
        description="Task 7C demo mode: pose, cartesian or both.",
    )
    execute_plan_arg = DeclareLaunchArgument(
        "execute_plan",
        default_value="false",
        description="Execute trajectories when planning succeeds.",
    )
    kinematics_params = {
        # 与 7B 保持一致：给 7C 本地 MoveGroupInterface 模型加载补上 kinematics 参数。
        "robot_description_kinematics": load_yaml("ur_moveit_config", "config/kinematics.yaml"),
    }

    scene_node = Node(
        package="ur3_moveit_scene_lab_cpp",
        executable="scene_cartesian_demo_node",
        name="ur3_scene_cartesian_demo",
        output="screen",
        parameters=[
            {
                "demo_mode": LaunchConfiguration("demo_mode"),
                "execute_plan": LaunchConfiguration("execute_plan"),
            },
            kinematics_params,
        ],
    )

    return LaunchDescription(
        [
            demo_mode_arg,
            execute_plan_arg,
            scene_node,
        ]
    )
