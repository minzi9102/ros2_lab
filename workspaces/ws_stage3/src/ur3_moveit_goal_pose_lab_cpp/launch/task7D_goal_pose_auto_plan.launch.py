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
    goal_topic_arg = DeclareLaunchArgument(
        "goal_topic",
        default_value="/goal_pose",
        description="PoseStamped topic used by RViz 2D Goal Pose.",
    )
    execute_plan_arg = DeclareLaunchArgument(
        "execute_plan",
        default_value="true",
        description="Execute automatically after planning succeeds.",
    )
    kinematics_params = {
        # 与 7B 保持一致：给 7D 本地 MoveGroupInterface 的模型加载过程补上 kinematics 参数。
        "robot_description_kinematics": load_yaml("ur_moveit_config", "config/kinematics.yaml"),
    }

    executor_node = Node(
        package="ur3_moveit_goal_pose_lab_cpp",
        executable="goal_pose_executor_node",
        name="ur3_goal_pose_executor",
        output="screen",
        parameters=[
            {
                "goal_topic": LaunchConfiguration("goal_topic"),
                "execute_plan": LaunchConfiguration("execute_plan"),
            },
            kinematics_params,
        ],
    )

    return LaunchDescription(
        [
            goal_topic_arg,
            execute_plan_arg,
            executor_node,
        ]
    )
