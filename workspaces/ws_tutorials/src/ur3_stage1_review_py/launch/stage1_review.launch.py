"""
Stage 1 验收复盘 — 一键启动 Launch 文件

依次运行三个验收演示节点，每个节点完成后自动退出。
用法：
  ros2 launch ur3_stage1_review_py stage1_review.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = "ur3_stage1_review_py"

    node_build = Node(
        package=pkg,
        executable="demo_package_build",
        name="demo_package_build_node",
        output="screen",
    )

    node_action = Node(
        package=pkg,
        executable="demo_action_rationale",
        name="demo_action_rationale_node",
        output="screen",
    )

    node_desc = Node(
        package=pkg,
        executable="demo_description_reader",
        name="demo_description_reader_node",
        output="screen",
    )

    return LaunchDescription([
        node_build,
        node_action,
        node_desc,
    ])
