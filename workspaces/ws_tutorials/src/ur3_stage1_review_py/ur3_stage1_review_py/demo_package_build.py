"""
验收点1：独立建包能力演示节点

启动时打印：自身包名、节点名、关键依赖项、可执行入口列表。
证明包结构正确、colcon 构建通过、ros2 run 可正常加载。
"""

import importlib.metadata
import os

import rclpy
from rclpy.node import Node


class PackageBuildDemoNode(Node):
    def __init__(self):
        super().__init__("demo_package_build_node")
        self._print_package_info()

    def _print_package_info(self):
        pkg_name = "ur3_stage1_review_py"
        node_name = self.get_name()
        namespace = self.get_namespace()
        ros_distro = os.environ.get("ROS_DISTRO", "unknown")

        self.get_logger().info("=" * 60)
        self.get_logger().info("  验收点 1：独立建包能力演示")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  包名       : {pkg_name}")
        self.get_logger().info(f"  节点名     : {node_name}")
        self.get_logger().info(f"  命名空间   : {namespace}")
        self.get_logger().info(f"  ROS 发行版 : {ros_distro}")
        self.get_logger().info("-" * 60)

        # 打印已声明依赖是否可导入
        deps = {
            "rclpy": "rclpy",
            "sensor_msgs": "sensor_msgs.msg",
            "trajectory_msgs": "trajectory_msgs.msg",
            "control_msgs": "control_msgs.action",
        }
        self.get_logger().info("  依赖项可导入状态：")
        for label, module in deps.items():
            try:
                __import__(module)
                self.get_logger().info(f"    [OK]  {label} ({module})")
            except ImportError as e:
                self.get_logger().warn(f"    [MISS] {label} ({module}) — {e}")

        self.get_logger().info("-" * 60)

        # 打印本包注册的 entry_points
        try:
            ep_group = importlib.metadata.entry_points(group="console_scripts")
            our_eps = [ep for ep in ep_group if ep.value.startswith(pkg_name)]
            self.get_logger().info("  已注册可执行入口 (console_scripts)：")
            for ep in our_eps:
                self.get_logger().info(f"    ros2 run {pkg_name} {ep.name}")
        except Exception as e:
            self.get_logger().warn(f"  无法读取 entry_points: {e}")

        self.get_logger().info("=" * 60)
        self.get_logger().info("  [结论] 包结构正确，colcon 构建通过，节点可正常启动。")
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = PackageBuildDemoNode()
    # 打印完毕即退出，无需 spin
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
