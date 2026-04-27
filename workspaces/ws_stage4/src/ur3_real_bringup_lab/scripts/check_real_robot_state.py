#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class RealRobotStateCheck(Node):
    def __init__(self) -> None:
        super().__init__("task8c_state_check")
        self.declare_parameter("require_external_control", True)
        self.declare_parameter("report_only", True)

    def report(self) -> None:
        require_external_control = self.get_parameter(
            "require_external_control"
        ).get_parameter_value().bool_value
        report_only = self.get_parameter("report_only").get_parameter_value().bool_value

        self.get_logger().info("Task 8C state-check scaffold started.")
        self.get_logger().info(f"require_external_control={require_external_control}")
        self.get_logger().info(f"report_only={report_only}")
        self.get_logger().warn(
            "TODO(human): 根据现场 Dashboard、controller 和 External Control 状态填写 pass/warn/block 矩阵。"
        )
        self.get_logger().warn(
            "This scaffold never unlocks protective stop, restarts safety, or sends motion commands."
        )


def main() -> int:
    rclpy.init()
    node = RealRobotStateCheck()
    try:
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
