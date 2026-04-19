#!/usr/bin/env python3

import sys

import rclpy

from moveit_msgs.msg import ServoStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.wait_for_message import wait_for_message


class WaitForServoStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("task7e_servo_status_gate")
        self.topic = self.declare_parameter("topic", "/servo_node/status").value
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 15.0).value)


def main(args=None) -> int:
    rclpy.init(args=args)
    node = WaitForServoStatusNode()
    qos_profile = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    try:
        node.get_logger().info(
            f"Waiting for the first ServoStatus message on {node.topic} for up to {node.timeout_sec:.1f}s."
        )
        received, msg = wait_for_message(
            ServoStatus,
            node,
            node.topic,
            qos_profile=qos_profile,
            time_to_wait=node.timeout_sec,
        )
        if not received or msg is None:
            node.get_logger().error(
                f"Timed out after {node.timeout_sec:.1f}s waiting for {node.topic}."
            )
            return 1

        node.get_logger().info(
            f"Received ServoStatus on {node.topic}: code={msg.code} message='{msg.message}'"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
