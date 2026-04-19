#!/usr/bin/env python3

import sys

import rclpy

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState


class WaitForJointStatesNode(Node):
    def __init__(self) -> None:
        super().__init__("task7e_joint_states_gate")
        self.topic = self.declare_parameter("topic", "/joint_states").value
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 15.0).value)


def main() -> int:
    rclpy.init(args=[])
    node = WaitForJointStatesNode()
    qos_profile = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    try:
        node.get_logger().info(
            f"Waiting for the first JointState message on {node.topic} for up to {node.timeout_sec:.1f}s."
        )
        received, _ = wait_for_message(
            JointState,
            node,
            node.topic,
            qos_profile=qos_profile,
            time_to_wait=node.timeout_sec,
        )
        if not received:
            node.get_logger().error(
                f"Timed out after {node.timeout_sec:.1f}s waiting for {node.topic}."
            )
            return 1

        node.get_logger().info(f"Received JointState traffic on {node.topic}.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
