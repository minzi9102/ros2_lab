#!/usr/bin/env python3

import sys

import rclpy

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateStampRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("task7e_joint_state_stamp_relay")
        self.source_topic = self.declare_parameter("source_topic", "/joint_states").value
        self.target_topic = self.declare_parameter(
            "target_topic", "/task7e/joint_states_fresh"
        ).value
        self.publish_period_sec = float(
            self.declare_parameter("publish_period_sec", 0.02).value
        )
        self.latest_msg: JointState | None = None
        self.logged_source_msg = False
        self.logged_relay_msg = False

        source_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        relay_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.subscription = self.create_subscription(
            JointState,
            self.source_topic,
            self.on_joint_state,
            source_qos,
        )
        self.publisher = self.create_publisher(
            JointState,
            self.target_topic,
            relay_qos,
        )
        self.timer = self.create_timer(self.publish_period_sec, self.publish_latest)

        self.get_logger().info(
            "Relaying joint states from "
            f"{self.source_topic} to {self.target_topic} at {self.publish_period_sec:.3f}s "
            "with fresh header stamps."
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_msg = msg
        if not self.logged_source_msg:
            self.get_logger().info(
                f"Received the first source JointState on {self.source_topic}."
            )
            self.logged_source_msg = True

    def publish_latest(self) -> None:
        if self.latest_msg is None:
            return

        relay_msg = JointState()
        relay_msg.header = self.latest_msg.header
        relay_msg.header.stamp = self.get_clock().now().to_msg()
        relay_msg.name = list(self.latest_msg.name)
        relay_msg.position = list(self.latest_msg.position)
        relay_msg.velocity = list(self.latest_msg.velocity)
        relay_msg.effort = list(self.latest_msg.effort)

        self.publisher.publish(relay_msg)
        if not self.logged_relay_msg:
            self.get_logger().info(
                f"Publishing fresh-stamped JointState messages on {self.target_topic}."
            )
            self.logged_relay_msg = True


def main() -> int:
    rclpy.init(args=[])
    node = JointStateStampRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
