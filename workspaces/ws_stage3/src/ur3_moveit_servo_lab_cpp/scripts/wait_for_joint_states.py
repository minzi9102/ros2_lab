#!/usr/bin/env python3

import sys
import time

from controller_manager_msgs.srv import ListControllers
import rclpy

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import JointState


class WaitForJointStatesNode(Node):
    def __init__(self) -> None:
        super().__init__("task7e_joint_states_gate")
        self.topic = self.declare_parameter("topic", "/joint_states").value
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 15.0).value)
        self.controller_manager_service = self.declare_parameter(
            "controller_manager_service",
            "/controller_manager/list_controllers",
        ).value
        self.required_active_controllers = list(
            self.declare_parameter(
                "required_active_controllers",
                ["joint_state_broadcaster", "forward_position_controller"],
            ).value
        )


def wait_for_active_controllers(node: WaitForJointStatesNode) -> bool:
    if not node.required_active_controllers:
        node.get_logger().info("No controller activation gate configured.")
        return True

    client = node.create_client(ListControllers, node.controller_manager_service)
    deadline = time.monotonic() + node.timeout_sec

    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if not client.wait_for_service(timeout_sec=min(0.5, max(0.0, remaining))):
            node.get_logger().info(
                f"Waiting for controller manager service {node.controller_manager_service}."
            )
            continue

        future: Future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=min(0.5, max(0.0, remaining)))
        if not future.done():
            node.get_logger().info("ListControllers request did not complete yet, retrying.")
            continue

        response = future.result()
        if response is None:
            node.get_logger().warn("ListControllers returned no response, retrying.")
            continue

        controller_states = {controller.name: controller.state for controller in response.controller}
        missing = [
            controller_name
            for controller_name in node.required_active_controllers
            if controller_states.get(controller_name) != "active"
        ]
        if not missing:
            node.get_logger().info(
                "Required controllers are active: "
                + ", ".join(node.required_active_controllers)
            )
            return True

        node.get_logger().info(
            "Waiting for controllers to become active: "
            + ", ".join(
                f"{controller_name}={controller_states.get(controller_name, 'missing')}"
                for controller_name in node.required_active_controllers
            )
        )
        time.sleep(0.2)

    node.get_logger().error(
        "Timed out waiting for required active controllers: "
        + ", ".join(node.required_active_controllers)
    )
    return False


def main(args=None) -> int:
    rclpy.init(args=args)
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
        if not wait_for_active_controllers(node):
            return 1
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
