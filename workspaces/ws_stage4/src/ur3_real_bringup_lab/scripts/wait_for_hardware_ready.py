#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
from typing import Optional

from controller_manager_msgs.srv import ListControllers
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


DEFAULT_UR3E_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


class HardwareReadyGate(Node):
    def __init__(self) -> None:
        super().__init__("task8b_hardware_ready_gate")
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("poll_period_sec", 0.25)
        self.declare_parameter("list_controllers_service", "/controller_manager/list_controllers")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_state_broadcaster_name", "joint_state_broadcaster")
        self.declare_parameter("expected_joint_names", DEFAULT_UR3E_JOINT_NAMES)

        self._last_joint_state: Optional[JointState] = None
        self._controllers_client = self.create_client(
            ListControllers,
            self._param_string("list_controllers_service"),
        )
        self.create_subscription(
            JointState,
            self._param_string("joint_state_topic"),
            self._on_joint_state,
            10,
        )

    def wait_until_ready(self) -> bool:
        timeout_sec = self.get_parameter("timeout_sec").get_parameter_value().double_value
        poll_period_sec = self.get_parameter("poll_period_sec").get_parameter_value().double_value
        deadline = time.monotonic() + timeout_sec

        self.get_logger().info(
            "Waiting for controller manager and complete /joint_states before launching dashboard client."
        )
        last_reason = "not checked yet"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            ready, reason = self._ready_once()
            if ready:
                self.get_logger().info(f"Hardware ready gate passed: {reason}")
                return True
            if reason != last_reason:
                self.get_logger().info(f"Hardware ready gate waiting: {reason}")
                last_reason = reason
            time.sleep(poll_period_sec)

        self.get_logger().error(
            f"Hardware ready gate timed out after {timeout_sec:.1f}s: {last_reason}"
        )
        return False

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_joint_state = msg

    def _ready_once(self) -> tuple[bool, str]:
        controllers = self._list_controllers()
        if controllers is None:
            return False, "controller manager list_controllers service is unavailable or timed out"

        controller_name = self._param_string("joint_state_broadcaster_name")
        state = self._controller_state(controllers, controller_name)
        if state != "active":
            if state is None:
                return False, f"{controller_name} is missing"
            return False, f"{controller_name} is {state}, expected active"

        joint_state_ready, joint_state_reason = self._joint_state_ready()
        if not joint_state_ready:
            return False, joint_state_reason

        return True, f"{controller_name}=active and complete JointState received"

    def _list_controllers(self) -> Optional[ListControllers.Response]:
        if not self._controllers_client.wait_for_service(timeout_sec=0.5):
            return None
        future = self._controllers_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done():
            return None
        return future.result()

    def _controller_state(
        self, response: ListControllers.Response, controller_name: str
    ) -> Optional[str]:
        for controller in response.controller:
            if controller.name == controller_name:
                return controller.state
        return None

    def _joint_state_ready(self) -> tuple[bool, str]:
        if self._last_joint_state is None:
            return False, "no JointState sample received yet"

        expected_names = set(self._expected_joint_names())
        present_names = set(self._last_joint_state.name)
        missing_names = sorted(expected_names - present_names)
        if missing_names:
            return False, f"JointState missing expected joints: {missing_names}"

        if len(self._last_joint_state.position) < len(self._last_joint_state.name):
            return False, "JointState position array is shorter than name array"

        return True, "complete JointState received"

    def _expected_joint_names(self) -> list[str]:
        return list(
            self.get_parameter("expected_joint_names")
            .get_parameter_value()
            .string_array_value
        )

    def _param_string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value


def main() -> int:
    rclpy.init()
    node = HardwareReadyGate()
    try:
        return 0 if node.wait_until_ready() else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
