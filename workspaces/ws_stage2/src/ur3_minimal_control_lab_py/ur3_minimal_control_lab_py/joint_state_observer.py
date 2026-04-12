import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState


class Ur3JointStateObserver(Node):
    def __init__(self) -> None:
        super().__init__("ur3_joint_state_observer")

        self.joint_state_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.print_rate_hz = float(self.declare_parameter("print_rate_hz", 1.0).value)
        self.expected_joints = list(
            self.declare_parameter(
                "expected_joints",
                [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            ).value
        )

        if self.print_rate_hz <= 0.0:
            self.get_logger().warn("print_rate_hz <= 0.0, fallback to 1.0")
            self.print_rate_hz = 1.0

        self._last_msg: JointState | None = None
        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.create_timer(1.0 / self.print_rate_hz, self._on_timer)

        self.get_logger().info(
            f"joint_state_observer started. topic={self.joint_state_topic} rate={self.print_rate_hz:.2f}Hz"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._last_msg = msg

    def _on_timer(self) -> None:
        if self._last_msg is None:
            self.get_logger().warn("Waiting for /joint_states ...")
            return

        index_by_name = {name: idx for idx, name in enumerate(self._last_msg.name)}
        parts: list[str] = []
        for joint_name in self.expected_joints:
            idx = index_by_name.get(joint_name)
            if idx is None or idx >= len(self._last_msg.position):
                parts.append(f"{joint_name}=N/A")
                continue

            rad = self._last_msg.position[idx]
            deg = math.degrees(rad)
            parts.append(f"{joint_name}={rad:.3f}rad({deg:.1f}deg)")

        self.get_logger().info(" | ".join(parts))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur3JointStateObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
