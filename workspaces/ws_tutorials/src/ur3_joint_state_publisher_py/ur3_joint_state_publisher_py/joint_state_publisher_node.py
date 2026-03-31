from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Ur3JointStatePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("ur3_joint_state_publisher_node")

        self.publish_topic = self.declare_parameter("publish_topic", "/joint_states").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        self.max_abs_position_rad = float(self.declare_parameter("max_abs_position_rad", 1.2).value)
        self.base_frequency_hz = float(self.declare_parameter("base_frequency_hz", 0.1).value)
        self.joint_names = list(
            self.declare_parameter(
                "joint_names",
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

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn("publish_rate_hz <= 0.0, fallback to 20.0 Hz")
            self.publish_rate_hz = 20.0

        if self.max_abs_position_rad <= 0.0:
            self.get_logger().warn("max_abs_position_rad <= 0.0, fallback to 1.2 rad")
            self.max_abs_position_rad = 1.2

        self.publisher = self.create_publisher(JointState, self.publish_topic, 10)
        self.start_time = self.get_clock().now()
        self.todo_reported = False

        period_sec = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period_sec, self.on_timer)

        self.get_logger().info(
            f"UR3 Python joint-state publisher started: topic={self.publish_topic} "
            f"rate={self.publish_rate_hz:.2f}Hz"
        )

    def compute_joint_positions(self, elapsed_sec: float) -> List[float]:
        """Compute one position vector in joint_names order."""
        # TODO(human): 请实现 6 轴关节位置生成逻辑（建议使用正弦函数 + 不同相位偏移）。
        # 要求：
        # 1) 返回长度必须等于 len(self.joint_names)
        # 2) 每个关节位置都要被限制在 [-self.max_abs_position_rad, self.max_abs_position_rad]
        # 3) 曲线应随时间连续变化，便于后续在 RViz/plot 中观察
        raise NotImplementedError(
            "TODO(human): implement compute_joint_positions(elapsed_sec) for learning exercise"
        )

    def on_timer(self) -> None:
        elapsed_sec = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        try:
            positions = self.compute_joint_positions(elapsed_sec)
        except NotImplementedError as exc:
            if not self.todo_reported:
                self.get_logger().error(str(exc))
                self.todo_reported = True
            return

        if len(positions) != len(self.joint_names):
            self.get_logger().error(
                "Invalid positions length: expected %d got %d",
                len(self.joint_names),
                len(positions),
            )
            return

        bounded_positions = []
        for pos in positions:
            bounded_positions.append(
                max(-self.max_abs_position_rad, min(self.max_abs_position_rad, float(pos)))
            )

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = bounded_positions
        msg.velocity = []
        msg.effort = []

        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur3JointStatePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
