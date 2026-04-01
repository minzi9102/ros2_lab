from collections import deque
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ur3_qos_lab_py.qos_profile_utils import make_qos_profile


class QosSubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("qos_subscriber_node")

        self.topic_name = self.declare_parameter("topic_name", "/ur3/qos_lab").value
        self.reliability = str(self.declare_parameter("reliability", "reliable").value)
        self.history = str(self.declare_parameter("history", "keep_last").value)
        self.depth = int(self.declare_parameter("depth", 10).value)
        self.report_period_sec = float(self.declare_parameter("report_period_sec", 1.0).value)

        if self.report_period_sec <= 0.0:
            self.get_logger().warn("report_period_sec <= 0.0, fallback to 1.0")
            self.report_period_sec = 1.0

        qos_profile = make_qos_profile(self.reliability, self.history, self.depth)
        self.subscription = self.create_subscription(
            String,
            self.topic_name,
            self.on_message,
            qos_profile,
        )

        self.rx_count = 0
        self.parse_fail_count = 0
        self.last_seq: Optional[int] = None
        self.latest_latency_ms: Optional[float] = None
        self.latency_window_ms = deque(maxlen=2000)

        self.report_timer = self.create_timer(self.report_period_sec, self.on_report)

        self.get_logger().info(
            "QoS subscriber started: topic=%s reliability=%s history=%s depth=%d"
            % (self.topic_name, self.reliability, self.history, self.depth)
        )

    def on_message(self, msg: String) -> None:
        self.rx_count += 1

        parts = msg.data.split(",", 2)
        if len(parts) < 2:
            self.parse_fail_count += 1
            return

        try:
            seq = int(parts[0])
            stamp_ns = int(parts[1])
        except ValueError:
            self.parse_fail_count += 1
            return

        now_ns = self.get_clock().now().nanoseconds
        self.latest_latency_ms = max(0.0, (now_ns - stamp_ns) / 1e6)
        self.latency_window_ms.append(self.latest_latency_ms)

        # TODO(human): implement packet-loss estimation using seq continuity.
        # Suggested output: expected_count, lost_count, loss_rate_percent.
        self.last_seq = seq

        # TODO(human): implement windowed latency statistics (p50/p95/max).
        # Keep the same time window across experiment groups for fair comparison.

    def on_report(self) -> None:
        summary = (
            "rx_count=%d parse_fail=%d latest_latency_ms=%s"
            % (
                self.rx_count,
                self.parse_fail_count,
                f"{self.latest_latency_ms:.3f}" if self.latest_latency_ms is not None else "N/A",
            )
        )
        self.get_logger().info(summary)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QosSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
