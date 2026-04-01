import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ur3_qos_lab_py.qos_profile_utils import make_qos_profile


class QosPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("qos_publisher_node")

        self.topic_name = self.declare_parameter("topic_name", "/ur3/qos_lab").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 100.0).value)
        self.payload_size_bytes = int(self.declare_parameter("payload_size_bytes", 64).value)
        self.reliability = str(self.declare_parameter("reliability", "reliable").value)
        self.history = str(self.declare_parameter("history", "keep_last").value)
        self.depth = int(self.declare_parameter("depth", 10).value)
        self.report_period_sec = float(self.declare_parameter("report_period_sec", 1.0).value)

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn("publish_rate_hz <= 0.0, fallback to 10.0 Hz")
            self.publish_rate_hz = 10.0
        if self.payload_size_bytes < 0:
            self.get_logger().warn("payload_size_bytes < 0, fallback to 0")
            self.payload_size_bytes = 0
        if self.report_period_sec <= 0.0:
            self.get_logger().warn("report_period_sec <= 0.0, fallback to 1.0")
            self.report_period_sec = 1.0

        qos_profile = make_qos_profile(self.reliability, self.history, self.depth)
        self.publisher = self.create_publisher(String, self.topic_name, qos_profile)

        self.seq = 0
        self.sent_count = 0

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)
        self.report_timer = self.create_timer(self.report_period_sec, self.on_report)

        self.get_logger().info(
            "QoS publisher started: topic=%s rate=%.2fHz payload=%dB reliability=%s history=%s depth=%d"
            % (
                self.topic_name,
                self.publish_rate_hz,
                self.payload_size_bytes,
                self.reliability,
                self.history,
                self.depth,
            )
        )

    def build_payload(self, seq: int, stamp_ns: int) -> str:
        base = f"{seq},{stamp_ns}"
        if self.payload_size_bytes <= len(base):
            return base

        # Keep payload deterministic so only QoS is the main variable.
        pad = "x" * (self.payload_size_bytes - len(base) - 1)
        return f"{base},{pad}" if pad else base

    def on_timer(self) -> None:
        stamp_ns = self.get_clock().now().nanoseconds
        msg = String()
        msg.data = self.build_payload(self.seq, stamp_ns)
        self.publisher.publish(msg)

        self.seq += 1
        self.sent_count += 1

    def on_report(self) -> None:
        self.get_logger().info("sent_count=%d seq=%d" % (self.sent_count, self.seq))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = QosPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
