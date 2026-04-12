from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


def speed_scaling_qos_profile() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class Ur3SpeedScalingMonitor(Node):
    def __init__(self) -> None:
        super().__init__("ur3_speed_scaling_monitor")

        self.speed_scaling_topic = self.declare_parameter(
            "speed_scaling_topic", "/speed_scaling_state_broadcaster/speed_scaling"
        ).value
        self.print_rate_hz = float(self.declare_parameter("print_rate_hz", 1.0).value)
        self.change_threshold = float(self.declare_parameter("change_threshold", 1.0).value)

        if self.print_rate_hz <= 0.0:
            self.get_logger().warn("print_rate_hz <= 0.0, fallback to 1.0")
            self.print_rate_hz = 1.0

        if self.change_threshold < 0.0:
            self.get_logger().warn("change_threshold < 0.0, fallback to 0.0")
            self.change_threshold = 0.0

        self._latest_value: float | None = None
        self._last_change_logged_value: float | None = None
        self._latest_receive_time = None

        self.create_subscription(
            Float64,
            self.speed_scaling_topic,
            self._on_speed_scaling,
            speed_scaling_qos_profile(),
        )
        self.create_timer(1.0 / self.print_rate_hz, self._on_timer)

        self.get_logger().info(
            "speed_scaling_monitor started. "
            f"topic={self.speed_scaling_topic} rate={self.print_rate_hz:.2f}Hz "
            f"change_threshold={self.change_threshold:.2f}"
        )

    def _on_speed_scaling(self, msg: Float64) -> None:
        self._latest_value = float(msg.data)
        self._latest_receive_time = self.get_clock().now()

        should_log_change = self._last_change_logged_value is None or (
            abs(self._latest_value - self._last_change_logged_value) >= self.change_threshold
        )
        if should_log_change:
            self._last_change_logged_value = self._latest_value
            self.get_logger().info(self._format_value(prefix="Update"))

    def _on_timer(self) -> None:
        if self._latest_value is None or self._latest_receive_time is None:
            self.get_logger().warn("Waiting for speed scaling message ...")
            return

        age = self.get_clock().now() - self._latest_receive_time
        self.get_logger().info(self._format_value(prefix="Latest", age=age))

    def _format_value(self, prefix: str, age: Duration | None = None) -> str:
        value = 0.0 if self._latest_value is None else self._latest_value
        fraction = value / 100.0
        parts = [f"{prefix}: speed_scaling={value:.1f}% fraction={fraction:.3f}"]
        if age is not None:
            parts.append(f"age={age.nanoseconds / 1_000_000_000:.2f}s")
        return " ".join(parts)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur3SpeedScalingMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
