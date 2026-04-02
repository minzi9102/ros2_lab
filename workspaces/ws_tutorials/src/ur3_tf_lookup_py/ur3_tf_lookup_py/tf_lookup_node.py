from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Ur3TfLookupNode(Node):
    def __init__(self) -> None:
        super().__init__("ur3_tf_lookup_node")

        self.source_frame = str(self.declare_parameter("source_frame", "base_link").value)
        self.target_frame = str(self.declare_parameter("target_frame", "tool0").value)
        self.query_rate_hz = float(self.declare_parameter("query_rate_hz", 5.0).value)
        self.timeout_sec = float(self.declare_parameter("timeout_sec", 0.2).value)
        self.use_sim_time = bool(self.get_parameter("use_sim_time").value)

        if self.query_rate_hz <= 0.0:
            self.get_logger().warn("query_rate_hz <= 0.0, fallback to 5.0")
            self.query_rate_hz = 5.0

        if self.timeout_sec < 0.0:
            self.get_logger().warn("timeout_sec < 0.0, fallback to 0.2")
            self.timeout_sec = 0.2

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.lookup_ok_count = 0
        self.lookup_fail_count = 0
        self.last_error: Optional[str] = None
        self._todo_reminder_printed = False

        self.timer = self.create_timer(1.0 / self.query_rate_hz, self.on_timer)

        self.get_logger().info(
            "tf lookup skeleton started: source=%s target=%s query_rate_hz=%.2f timeout_sec=%.3f use_sim_time=%s"
            % (
                self.source_frame,
                self.target_frame,
                self.query_rate_hz,
                self.timeout_sec,
                self.use_sim_time,
            )
        )

    def on_timer(self) -> None:
        # TODO(human): implement the lookup strategy and exception handling.
        # Suggested milestones:
        # 1. Query transform with explicit timeout.
        # 2. Catch Lookup/Connectivity/Extrapolation related exceptions.
        # 3. Decide retry behavior and log level for each exception category.
        # 4. Print stable pose fields (translation + quaternion, optional RPY).
        #
        # Starting point API hint (do not copy blindly):
        # transform = self.tf_buffer.lookup_transform(
        #     self.source_frame,
        #     self.target_frame,
        #     Time(),
        #     timeout=Duration(seconds=self.timeout_sec),
        # )
        try:
            _ = self.tf_buffer.can_transform(
                self.source_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=self.timeout_sec),
            )
        except TransformException as exc:
            self.lookup_fail_count += 1
            self.last_error = str(exc)
            return

        if not self._todo_reminder_printed:
            self.get_logger().warn(
                "TODO(human): complete lookup_transform + exception branches in on_timer()."
            )
            self._todo_reminder_printed = True



def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur3TfLookupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
