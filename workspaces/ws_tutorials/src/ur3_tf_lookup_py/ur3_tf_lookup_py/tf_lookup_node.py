from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformException,
    TransformListener,
)


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
        self.lookup_timeout = Duration(seconds=self.timeout_sec)

        self.lookup_ok_count = 0
        self.lookup_fail_count = 0
        self.last_error: Optional[str] = None

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
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                Time(),
                timeout=self.lookup_timeout,
            )
        except ExtrapolationException as exc:
            self.lookup_fail_count += 1
            current_error = f"extrapolation: {exc}"
            if self.last_error != current_error or self.lookup_fail_count % 20 == 0:
                self.get_logger().debug(
                    "tf lookup extrapolation (%s <- %s): %s"
                    % (self.source_frame, self.target_frame, exc)
                )
            self.last_error = current_error
            return
        except ConnectivityException as exc:
            self.lookup_fail_count += 1
            current_error = f"connectivity: {exc}"
            if self.last_error != current_error or self.lookup_fail_count % 10 == 0:
                self.get_logger().warn(
                    "tf lookup connectivity issue (%s <- %s): %s"
                    % (self.source_frame, self.target_frame, exc)
                )
            self.last_error = current_error
            return
        except LookupException as exc:
            self.lookup_fail_count += 1
            current_error = f"lookup: {exc}"
            if self.last_error != current_error or self.lookup_fail_count % 10 == 0:
                self.get_logger().warn(
                    "tf lookup frame missing (%s <- %s): %s"
                    % (self.source_frame, self.target_frame, exc)
                )
            self.last_error = current_error
            return
        except TransformException as exc:
            self.lookup_fail_count += 1
            current_error = f"transform: {exc}"
            if self.last_error != current_error or self.lookup_fail_count % 5 == 0:
                self.get_logger().error(
                    "tf lookup failed (%s <- %s): %s"
                    % (self.source_frame, self.target_frame, exc)
                )
            self.last_error = current_error
            return
        self.lookup_ok_count += 1

        if self.last_error is not None:
            self.get_logger().info(
                "tf lookup recovered: source=%s target=%s ok=%d fail=%d"
                % (
                    self.source_frame,
                    self.target_frame,
                    self.lookup_ok_count,
                    self.lookup_fail_count,
                )
            )
            self.last_error = None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.get_logger().info(
            "tf pose source=%s target=%s xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)"
            % (
                self.source_frame,
                self.target_frame,
                translation.x,
                translation.y,
                translation.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
        )



def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur3TfLookupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
