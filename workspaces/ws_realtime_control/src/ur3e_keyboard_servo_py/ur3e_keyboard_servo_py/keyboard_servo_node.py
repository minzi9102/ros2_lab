import time
from typing import Optional

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node

from .key_mapping import KeyAction, map_key
from .safety_limiter import SafetyLimiter, TwistCommand
from .terminal_key_reader import TerminalKeyReader


class KeyboardServoNode(Node):
    def __init__(self, key_reader: Optional[TerminalKeyReader] = None) -> None:
        super().__init__('ur3e_keyboard_servo')

        self.command_topic = self.declare_parameter(
            'command_topic',
            '/servo_node/delta_twist_cmds',
        ).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 30.0).value)
        self.key_timeout_sec = float(self.declare_parameter('key_timeout_sec', 0.20).value)
        self.linear_speed_mps = float(self.declare_parameter('linear_speed_mps', 0.02).value)
        self.enable_z = bool(self.declare_parameter('enable_z', False).value)
        self.enable_rotation = bool(self.declare_parameter('enable_rotation', False).value)

        self._publisher = self.create_publisher(TwistStamped, self.command_topic, 10)
        self._limiter = SafetyLimiter(
            linear_speed_mps=self.linear_speed_mps,
            key_timeout_sec=self.key_timeout_sec,
            enable_z=self.enable_z,
            enable_rotation=self.enable_rotation,
        )
        self._key_reader = key_reader or TerminalKeyReader()
        self._quit_requested = False

        period_sec = 1.0 / self.publish_rate_hz
        self._timer = self.create_timer(period_sec, self._on_timer)

        self.get_logger().info(
            'Keyboard Servo node started. command_topic=%s frame_id=%s rate=%.1fHz speed=%.4fm/s timeout=%.2fs',
            self.command_topic,
            self.frame_id,
            self.publish_rate_hz,
            self.linear_speed_mps,
            self.key_timeout_sec,
        )

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def close(self) -> None:
        self._key_reader.close()

    def publish_stop(self) -> None:
        self._limiter.stop()
        self._publish_twist(TwistCommand())

    def _on_timer(self) -> None:
        now_sec = time.monotonic()
        raw_key = self._key_reader.read_key()
        key_command = map_key(raw_key)

        twist = self._limiter.apply_key_command(key_command, now_sec)
        self._publish_twist(twist)

        if key_command.action == KeyAction.QUIT:
            self._quit_requested = True
            self.get_logger().info('Quit requested. Publishing stop command before shutdown.')
            rclpy.shutdown()

    def _publish_twist(self, command: TwistCommand) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = command.linear_x
        msg.twist.linear.y = command.linear_y
        msg.twist.linear.z = command.linear_z
        msg.twist.angular.x = command.angular_x
        msg.twist.angular.y = command.angular_y
        msg.twist.angular.z = command.angular_z
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardServoNode()

    try:
        node._key_reader.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Publishing stop command.')
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
