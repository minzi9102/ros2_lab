import time
from typing import Optional

from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
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
        self.command_type_service = self.declare_parameter(
            'command_type_service',
            '/servo_node/switch_command_type',
        ).value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 30.0).value)
        self.key_timeout_sec = float(self.declare_parameter('key_timeout_sec', 0.20).value)
        self.linear_speed_mps = float(self.declare_parameter('linear_speed_mps', 0.02).value)
        self.enable_z = bool(self.declare_parameter('enable_z', False).value)
        self.enable_rotation = bool(self.declare_parameter('enable_rotation', False).value)

        self._publisher = self.create_publisher(TwistStamped, self.command_topic, 10)
        self._command_type_client = self.create_client(
            ServoCommandType,
            self.command_type_service,
        )
        self._command_type_future = None
        self._twist_mode_ready = False
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
            'Keyboard Servo node started. '
            f'command_topic={self.command_topic} '
            f'command_type_service={self.command_type_service} '
            f'frame_id={self.frame_id} '
            f'rate={self.publish_rate_hz:.1f}Hz '
            f'speed={self.linear_speed_mps:.4f}m/s '
            f'timeout={self.key_timeout_sec:.2f}s'
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
        if not self._ensure_twist_mode_ready():
            return

        now_sec = time.monotonic()
        raw_key = self._key_reader.read_key()
        key_command = map_key(raw_key)
        if key_command.action != KeyAction.IGNORE:
            self.get_logger().info(
                f'Key command received: action={key_command.action.value} '
                f'x={key_command.x:.1f} y={key_command.y:.1f}'
            )

        twist = self._limiter.apply_key_command(key_command, now_sec)
        self._publish_twist(twist)

        if key_command.action == KeyAction.QUIT:
            self._quit_requested = True
            self.get_logger().info('Quit requested. Publishing stop command before shutdown.')
            rclpy.shutdown()

    def _ensure_twist_mode_ready(self) -> bool:
        if self._twist_mode_ready:
            return True

        if self._command_type_future is None:
            if not self._command_type_client.service_is_ready():
                self.get_logger().info(
                    f'Waiting for Servo command type service: {self.command_type_service}'
                )
                return False

            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.TWIST
            self._command_type_future = self._command_type_client.call_async(request)
            self.get_logger().info('Requested MoveIt Servo TWIST command mode.')
            return False

        if not self._command_type_future.done():
            return False

        response = self._command_type_future.result()
        if response is None or not response.success:
            self.get_logger().error('MoveIt Servo rejected TWIST command mode request.')
            rclpy.shutdown()
            return False

        self._twist_mode_ready = True
        self.get_logger().info('MoveIt Servo accepted TWIST command mode.')
        return True

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
        if node._key_reader.is_interactive:
            node.get_logger().info(
                f'Keyboard input attached to {node._key_reader.source_name}. '
                'Keep this terminal focused while pressing keys.'
            )
        else:
            node.get_logger().warn(
                'Keyboard input is not interactive. Run from a real terminal or start '
                'keyboard_servo_node separately to control motion.'
            )
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
