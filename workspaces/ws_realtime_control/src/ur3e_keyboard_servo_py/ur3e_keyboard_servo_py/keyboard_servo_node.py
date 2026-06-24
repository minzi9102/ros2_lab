import time
from typing import Optional

from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .evdev_key_reader import EvdevKeyReader, KeyEventValue
from .joy_mapping import JoyControl, JoyMapper
from .key_mapping import KeyAction, map_key
from .pressed_key_state import PressedKeyState
from .safety_limiter import SafetyLimiter, TwistCommand
from .smooth_velocity import SmoothVelocityController
from .terminal_key_reader import TerminalKeyReader


REQUIRED_REAL_CONFIRMATION = 'I_CONFIRM_REAL_ROBOT_MOTION'
SUPPORTED_INPUT_BACKENDS = ('terminal', 'evdev', 'joy')
SUPPORTED_COMMAND_FRAMES = ('base_link', 'tool0')


def is_motion_confirmation_valid(
    *,
    require_confirmation: bool,
    human_confirmation: str,
    required_confirmation_text: str = REQUIRED_REAL_CONFIRMATION,
) -> bool:
    if not require_confirmation:
        return True
    return human_confirmation == required_confirmation_text


class KeyboardServoNode(Node):
    def __init__(
        self,
        key_reader: Optional[TerminalKeyReader] = None,
        evdev_reader: Optional[EvdevKeyReader] = None,
    ) -> None:
        super().__init__('ur3e_keyboard_servo')

        self.command_topic = self.declare_parameter(
            'command_topic',
            '/servo_node/delta_twist_cmds',
        ).value
        self.command_type_service = self.declare_parameter(
            'command_type_service',
            '/servo_node/switch_command_type',
        ).value
        self.frame_id = str(self.declare_parameter('frame_id', 'base_link').value)
        self.input_backend = str(
            self.declare_parameter('input_backend', 'terminal').value
        )
        self.input_device = str(self.declare_parameter('input_device', '').value)
        self.joy_topic = str(self.declare_parameter('joy_topic', '/joy').value)
        self.joy_deadzone = float(self.declare_parameter('joy_deadzone', 0.08).value)
        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 30.0).value)
        self.key_timeout_sec = float(self.declare_parameter('key_timeout_sec', 0.20).value)
        self.linear_speed_mps = float(self.declare_parameter('linear_speed_mps', 0.02).value)
        self.acceleration_mps2 = float(
            self.declare_parameter('acceleration_mps2', 0.50).value
        )
        self.deceleration_mps2 = float(
            self.declare_parameter('deceleration_mps2', 0.80).value
        )
        self.enable_z = bool(self.declare_parameter('enable_z', False).value)
        self.enable_rotation = bool(self.declare_parameter('enable_rotation', False).value)
        self.require_confirmation = bool(
            self.declare_parameter('require_confirmation', False).value
        )
        self.human_confirmation = self.declare_parameter('human_confirmation', '').value
        self.required_confirmation_text = self.declare_parameter(
            'required_confirmation_text',
            REQUIRED_REAL_CONFIRMATION,
        ).value
        self.max_session_duration_sec = float(
            self.declare_parameter('max_session_duration_sec', 0.0).value
        )

        if self.max_session_duration_sec < 0.0:
            raise ValueError('max_session_duration_sec must be non-negative')
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        if self.input_backend not in SUPPORTED_INPUT_BACKENDS:
            raise ValueError(
                f'input_backend must be one of {SUPPORTED_INPUT_BACKENDS}, '
                f'got {self.input_backend!r}'
            )
        if self.frame_id not in SUPPORTED_COMMAND_FRAMES:
            raise ValueError(
                f'frame_id must be one of {SUPPORTED_COMMAND_FRAMES}, got {self.frame_id!r}'
            )

        if not is_motion_confirmation_valid(
            require_confirmation=self.require_confirmation,
            human_confirmation=str(self.human_confirmation),
            required_confirmation_text=str(self.required_confirmation_text),
        ):
            self.get_logger().error(
                'Motion confirmation rejected. Refusing to start keyboard Servo control.'
            )
            raise RuntimeError('motion confirmation rejected')

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
        self._evdev_reader = evdev_reader or EvdevKeyReader(self.input_device)
        self._pressed_key_state = PressedKeyState()
        self._joy_mapper = JoyMapper(deadzone=self.joy_deadzone)
        self._latest_joy_control = JoyControl()
        self._last_joy_msg_time = 0.0
        self._joy_subscription = None
        if self.input_backend == 'joy':
            self._joy_subscription = self.create_subscription(
                Joy,
                self.joy_topic,
                self._on_joy_message,
                10,
            )
        self._smooth_velocity = SmoothVelocityController(
            linear_speed_mps=self.linear_speed_mps,
            acceleration_mps2=self.acceleration_mps2,
            deceleration_mps2=self.deceleration_mps2,
        )
        self._quit_requested = False
        self._session_start_time = time.monotonic()
        self._last_timer_time = self._session_start_time
        self._last_service_wait_log_time = 0.0

        period_sec = 1.0 / self.publish_rate_hz
        self._timer = self.create_timer(period_sec, self._on_timer)

        self.get_logger().info(
            'Keyboard Servo node started. '
            f'command_topic={self.command_topic} '
            f'command_type_service={self.command_type_service} '
            f'frame_id={self.frame_id} '
            f'input_backend={self.input_backend} '
            f'joy_topic={self.joy_topic} '
            f'rate={self.publish_rate_hz:.1f}Hz '
            f'speed={self.linear_speed_mps:.4f}m/s '
            f'timeout={self.key_timeout_sec:.2f}s '
            f'max_session={self.max_session_duration_sec:.1f}s '
            f'require_confirmation={self.require_confirmation}'
        )

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def close(self) -> None:
        self._key_reader.close()
        self._evdev_reader.close()

    def publish_stop(self) -> None:
        self._limiter.stop()
        self._pressed_key_state.clear()
        self._publish_twist(self._smooth_velocity.stop_immediately())

    def _on_timer(self) -> None:
        if not self._ensure_twist_mode_ready():
            return

        now_sec = time.monotonic()
        dt_sec = now_sec - self._last_timer_time
        self._last_timer_time = now_sec
        if self._session_expired(now_sec):
            self.get_logger().warn(
                'Maximum keyboard Servo session duration reached. '
                'Publishing stop command before shutdown.'
            )
            self.publish_stop()
            rclpy.shutdown()
            return

        if self.input_backend == 'evdev':
            self._on_evdev_timer(dt_sec)
            return
        if self.input_backend == 'joy':
            self._on_joy_timer(dt_sec, now_sec)
            return

        self._on_terminal_timer(now_sec)

    def _on_terminal_timer(self, now_sec: float) -> None:
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

    def _on_evdev_timer(self, dt_sec: float) -> None:
        emergency_stop = False
        quit_requested = False

        for event in self._evdev_reader.read_events():
            decision = self._pressed_key_state.apply(event)
            emergency_stop = emergency_stop or decision.emergency_stop
            quit_requested = quit_requested or decision.quit_requested
            if event.value != KeyEventValue.REPEAT:
                self.get_logger().info(
                    f'Evdev key event: key={event.key_name} value={event.value.name.lower()}'
                )

        if emergency_stop:
            self._publish_twist(self._smooth_velocity.stop_immediately())
        else:
            target_x, target_y = self._pressed_key_state.target_axes()
            self._publish_twist(
                self._smooth_velocity.update(target_x, target_y, dt_sec)
            )

        if quit_requested:
            self._quit_requested = True
            self.get_logger().info('Quit requested. Publishing stop command before shutdown.')
            rclpy.shutdown()

    def _on_joy_message(self, msg: Joy) -> None:
        self._latest_joy_control = self._joy_mapper.map(msg.axes, msg.buttons)
        self._last_joy_msg_time = time.monotonic()

    def _on_joy_timer(self, dt_sec: float, now_sec: float) -> None:
        control = self._latest_joy_control
        if self._last_joy_msg_time == 0.0:
            self._publish_twist(self._smooth_velocity.update(0.0, 0.0, dt_sec))
            return

        if now_sec - self._last_joy_msg_time > self.key_timeout_sec:
            control = JoyControl()

        if control.emergency_stop:
            self._publish_twist(self._smooth_velocity.stop_immediately())
        else:
            self._publish_twist(
                self._smooth_velocity.update(control.target_x, control.target_y, dt_sec)
            )

        if control.quit_requested:
            self._quit_requested = True
            self.get_logger().info('Joy quit requested. Publishing stop command before shutdown.')
            rclpy.shutdown()

    def _ensure_twist_mode_ready(self) -> bool:
        if self._twist_mode_ready:
            return True

        if self._command_type_future is None:
            if not self._command_type_client.service_is_ready():
                now_sec = time.monotonic()
                if now_sec - self._last_service_wait_log_time >= 1.0:
                    self.get_logger().info(
                        'Waiting for Servo command type service: '
                        f'{self.command_type_service}'
                    )
                    self._last_service_wait_log_time = now_sec
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

    def _session_expired(self, now_sec: float) -> bool:
        if self.max_session_duration_sec <= 0.0:
            return False
        return now_sec - self._session_start_time >= self.max_session_duration_sec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = KeyboardServoNode()
        if node.input_backend == 'evdev':
            node._evdev_reader.start()
            node.get_logger().info(
                f'Evdev keyboard input attached to {node._evdev_reader.source_name}.'
            )
        elif node.input_backend == 'terminal':
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
        else:
            node.get_logger().info(
                f'Joy input enabled. Waiting for sensor_msgs/Joy on {node.joy_topic}.'
            )
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('Keyboard interrupt received. Publishing stop command.')
    except (RuntimeError, ValueError) as exc:
        print(f'Keyboard Servo node refused to start: {exc}')
    finally:
        if node is not None and rclpy.ok():
            node.publish_stop()
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
