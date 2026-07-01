import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (
    AccelStamped,
    Point,
    PoseStamped,
    TransformStamped,
    TwistStamped,
)
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .joy_mapping import JoyIntent, JoyMapper
from .kinematics import (
    Quaternion,
    VirtualPenConfig,
    VirtualPenKinematicState,
    VirtualPenKinematics,
    Vector3,
)


class VirtualPenNode(Node):
    def __init__(self) -> None:
        super().__init__("virtual_pen_node")
        self.world_frame = str(self.declare_parameter("world_frame", "base_link").value)
        self.paper_frame = str(self.declare_parameter("paper_frame", "paper_frame").value)
        self.joy_topic = str(self.declare_parameter("joy_topic", "/joy").value)
        self.update_rate_hz = float(
            self.declare_parameter("update_rate_hz", 125.0).value
        )
        self.joy_deadzone = float(self.declare_parameter("joy_deadzone", 0.08).value)
        self.joy_timeout_sec = float(
            self.declare_parameter("joy_timeout_sec", 0.25).value
        )
        if self.update_rate_hz <= 0.0:
            raise ValueError("update_rate_hz must be positive")
        if self.joy_timeout_sec <= 0.0:
            raise ValueError("joy_timeout_sec must be positive")

        config = VirtualPenConfig(
            max_speed_mps=self._float("max_speed_mps", 0.03),
            max_accel_mps2=self._float("max_accel_mps2", 0.08),
            max_decel_mps2=self._float("max_decel_mps2", 0.16),
            max_jerk_mps3=self._float("max_jerk_mps3", 0.80),
            confidence_speed_low_mps=self._float(
                "confidence_speed_low_mps", 0.003
            ),
            confidence_speed_high_mps=self._float(
                "confidence_speed_high_mps", 0.015
            ),
            max_yaw_rate_radps=math.radians(
                self._float("max_yaw_rate_degps", 30.0)
            ),
            max_yaw_accel_radps2=math.radians(
                self._float("max_yaw_accel_degps2", 120.0)
            ),
            max_tilt_rad=math.radians(self._float("max_tilt_deg", 20.0)),
            tilt_speed_low_mps=self._float("tilt_speed_low_mps", 0.003),
            tilt_speed_high_mps=self._float("tilt_speed_high_mps", 0.020),
            max_tilt_rate_radps=math.radians(
                self._float("max_tilt_rate_degps", 12.0)
            ),
            max_untilt_rate_radps=math.radians(
                self._float("max_untilt_rate_degps", 12.0)
            ),
            max_tilt_accel_radps2=math.radians(
                self._float("max_tilt_accel_degps2", 80.0)
            ),
            max_axis_angular_speed_radps=math.radians(
                self._float("max_axis_angular_speed_degps", 12.0)
            ),
            max_axis_angular_accel_radps2=math.radians(
                self._float("max_axis_angular_accel_degps2", 80.0)
            ),
            hold_time_sec=self._float("hold_time_sec", 0.30),
            paper_width_m=self._float("paper_width_m", 0.24),
            paper_height_m=self._float("paper_height_m", 0.16),
            paper_origin_world=self._float_list(
                "paper_origin_xyz", [0.45, 0.0, 0.12], 3
            ),
            tool0_to_pen_tip=self._float_list(
                "tool0_to_pen_tip_xyz", [0.0, 0.0, 0.14], 3
            ),
            initial_tip_xy=self._float_list("initial_tip_xy", [0.0, 0.0], 2),
            initial_yaw_rad=math.radians(
                self._float("initial_yaw_deg", 180.0)
            ),
        )
        self._pen = VirtualPenKinematics(config)
        self._joy_mapper = JoyMapper(self.joy_deadzone)
        self._intent = JoyIntent()
        self._last_joy_time = 0.0
        self._last_timer_time = time.monotonic()
        self._quit_requested = False

        self._tip_pose_pub = self.create_publisher(
            PoseStamped, "/pen_writing/virtual_pen/tip_pose", 10
        )
        self._tip_twist_pub = self.create_publisher(
            TwistStamped, "/pen_writing/virtual_pen/tip_twist", 10
        )
        self._tip_accel_pub = self.create_publisher(
            AccelStamped, "/pen_writing/virtual_pen/tip_accel", 10
        )
        self._tool_pose_pub = self.create_publisher(
            PoseStamped, "/pen_writing/virtual_pen/tool0_pose", 10
        )
        self._tool_twist_pub = self.create_publisher(
            TwistStamped, "/pen_writing/virtual_pen/tool0_twist", 10
        )
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/pen_writing/virtual_pen/diagnostics", 10
        )
        self._markers_pub = self.create_publisher(
            MarkerArray, "/pen_writing/virtual_pen/markers", 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._joy_subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self._on_joy,
            10,
        )
        self._timer = self.create_timer(1.0 / self.update_rate_hz, self._on_timer)
        self.get_logger().info(
            f"Virtual pen started: frame={self.world_frame}, "
            f"rate={self.update_rate_hz:.1f}Hz, max_speed={config.max_speed_mps:.3f}m/s"
        )

    def _float(self, name: str, default: float) -> float:
        return float(self.declare_parameter(name, default).value)

    def _float_list(
        self,
        name: str,
        default: list[float],
        size: int,
    ) -> tuple[float, ...]:
        values = tuple(float(value) for value in self.declare_parameter(name, default).value)
        if len(values) != size:
            raise ValueError(f"{name} must contain {size} values")
        return values

    def _on_joy(self, message: Joy) -> None:
        self._intent = self._joy_mapper.map(message.axes, message.buttons)
        self._last_joy_time = time.monotonic()
        if self._intent.emergency_stop:
            self._pen.emergency_stop()
        if self._intent.quit_requested:
            self._quit_requested = True

    def _on_timer(self) -> None:
        now = time.monotonic()
        nominal_dt = 1.0 / self.update_rate_hz
        dt_sec = min(max(now - self._last_timer_time, 1e-9), 2.0 * nominal_dt)
        self._last_timer_time = now
        intent = self._current_intent(now)
        state = (
            self._pen.state
            if intent.emergency_stop
            else self._pen.update(dt_sec, intent.x, intent.y)
        )
        self._publish(state)
        if self._quit_requested:
            self.get_logger().info("Joy quit requested")
            rclpy.shutdown()

    def _current_intent(self, now: float) -> JoyIntent:
        if self._last_joy_time == 0.0:
            return JoyIntent()
        if now - self._last_joy_time > self.joy_timeout_sec:
            return JoyIntent()
        return self._intent

    def _publish(self, state: VirtualPenKinematicState) -> None:
        stamp = self.get_clock().now().to_msg()
        self._tip_pose_pub.publish(
            self._pose_message(stamp, state.tip_position_world, state.orientation_world)
        )
        self._tip_twist_pub.publish(
            self._twist_message(
                stamp,
                state.tip_velocity_world,
                state.angular_velocity_world,
            )
        )
        self._tip_accel_pub.publish(
            self._accel_message(
                stamp,
                state.tip_acceleration_world,
                state.angular_acceleration_world,
            )
        )
        self._tool_pose_pub.publish(
            self._pose_message(
                stamp,
                state.tool0_position_world,
                state.tool0_orientation_world,
            )
        )
        self._tool_twist_pub.publish(
            self._twist_message(
                stamp,
                state.tool0_linear_velocity_world,
                state.tool0_angular_velocity_world,
            )
        )
        self._diagnostics_pub.publish(self._diagnostics(stamp, state))
        self._markers_pub.publish(self._markers(stamp, state))
        self._tf_broadcaster.sendTransform(
            [
                self._transform(
                    stamp,
                    self.world_frame,
                    self.paper_frame,
                    self._pen.config.paper_origin_world,
                    Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
                self._transform(
                    stamp,
                    self.world_frame,
                    "pen_tip",
                    state.tip_position_world,
                    state.orientation_world,
                ),
                self._transform(
                    stamp,
                    self.world_frame,
                    "tool0_target",
                    state.tool0_position_world,
                    state.tool0_orientation_world,
                ),
            ]
        )

    def _pose_message(
        self,
        stamp,
        position: Vector3,
        orientation: Quaternion,
    ) -> PoseStamped:
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        _set_vector(message.pose.position, position)
        _set_quaternion(message.pose.orientation, orientation)
        return message

    def _twist_message(
        self,
        stamp,
        linear: Vector3,
        angular: Vector3,
    ) -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        _set_vector(message.twist.linear, linear)
        _set_vector(message.twist.angular, angular)
        return message

    def _accel_message(
        self,
        stamp,
        linear: Vector3,
        angular: Vector3,
    ) -> AccelStamped:
        message = AccelStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.world_frame
        _set_vector(message.accel.linear, linear)
        _set_vector(message.accel.angular, angular)
        return message

    def _diagnostics(
        self,
        stamp,
        state: VirtualPenKinematicState,
    ) -> DiagnosticArray:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = "virtual_pen_kinematics"
        status.message = state.motion_phase
        status.hardware_id = "virtual_pen"
        status.values = [
            KeyValue(key="motion_phase", value=state.motion_phase),
            KeyValue(key="planar_speed_mps", value=f"{state.planar_speed_mps:.9f}"),
            KeyValue(
                key="direction_confidence",
                value=f"{state.direction_confidence:.9f}",
            ),
            KeyValue(key="yaw_rad", value=f"{state.yaw_rad:.9f}"),
            KeyValue(key="tilt_rad", value=f"{state.tilt_rad:.9f}"),
            KeyValue(
                key="axis_angular_speed_radps",
                value=f"{state.axis_angular_speed_radps:.9f}",
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = stamp
        message.status = [status]
        return message

    def _markers(self, stamp, state: VirtualPenKinematicState) -> MarkerArray:
        origin = self._pen.config.paper_origin_world
        paper = self._marker(stamp, 0, Marker.CUBE, "paper")
        _set_vector(paper.pose.position, (origin[0], origin[1], origin[2] - 0.001))
        paper.scale.x = self._pen.config.paper_width_m
        paper.scale.y = self._pen.config.paper_height_m
        paper.scale.z = 0.002
        paper.color = ColorRGBA(r=0.92, g=0.92, b=0.88, a=0.85)

        tip = self._marker(stamp, 1, Marker.SPHERE, "pen_tip")
        _set_vector(tip.pose.position, state.tip_position_world)
        tip.scale.x = tip.scale.y = tip.scale.z = 0.016
        tip.color = ColorRGBA(r=0.95, g=0.20, b=0.10, a=1.0)

        shaft = self._marker(stamp, 2, Marker.ARROW, "pen_shaft")
        shaft.points = [
            _point(state.tool0_position_world),
            _point(state.tip_position_world),
        ]
        shaft.scale.x = 0.009
        shaft.scale.y = 0.014
        shaft.scale.z = 0.018
        shaft.color = ColorRGBA(r=0.10, g=0.35, b=0.95, a=1.0)

        velocity = self._marker(stamp, 3, Marker.ARROW, "tip_velocity")
        velocity.points = [
            _point(state.tip_position_world),
            _point(
                (
                    state.tip_position_world[0] + state.tip_velocity_world[0],
                    state.tip_position_world[1] + state.tip_velocity_world[1],
                    state.tip_position_world[2] + state.tip_velocity_world[2],
                )
            ),
        ]
        velocity.scale.x = 0.004
        velocity.scale.y = 0.008
        velocity.scale.z = 0.012
        velocity.color = ColorRGBA(r=0.10, g=0.85, b=0.20, a=1.0)

        bounds = self._marker(stamp, 4, Marker.LINE_STRIP, "paper_bounds")
        half_width = self._pen.config.paper_width_m / 2.0
        half_height = self._pen.config.paper_height_m / 2.0
        bounds.points = [
            _point((origin[0] - half_width, origin[1] - half_height, origin[2])),
            _point((origin[0] + half_width, origin[1] - half_height, origin[2])),
            _point((origin[0] + half_width, origin[1] + half_height, origin[2])),
            _point((origin[0] - half_width, origin[1] + half_height, origin[2])),
            _point((origin[0] - half_width, origin[1] - half_height, origin[2])),
        ]
        bounds.scale.x = 0.002
        bounds.color = ColorRGBA(r=0.15, g=0.15, b=0.15, a=1.0)
        return MarkerArray(markers=[paper, tip, shaft, velocity, bounds])

    def _marker(self, stamp, marker_id: int, marker_type: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _transform(
        self,
        stamp,
        parent: str,
        child: str,
        position: Vector3,
        orientation: Quaternion,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        _set_vector(transform.transform.translation, position)
        _set_quaternion(transform.transform.rotation, orientation)
        return transform


def _set_vector(destination, source: Vector3) -> None:
    destination.x, destination.y, destination.z = source


def _set_quaternion(destination, source: Quaternion) -> None:
    destination.x = source.x
    destination.y = source.y
    destination.z = source.z
    destination.w = source.w


def _point(vector: Vector3) -> Point:
    point = Point()
    _set_vector(point, vector)
    return point


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VirtualPenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
