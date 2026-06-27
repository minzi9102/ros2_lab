import math
import time
from dataclasses import dataclass
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .joy_mapping import JoyControl, JoyMapper
from .pen_math import (
    PaperBounds,
    PlanarVelocity,
    planar_turn_speed_scale,
    SmoothPlanarVelocity,
    VirtualPenState,
)
from .pose_math import ContinuousPenOrientation, rotate_vector


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


class PenWritingVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("pen_writing_visualizer")

        self.world_frame = str(self.declare_parameter("world_frame", "base_link").value)
        self.paper_frame = str(self.declare_parameter("paper_frame", "paper_frame").value)
        self.marker_topic = str(
            self.declare_parameter("marker_topic", "/pen_writing/markers").value
        )
        self.joy_topic = str(self.declare_parameter("joy_topic", "/joy").value)
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 60.0).value
        )
        self.joy_deadzone = float(self.declare_parameter("joy_deadzone", 0.08).value)
        self.joy_timeout_sec = float(
            self.declare_parameter("joy_timeout_sec", 0.25).value
        )
        self.max_planar_speed_mps = float(
            self.declare_parameter("max_planar_speed_mps", 0.025).value
        )
        self.turn_speed_scale_active_speed_mps = float(
            self.declare_parameter("turn_speed_scale_active_speed_mps", 0.01).value
        )
        self.turn_speed_scale_soft_deg = float(
            self.declare_parameter("turn_speed_scale_soft_deg", 45.0).value
        )
        self.turn_speed_scale_hard_deg = float(
            self.declare_parameter("turn_speed_scale_hard_deg", 135.0).value
        )
        self.turn_speed_scale_min = float(
            self.declare_parameter("turn_speed_scale_min", 0.35).value
        )
        self.acceleration_mps2 = float(
            self.declare_parameter("acceleration_mps2", 0.08).value
        )
        self.deceleration_mps2 = float(
            self.declare_parameter("deceleration_mps2", 0.16).value
        )
        self.yaw_hold_speed_mps = float(
            self.declare_parameter("yaw_hold_speed_mps", 0.005).value
        )
        self.tilt_activate_speed_mps = float(
            self.declare_parameter("tilt_activate_speed_mps", 0.01).value
        )
        self.tilt_rate_degps = float(
            self.declare_parameter("tilt_rate_degps", 8.0).value
        )
        self.untilt_rate_degps = float(
            self.declare_parameter("untilt_rate_degps", 10.0).value
        )
        self.max_pen_axis_angular_speed_degps = float(
            self.declare_parameter(
                "max_pen_axis_angular_speed_degps",
                10.0,
            ).value
        )
        self.pen_length_m = float(self.declare_parameter("pen_length_m", 0.14).value)
        self.pen_radius_m = float(self.declare_parameter("pen_radius_m", 0.006).value)
        self.pen_tip_radius_m = float(
            self.declare_parameter("pen_tip_radius_m", 0.01).value
        )
        self.target_pen_tip_axis_length_m = float(
            self.declare_parameter("target_pen_tip_axis_length_m", 0.08).value
        )
        self.fixed_tilt_deg = float(self.declare_parameter("fixed_tilt_deg", 20.0).value)
        self.paper_width_m = float(self.declare_parameter("paper_width_m", 0.24).value)
        self.paper_height_m = float(self.declare_parameter("paper_height_m", 0.16).value)
        self.paper_origin_xyz = self._declare_float_list(
            "paper_origin_xyz",
            [0.45, 0.0, 0.12],
            expected_size=3,
        )
        initial_tip_xy = self._declare_float_list(
            "initial_tip_xy",
            [0.0, 0.0],
            expected_size=2,
        )

        self._validate_parameters()

        self._marker_publisher = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._joy_subscription = self.create_subscription(
            Joy,
            self.joy_topic,
            self._on_joy_message,
            10,
        )
        self._joy_mapper = JoyMapper(deadzone=self.joy_deadzone)
        self._latest_joy_control = JoyControl()
        self._last_joy_msg_time = 0.0
        self._velocity = SmoothPlanarVelocity(
            max_speed_mps=self.max_planar_speed_mps,
            acceleration_mps2=self.acceleration_mps2,
            deceleration_mps2=self.deceleration_mps2,
        )
        self._pen_state = VirtualPenState(
            initial_tip_x=initial_tip_xy[0],
            initial_tip_y=initial_tip_xy[1],
            initial_yaw=math.pi,
            paper_bounds=PaperBounds(
                width=self.paper_width_m,
                height=self.paper_height_m,
            ),
            yaw_hold_speed_mps=self.yaw_hold_speed_mps,
            target_tilt_rad=math.radians(self.fixed_tilt_deg),
            tilt_activate_speed_mps=self.tilt_activate_speed_mps,
            tilt_rate_radps=math.radians(self.tilt_rate_degps),
            untilt_rate_radps=math.radians(self.untilt_rate_degps),
        )
        self._pen_orientation = ContinuousPenOrientation(
            initial_pen_pose=self._pen_state.pose,
            pen_length=self.pen_length_m,
            max_axis_angular_speed_radps=math.radians(
                self.max_pen_axis_angular_speed_degps
            ),
        )
        self._last_timer_time = time.monotonic()
        self._quit_requested = False

        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            "Pen writing visualizer started. "
            f"world_frame={self.world_frame} paper_frame={self.paper_frame} "
            f"marker_topic={self.marker_topic} joy_topic={self.joy_topic} "
            f"rate={self.publish_rate_hz:.1f}Hz speed={self.max_planar_speed_mps:.3f}m/s "
            "mode=visualization_only"
        )

    def _declare_float_list(
        self,
        name: str,
        default_value: list[float],
        *,
        expected_size: int,
    ) -> list[float]:
        value = self.declare_parameter(name, default_value).value
        result = [float(item) for item in value]
        if len(result) != expected_size:
            raise ValueError(f"{name} must contain {expected_size} values")
        return result

    def _validate_parameters(self) -> None:
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.joy_timeout_sec <= 0.0:
            raise ValueError("joy_timeout_sec must be greater than zero")
        if self.pen_length_m <= 0.0:
            raise ValueError("pen_length_m must be greater than zero")
        if self.pen_radius_m <= 0.0:
            raise ValueError("pen_radius_m must be greater than zero")
        if self.pen_tip_radius_m <= 0.0:
            raise ValueError("pen_tip_radius_m must be greater than zero")
        if self.paper_width_m <= 0.0 or self.paper_height_m <= 0.0:
            raise ValueError("paper dimensions must be greater than zero")
        if self.fixed_tilt_deg < 0.0 or self.fixed_tilt_deg >= 90.0:
            raise ValueError("fixed_tilt_deg must be in [0, 90)")
        if self.tilt_activate_speed_mps < 0.0:
            raise ValueError("tilt_activate_speed_mps must be non-negative")
        if self.tilt_rate_degps <= 0.0:
            raise ValueError("tilt_rate_degps must be greater than zero")
        if self.untilt_rate_degps <= 0.0:
            raise ValueError("untilt_rate_degps must be greater than zero")
        if self.max_pen_axis_angular_speed_degps <= 0.0:
            raise ValueError(
                "max_pen_axis_angular_speed_degps must be greater than zero"
            )
        planar_turn_speed_scale(
            current_velocity=PlanarVelocity(),
            target_x=0.0,
            target_y=0.0,
            active_speed_mps=self.turn_speed_scale_active_speed_mps,
            soft_turn_deg=self.turn_speed_scale_soft_deg,
            hard_turn_deg=self.turn_speed_scale_hard_deg,
            min_scale=self.turn_speed_scale_min,
        )
        if self.target_pen_tip_axis_length_m <= 0.0:
            raise ValueError(
                "target_pen_tip_axis_length_m must be greater than zero"
            )

    def _on_joy_message(self, msg: Joy) -> None:
        self._latest_joy_control = self._joy_mapper.map(msg.axes, msg.buttons)
        self._last_joy_msg_time = time.monotonic()

    def _on_timer(self) -> None:
        now_sec = time.monotonic()
        dt_sec = now_sec - self._last_timer_time
        self._last_timer_time = now_sec

        control = self._current_control(now_sec)
        if control.emergency_stop:
            velocity = self._velocity.stop_immediately()
        else:
            turn_scale = planar_turn_speed_scale(
                current_velocity=self._velocity.current(),
                target_x=control.target_x,
                target_y=control.target_y,
                active_speed_mps=self.turn_speed_scale_active_speed_mps,
                soft_turn_deg=self.turn_speed_scale_soft_deg,
                hard_turn_deg=self.turn_speed_scale_hard_deg,
                min_scale=self.turn_speed_scale_min,
            )
            velocity = self._velocity.update(
                control.target_x * turn_scale,
                control.target_y * turn_scale,
                dt_sec,
            )

        pose = self._pen_state.update(velocity, dt_sec)
        self._pen_orientation.update(pose, dt_sec)
        self._publish_tf_and_markers(velocity)

        if control.quit_requested:
            self._quit_requested = True
            self.get_logger().info("Joy quit requested. Visualization node shutting down.")
            rclpy.shutdown()
            return

        if control.emergency_stop:
            self.get_logger().debug(
                f"Emergency stop: tip=({pose.tip_x:.3f}, {pose.tip_y:.3f})"
            )

    def _current_control(self, now_sec: float) -> JoyControl:
        if self._last_joy_msg_time == 0.0:
            return JoyControl()
        if now_sec - self._last_joy_msg_time > self.joy_timeout_sec:
            return JoyControl()
        return self._latest_joy_control

    def _publish_tf_and_markers(self, velocity: PlanarVelocity) -> None:
        stamp = self.get_clock().now().to_msg()
        self._tf_broadcaster.sendTransform(
            [
                self._paper_transform(stamp),
                self._pen_tip_transform(stamp),
            ]
        )
        self._marker_publisher.publish(self._make_marker_array(velocity))

    def _paper_transform(self, stamp) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = self.paper_frame
        transform.transform.translation.x = self.paper_origin_xyz[0]
        transform.transform.translation.y = self.paper_origin_xyz[1]
        transform.transform.translation.z = self.paper_origin_xyz[2]
        transform.transform.rotation.w = 1.0
        return transform

    def _pen_tip_transform(self, stamp) -> TransformStamped:
        pose = self._pen_state.pose
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.paper_frame
        transform.child_frame_id = "pen_tip"
        transform.transform.translation.x = pose.tip_x
        transform.transform.translation.y = pose.tip_y
        transform.transform.translation.z = 0.0
        orientation = self._pen_orientation.orientation
        transform.transform.rotation.x = orientation.x
        transform.transform.rotation.y = orientation.y
        transform.transform.rotation.z = orientation.z
        transform.transform.rotation.w = orientation.w
        return transform

    def _make_marker_array(self, velocity: PlanarVelocity) -> MarkerArray:
        pose = self._pen_state.pose
        z_axis = rotate_vector(
            self._pen_orientation.orientation,
            (0.0, 0.0, 1.0),
        )
        tip = Point3(pose.tip_x, pose.tip_y, 0.0)
        tail = Point3(
            pose.tip_x - z_axis[0] * self.pen_length_m,
            pose.tip_y - z_axis[1] * self.pen_length_m,
            -z_axis[2] * self.pen_length_m,
        )

        markers = [
            self._paper_marker(marker_id=0),
            self._paper_bounds_marker(marker_id=1),
            self._tip_marker(marker_id=2, tip=tip),
            self._axis_marker(marker_id=3, tip=tip, tail=tail),
            self._motion_marker(marker_id=4, tip=tip, velocity=velocity),
            self._tail_marker(marker_id=5, tail=tail),
            self._pose_axis_marker(
                marker_id=6,
                tip=tip,
                local_axis=(1.0, 0.0, 0.0),
                namespace="target_pen_tip_x_axis",
                color=ColorRGBA(r=0.95, g=0.10, b=0.10, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=7,
                tip=tip,
                local_axis=(0.0, 1.0, 0.0),
                namespace="target_pen_tip_y_axis",
                color=ColorRGBA(r=0.10, g=0.85, b=0.20, a=1.0),
            ),
            self._pose_axis_marker(
                marker_id=8,
                tip=tip,
                local_axis=(0.0, 0.0, 1.0),
                namespace="target_pen_tip_z_axis",
                color=ColorRGBA(r=0.10, g=0.35, b=1.0, a=1.0),
            ),
        ]
        return MarkerArray(markers=markers)

    def _base_marker(self, marker_id: int, marker_type: int, namespace: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.paper_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.frame_locked = False
        return marker

    def _paper_marker(self, marker_id: int) -> Marker:
        marker = self._base_marker(marker_id, Marker.CUBE, "paper")
        marker.pose.position.z = -0.001
        marker.scale.x = self.paper_width_m
        marker.scale.y = self.paper_height_m
        marker.scale.z = 0.002
        marker.color = ColorRGBA(r=0.92, g=0.92, b=0.86, a=0.70)
        return marker

    def _paper_bounds_marker(self, marker_id: int) -> Marker:
        marker = self._base_marker(marker_id, Marker.LINE_STRIP, "paper_bounds")
        marker.scale.x = 0.003
        marker.color = ColorRGBA(r=0.12, g=0.12, b=0.12, a=1.0)
        half_w = self.paper_width_m / 2.0
        half_h = self.paper_height_m / 2.0
        marker.points = self._points(
            [
                (-half_w, -half_h, 0.002),
                (half_w, -half_h, 0.002),
                (half_w, half_h, 0.002),
                (-half_w, half_h, 0.002),
                (-half_w, -half_h, 0.002),
            ]
        )
        return marker

    def _tip_marker(self, marker_id: int, tip: Point3) -> Marker:
        marker = self._base_marker(marker_id, Marker.SPHERE, "pen_tip")
        marker.pose.position.x = tip.x
        marker.pose.position.y = tip.y
        marker.pose.position.z = tip.z + self.pen_tip_radius_m
        marker.scale.x = self.pen_tip_radius_m * 2.0
        marker.scale.y = self.pen_tip_radius_m * 2.0
        marker.scale.z = self.pen_tip_radius_m * 2.0
        marker.color = ColorRGBA(r=0.05, g=0.25, b=0.95, a=1.0)
        return marker

    def _tail_marker(self, marker_id: int, tail: Point3) -> Marker:
        marker = self._base_marker(marker_id, Marker.SPHERE, "pen_tail")
        marker.pose.position.x = tail.x
        marker.pose.position.y = tail.y
        marker.pose.position.z = tail.z
        marker.scale.x = self.pen_radius_m * 2.2
        marker.scale.y = self.pen_radius_m * 2.2
        marker.scale.z = self.pen_radius_m * 2.2
        marker.color = ColorRGBA(r=0.95, g=0.65, b=0.10, a=1.0)
        return marker

    def _axis_marker(self, marker_id: int, tip: Point3, tail: Point3) -> Marker:
        marker = self._base_marker(marker_id, Marker.ARROW, "pen_axis")
        marker.points = self._points(
            [
                (tail.x, tail.y, tail.z),
                (tip.x, tip.y, tip.z),
            ]
        )
        marker.scale.x = self.pen_radius_m
        marker.scale.y = self.pen_radius_m * 2.4
        marker.scale.z = self.pen_radius_m * 2.4
        marker.color = ColorRGBA(r=0.03, g=0.58, b=0.34, a=1.0)
        return marker

    def _motion_marker(
        self,
        marker_id: int,
        tip: Point3,
        velocity: PlanarVelocity,
    ) -> Marker:
        marker = self._base_marker(marker_id, Marker.ARROW, "motion_direction")
        speed = math.hypot(velocity.x, velocity.y)
        if speed < 1e-9:
            marker.action = Marker.DELETE
            return marker

        scale = min(0.08, 0.04 + speed)
        marker.points = self._points(
            [
                (tip.x, tip.y, 0.02),
                (
                    tip.x + velocity.x / speed * scale,
                    tip.y + velocity.y / speed * scale,
                    0.02,
                ),
            ]
        )
        marker.scale.x = 0.004
        marker.scale.y = 0.012
        marker.scale.z = 0.012
        marker.color = ColorRGBA(r=0.85, g=0.10, b=0.18, a=1.0)
        return marker

    def _pose_axis_marker(
        self,
        *,
        marker_id: int,
        tip: Point3,
        local_axis: tuple[float, float, float],
        namespace: str,
        color: ColorRGBA,
    ) -> Marker:
        axis = rotate_vector(self._pen_orientation.orientation, local_axis)
        marker = self._base_marker(marker_id, Marker.ARROW, namespace)
        marker.points = self._points(
            [
                (tip.x, tip.y, tip.z),
                (
                    tip.x + axis[0] * self.target_pen_tip_axis_length_m,
                    tip.y + axis[1] * self.target_pen_tip_axis_length_m,
                    tip.z + axis[2] * self.target_pen_tip_axis_length_m,
                ),
            ]
        )
        marker.scale.x = 0.004
        marker.scale.y = 0.010
        marker.scale.z = 0.010
        marker.color = color
        return marker

    @staticmethod
    def _points(points: Iterable[tuple[float, float, float]]) -> list[Point]:
        result = []
        for x, y, z in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            result.append(point)
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PenWritingVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("Keyboard interrupt received.")
    except (RuntimeError, ValueError) as exc:
        print(f"Pen writing visualizer refused to start: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
