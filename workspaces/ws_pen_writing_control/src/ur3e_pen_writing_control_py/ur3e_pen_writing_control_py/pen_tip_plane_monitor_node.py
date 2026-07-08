import math
from typing import Iterable

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from .pose_math import Point3, PoseTarget, Quaternion, transform_point


def float_list(value: Iterable[float], *, expected_size: int, name: str) -> list[float]:
    result = [float(item) for item in value]
    if len(result) != expected_size:
        raise ValueError(f"{name} must contain {expected_size} values")
    return result


def normalize_vector(vector: Point3) -> Point3:
    norm = math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
    if norm <= 0.0:
        raise ValueError("paper_normal_xyz must not be zero")
    return Point3(x=vector.x / norm, y=vector.y / norm, z=vector.z / norm)


def signed_distance_to_plane(*, point: Point3, center: Point3, normal: Point3) -> float:
    unit_normal = normalize_vector(normal)
    return (
        unit_normal.x * (point.x - center.x)
        + unit_normal.y * (point.y - center.y)
        + unit_normal.z * (point.z - center.z)
    )


def actual_pen_tip_from_tool_pose(
    *, tool_pose: PoseTarget, tool0_to_pen_tip: Point3
) -> Point3:
    return transform_point(tool_pose, tool0_to_pen_tip)


def pose_target_from_transform(transform) -> PoseTarget:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return PoseTarget(
        position=Point3(x=translation.x, y=translation.y, z=translation.z),
        orientation=Quaternion(
            x=rotation.x,
            y=rotation.y,
            z=rotation.z,
            w=rotation.w,
        ),
    )


class PenTipPlaneMonitor(Node):
    def __init__(self):
        super().__init__("pen_tip_plane_monitor")
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.tool_frame = str(self.declare_parameter("tool_frame", "tool0").value)
        tool0_to_pen_tip_xyz = float_list(
            self.declare_parameter("tool0_to_pen_tip_xyz", [0.0, 0.0, 0.14]).value,
            expected_size=3,
            name="tool0_to_pen_tip_xyz",
        )
        paper_center_xyz = float_list(
            self.declare_parameter("paper_center_xyz", [0.45, 0.0, 0.12]).value,
            expected_size=3,
            name="paper_center_xyz",
        )
        paper_normal_xyz = float_list(
            self.declare_parameter("paper_normal_xyz", [0.0, 0.0, 1.0]).value,
            expected_size=3,
            name="paper_normal_xyz",
        )
        self.tool0_to_pen_tip = Point3(*tool0_to_pen_tip_xyz)
        self.paper_center = Point3(*paper_center_xyz)
        self.paper_normal = normalize_vector(Point3(*paper_normal_xyz))
        self.warn_below_m = float(self.declare_parameter("warn_below_m", 0.001).value)
        self.error_below_m = float(self.declare_parameter("error_below_m", 0.003).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        if self.warn_below_m < 0.0 or self.error_below_m < 0.0:
            raise ValueError("warn_below_m and error_below_m must be non-negative")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.report)
        self.get_logger().info(
            "Pen tip plane monitor started. "
            f"base_frame={self.base_frame} tool_frame={self.tool_frame} "
            f"tool0_to_pen_tip=({self.tool0_to_pen_tip.x:.6f}, "
            f"{self.tool0_to_pen_tip.y:.6f}, {self.tool0_to_pen_tip.z:.6f}) "
            f"paper_center=({self.paper_center.x:.6f}, {self.paper_center.y:.6f}, "
            f"{self.paper_center.z:.6f}) paper_normal=({self.paper_normal.x:.6f}, "
            f"{self.paper_normal.y:.6f}, {self.paper_normal.z:.6f})"
        )

    def report(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        tool_pose = pose_target_from_transform(transform)
        tip = actual_pen_tip_from_tool_pose(
            tool_pose=tool_pose,
            tool0_to_pen_tip=self.tool0_to_pen_tip,
        )
        signed_distance_m = signed_distance_to_plane(
            point=tip,
            center=self.paper_center,
            normal=self.paper_normal,
        )
        below_paper_m = max(0.0, -signed_distance_m)
        message = (
            f"actual_tip_xyz=({tip.x:.6f}, {tip.y:.6f}, {tip.z:.6f}) "
            f"paper_center_z={self.paper_center.z:.6f} "
            f"signed_distance_mm={signed_distance_m * 1000.0:.3f} "
            f"below_paper_mm={below_paper_m * 1000.0:.3f}"
        )
        if below_paper_m >= self.error_below_m:
            self.get_logger().error(message)
        elif below_paper_m >= self.warn_below_m:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)


def main():
    rclpy.init()
    node = PenTipPlaneMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
