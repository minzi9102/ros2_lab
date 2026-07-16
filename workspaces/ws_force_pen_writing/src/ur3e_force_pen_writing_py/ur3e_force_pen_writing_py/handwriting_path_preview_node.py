import math

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from .handwriting_path import compile_strokes, load_handwriting


MAX_PREVIEW_DIMENSION_M = 0.01


def make_preview_markers(
    strokes,
    *,
    anchor_xyz: tuple[float, float, float],
    writing_width_m: float,
    writing_height_m: float,
    frame_id: str,
    stamp,
    z_offset_m: float,
) -> MarkerArray:
    markers = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    boundary = _marker(Marker.LINE_STRIP, 0, frame_id, stamp)
    boundary.ns = "handwriting_boundary"
    boundary.scale.x = 0.0003
    boundary.color.r = 0.3
    boundary.color.g = 0.3
    boundary.color.b = 0.3
    boundary.color.a = 0.8
    half_width = writing_width_m / 2.0
    half_height = writing_height_m / 2.0
    boundary.points = [
        _point(anchor_xyz, x, y, z_offset_m)
        for x, y in (
            (-half_width, -half_height),
            (half_width, -half_height),
            (half_width, half_height),
            (-half_width, half_height),
            (-half_width, -half_height),
        )
    ]
    markers.markers.append(boundary)

    for index, stroke in enumerate(strokes):
        line = _marker(Marker.LINE_STRIP, 100 + index, frame_id, stamp)
        line.ns = "handwriting_strokes"
        line.scale.x = 0.0005
        line.color.r = 0.1
        line.color.g = 0.4
        line.color.b = 0.9
        line.color.a = 1.0
        line.points = [
            _point(anchor_xyz, x, y, z_offset_m) for x, y in stroke
        ]
        markers.markers.append(line)

        start = _marker(Marker.SPHERE, 200 + index, frame_id, stamp)
        start.ns = "handwriting_starts"
        start.scale.x = start.scale.y = start.scale.z = 0.0015
        start.color.g = 0.8
        start.color.a = 1.0
        start.pose.position = line.points[0]
        start.pose.orientation.w = 1.0
        markers.markers.append(start)

        label = _marker(Marker.TEXT_VIEW_FACING, 300 + index, frame_id, stamp)
        label.ns = "handwriting_order"
        label.scale.z = 0.003
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.pose.position = _point(anchor_xyz, stroke[0][0], stroke[0][1], 0.003)
        label.pose.orientation.w = 1.0
        label.text = str(index + 1)
        markers.markers.append(label)
    return markers


def _marker(marker_type: int, marker_id: int, frame_id: str, stamp) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _point(
    anchor_xyz: tuple[float, float, float],
    offset_x: float,
    offset_y: float,
    offset_z: float,
) -> Point:
    return Point(
        x=anchor_xyz[0] + offset_x,
        y=anchor_xyz[1] + offset_y,
        z=anchor_xyz[2] + offset_z,
    )


class HandwritingPathPreviewNode(Node):
    def __init__(self) -> None:
        super().__init__("handwriting_path_preview")
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        trajectory_file = str(self.declare_parameter("trajectory_file", "").value)
        self.writing_width_m = float(
            self.declare_parameter("writing_width_m", 0.01).value
        )
        self.writing_height_m = float(
            self.declare_parameter("writing_height_m", 0.01).value
        )
        simplify_tolerance_m = float(
            self.declare_parameter("path_simplify_tolerance_m", 0.00025).value
        )
        cartesian_step_m = float(
            self.declare_parameter("cartesian_step_m", 0.0005).value
        )
        self.z_offset_m = float(
            self.declare_parameter("preview_z_offset_m", 0.0005).value
        )
        self.use_anchor_parameter = bool(
            self.declare_parameter("use_anchor_parameter", False).value
        )
        anchor_values = self.declare_parameter(
            "anchor_xyz", [0.0, 0.0, 0.0]
        ).value
        self.anchor_xyz = tuple(float(value) for value in anchor_values)
        detected_point_topic = str(
            self.declare_parameter(
                "detected_point_topic", "/pen_writing/detected_paper_point"
            ).value
        )
        marker_topic = str(
            self.declare_parameter(
                "marker_topic", "/pen_writing/handwriting_preview"
            ).value
        )
        self._validate_parameters(trajectory_file, simplify_tolerance_m, cartesian_step_m)
        self.strokes = compile_strokes(
            load_handwriting(trajectory_file),
            writing_width_m=self.writing_width_m,
            writing_height_m=self.writing_height_m,
            simplify_tolerance_m=simplify_tolerance_m,
            cartesian_step_m=cartesian_step_m,
        )

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, marker_topic, latched)
        if self.use_anchor_parameter:
            self._publish_preview()
        else:
            self.create_subscription(
                PointStamped, detected_point_topic, self._on_detected_point, latched
            )
            self.get_logger().info(
                f"Waiting for paper anchor on {detected_point_topic}; preview is read-only"
            )

    def _validate_parameters(
        self,
        trajectory_file: str,
        simplify_tolerance_m: float,
        cartesian_step_m: float,
    ) -> None:
        if not trajectory_file:
            raise ValueError("trajectory_file must not be empty")
        if not 0.0 < self.writing_width_m <= MAX_PREVIEW_DIMENSION_M:
            raise ValueError("writing_width_m must be in (0, 0.01]")
        if not 0.0 < self.writing_height_m <= MAX_PREVIEW_DIMENSION_M:
            raise ValueError("writing_height_m must be in (0, 0.01]")
        if not 0.0 <= simplify_tolerance_m <= 0.001:
            raise ValueError("path_simplify_tolerance_m must be in [0, 0.001]")
        if not 0.0 < cartesian_step_m <= 0.0005:
            raise ValueError("cartesian_step_m must be in (0, 0.0005]")
        if self.z_offset_m < 0.0:
            raise ValueError("preview_z_offset_m must be non-negative")
        if len(self.anchor_xyz) != 3 or not all(
            math.isfinite(value) for value in self.anchor_xyz
        ):
            raise ValueError("anchor_xyz must contain three finite values")

    def _on_detected_point(self, message: PointStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.base_frame:
            self.get_logger().error(
                f"Ignoring paper anchor in {message.header.frame_id}; expected {self.base_frame}"
            )
            return
        self.anchor_xyz = (message.point.x, message.point.y, message.point.z)
        self._publish_preview()

    def _publish_preview(self) -> None:
        markers = make_preview_markers(
            self.strokes,
            anchor_xyz=self.anchor_xyz,
            writing_width_m=self.writing_width_m,
            writing_height_m=self.writing_height_m,
            frame_id=self.base_frame,
            stamp=self.get_clock().now().to_msg(),
            z_offset_m=self.z_offset_m,
        )
        self.publisher.publish(markers)
        self.get_logger().info(
            f"Published read-only handwriting preview: {len(self.strokes)} strokes, "
            f"anchor={self.anchor_xyz}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = HandwritingPathPreviewNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
