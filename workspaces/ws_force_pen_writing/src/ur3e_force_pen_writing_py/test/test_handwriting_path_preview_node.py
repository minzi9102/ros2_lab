import inspect
from types import SimpleNamespace

import pytest
from visualization_msgs.msg import Marker

from ur3e_force_pen_writing_py.handwriting_path_preview_node import (
    HandwritingPathPreviewNode,
    make_preview_markers,
)


def test_preview_keeps_strokes_separate_and_marks_order():
    markers = make_preview_markers(
        [[(-0.005, 0.0), (0.0, 0.0)], [(0.0, 0.005), (0.005, 0.005)]],
        anchor_xyz=(0.4, 0.1, 0.2),
        writing_width_m=0.01,
        writing_height_m=0.01,
        frame_id="base_link",
        stamp=SimpleNamespace(sec=1, nanosec=2),
        z_offset_m=0.0005,
    )

    line_markers = [marker for marker in markers.markers if marker.type == Marker.LINE_STRIP]
    labels = [marker for marker in markers.markers if marker.type == Marker.TEXT_VIEW_FACING]
    assert len(line_markers) == 3
    assert len(line_markers[1].points) == 2
    assert len(line_markers[2].points) == 2
    assert line_markers[1].points[-1] != line_markers[2].points[0]
    assert [label.text for label in labels] == ["1", "2"]


def test_preview_offsets_compiled_points_from_paper_anchor():
    markers = make_preview_markers(
        [[(-0.005, 0.005), (0.005, -0.005)]],
        anchor_xyz=(0.4, 0.1, 0.2),
        writing_width_m=0.01,
        writing_height_m=0.01,
        frame_id="base_link",
        stamp=SimpleNamespace(sec=1, nanosec=2),
        z_offset_m=0.0005,
    )

    stroke = next(
        marker for marker in markers.markers if marker.ns == "handwriting_strokes"
    )
    assert (stroke.points[0].x, stroke.points[0].y, stroke.points[0].z) == pytest.approx(
        (0.395, 0.105, 0.2005)
    )
    assert (stroke.points[-1].x, stroke.points[-1].y) == pytest.approx((0.405, 0.095))


def test_preview_node_has_no_motion_service_or_action_interface():
    source = inspect.getsource(HandwritingPathPreviewNode)

    assert "create_service" not in source
    assert "ActionClient" not in source
    assert "create_publisher(MarkerArray" in source
    assert "create_timer(0.5, self._publish_preview)" in source
    assert "if not self._anchor_ready" in source
