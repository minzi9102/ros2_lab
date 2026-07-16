import json
import math

import pytest

from ur3e_force_pen_writing_py.handwriting_path import (
    compile_strokes,
    load_handwriting,
    path_length,
    resample_polyline,
    save_handwriting,
    SCHEMA,
    simplify_polyline,
    validate_strokes,
)


def test_handwriting_round_trip_uses_versioned_normalized_schema(tmp_path):
    path = tmp_path / "writing.json"
    strokes = [[(0.1, 0.2), (0.3, 0.4)], [(0.8, 0.7), (0.9, 0.6)]]

    save_handwriting(path, strokes)

    document = json.loads(path.read_text(encoding="utf-8"))
    assert document["schema"] == SCHEMA
    assert load_handwriting(path) == strokes


@pytest.mark.parametrize(
    ("strokes", "message"),
    (
        ([], "non-empty"),
        ([[[0.0, 0.0]]], "at least two"),
        ([[[0.0, 0.0], [0.0, 0.0]]], "no movement"),
        ([[[0.0, 0.0], [1.1, 0.0]]], r"in \[0, 1\]"),
        ([[[0.0, 0.0], [math.nan, 0.0]]], "finite"),
    ),
)
def test_stroke_validation_rejects_unsafe_documents(strokes, message):
    with pytest.raises(ValueError, match=message):
        validate_strokes(strokes)


def test_loader_rejects_unknown_schema(tmp_path):
    path = tmp_path / "writing.json"
    path.write_text('{"schema":"other","strokes":[]}', encoding="utf-8")

    with pytest.raises(ValueError, match="schema"):
        load_handwriting(path)


def test_compiler_fits_aspect_ratio_centers_and_inverts_canvas_y():
    compiled = compile_strokes(
        [[(0.0, 0.0), (1.0, 0.5), (1.0, 1.0)]],
        writing_width_m=0.010,
        writing_height_m=0.020,
        simplify_tolerance_m=0.0,
        cartesian_step_m=0.005,
    )[0]

    assert compiled[0] == pytest.approx((-0.005, 0.005))
    assert compiled[-1] == pytest.approx((0.005, -0.005))
    assert max(point[0] for point in compiled) - min(
        point[0] for point in compiled
    ) == pytest.approx(0.010)
    assert max(point[1] for point in compiled) - min(
        point[1] for point in compiled
    ) == pytest.approx(0.010)


def test_rdp_preserves_corner_and_endpoints():
    points = [(0.0, 0.0), (0.004, 0.0), (0.005, 0.0), (0.005, 0.005)]

    simplified = simplify_polyline(points, 0.00025)

    assert simplified == [(0.0, 0.0), (0.005, 0.0), (0.005, 0.005)]


def test_resampling_bounds_every_cartesian_step_and_preserves_endpoint():
    resampled = resample_polyline([(0.0, 0.0), (0.0011, 0.0)], 0.0005)

    assert resampled[-1] == (0.0011, 0.0)
    assert (
        max(math.dist(a, b) for a, b in zip(resampled, resampled[1:]))
        <= 0.0005
    )
    assert path_length([resampled]) == pytest.approx(0.0011)


def test_compiler_keeps_multiple_strokes_separate():
    compiled = compile_strokes(
        [[(0.0, 0.0), (0.5, 0.0)], [(0.5, 1.0), (1.0, 1.0)]],
        writing_width_m=0.010,
        writing_height_m=0.010,
    )

    assert len(compiled) == 2
    assert compiled[0][-1] != compiled[1][0]
