import json
import math
from pathlib import Path
from typing import Iterable


SCHEMA = "ur3e_handwriting/v1"
Point2 = tuple[float, float]
Strokes = list[list[Point2]]


def validate_strokes(strokes: object) -> Strokes:
    if not isinstance(strokes, list) or not strokes:
        raise ValueError("strokes must be a non-empty list")
    validated: Strokes = []
    for stroke_index, stroke in enumerate(strokes):
        if not isinstance(stroke, list) or len(stroke) < 2:
            raise ValueError(f"stroke {stroke_index} must contain at least two points")
        points: list[Point2] = []
        for point_index, point in enumerate(stroke):
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError(
                    f"stroke {stroke_index} point {point_index} must be [x, y]"
                )
            try:
                x, y = float(point[0]), float(point[1])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"stroke {stroke_index} point {point_index} must be numeric"
                ) from exc
            if not all(
                math.isfinite(value) and 0.0 <= value <= 1.0
                for value in (x, y)
            ):
                raise ValueError(
                    f"stroke {stroke_index} point {point_index} must be finite and in [0, 1]"
                )
            points.append((x, y))
        if max(math.dist(points[0], point) for point in points[1:]) <= 1e-12:
            raise ValueError(f"stroke {stroke_index} has no movement")
        validated.append(points)
    return validated


def load_handwriting(path: str | Path) -> Strokes:
    with Path(path).open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict) or document.get("schema") != SCHEMA:
        raise ValueError(f"schema must equal {SCHEMA!r}")
    return validate_strokes(document.get("strokes"))


def save_handwriting(path: str | Path, strokes: object) -> None:
    validated = validate_strokes(strokes)
    document = {
        "schema": SCHEMA,
        "strokes": [
            [[round(x, 8), round(y, 8)] for x, y in stroke]
            for stroke in validated
        ],
    }
    with Path(path).open("w", encoding="utf-8") as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def compile_strokes(
    strokes: object,
    *,
    writing_width_m: float,
    writing_height_m: float,
    simplify_tolerance_m: float = 0.00025,
    cartesian_step_m: float = 0.0005,
) -> Strokes:
    validated = validate_strokes(strokes)
    if writing_width_m <= 0.0 or writing_height_m <= 0.0:
        raise ValueError("writing dimensions must be positive")
    if simplify_tolerance_m < 0.0:
        raise ValueError("simplify_tolerance_m must be non-negative")
    if cartesian_step_m <= 0.0:
        raise ValueError("cartesian_step_m must be positive")

    all_points = [point for stroke in validated for point in stroke]
    minimum_x = min(point[0] for point in all_points)
    maximum_x = max(point[0] for point in all_points)
    minimum_y = min(point[1] for point in all_points)
    maximum_y = max(point[1] for point in all_points)
    extent_x = maximum_x - minimum_x
    extent_y = maximum_y - minimum_y
    scales = []
    if extent_x > 1e-12:
        scales.append(writing_width_m / extent_x)
    if extent_y > 1e-12:
        scales.append(writing_height_m / extent_y)
    if not scales:
        raise ValueError("handwriting path has no movement")
    scale = min(scales)
    center_x = (minimum_x + maximum_x) / 2.0
    center_y = (minimum_y + maximum_y) / 2.0

    compiled = []
    for stroke in validated:
        physical = [
            ((x - center_x) * scale, -(y - center_y) * scale)
            for x, y in stroke
        ]
        simplified = simplify_polyline(physical, simplify_tolerance_m)
        compiled.append(resample_polyline(simplified, cartesian_step_m))
    return compiled


def simplify_polyline(points: Iterable[Point2], tolerance: float) -> list[Point2]:
    points = list(points)
    if tolerance < 0.0:
        raise ValueError("tolerance must be non-negative")
    if len(points) <= 2:
        return points
    start, end = points[0], points[-1]
    furthest_index = 0
    furthest_distance = -1.0
    for index, point in enumerate(points[1:-1], start=1):
        distance = _distance_to_segment(point, start, end)
        if distance > furthest_distance:
            furthest_index = index
            furthest_distance = distance
    if furthest_distance <= tolerance:
        return [start, end]
    left = simplify_polyline(points[: furthest_index + 1], tolerance)
    right = simplify_polyline(points[furthest_index:], tolerance)
    return left[:-1] + right


def resample_polyline(points: Iterable[Point2], maximum_step: float) -> list[Point2]:
    points = list(points)
    if maximum_step <= 0.0:
        raise ValueError("maximum_step must be positive")
    if len(points) < 2:
        raise ValueError("polyline must contain at least two points")
    result = [points[0]]
    for start, end in zip(points, points[1:]):
        distance = math.dist(start, end)
        if distance <= 1e-12:
            continue
        segment_count = max(1, math.ceil(distance / maximum_step))
        for index in range(1, segment_count + 1):
            ratio = index / segment_count
            result.append(
                (
                    start[0] + (end[0] - start[0]) * ratio,
                    start[1] + (end[1] - start[1]) * ratio,
                )
            )
    if len(result) < 2:
        raise ValueError("polyline has no movement")
    return result


def path_length(strokes: Iterable[Iterable[Point2]]) -> float:
    total = 0.0
    for stroke in strokes:
        points = list(stroke)
        total += sum(
            math.dist(start, end) for start, end in zip(points, points[1:])
        )
    return total


def _distance_to_segment(point: Point2, start: Point2, end: Point2) -> float:
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared <= 1e-24:
        return math.dist(point, start)
    projection = (
        (point[0] - start[0]) * delta_x + (point[1] - start[1]) * delta_y
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    nearest = (start[0] + projection * delta_x, start[1] + projection * delta_y)
    return math.dist(point, nearest)
