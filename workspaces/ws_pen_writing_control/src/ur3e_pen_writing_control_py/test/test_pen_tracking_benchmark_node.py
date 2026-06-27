import csv
import math

import pytest

from ur3e_pen_writing_control_py.pen_tracking_benchmark_node import (
    BenchmarkInfrastructureError,
    MetricThresholds,
    analyze_alignment_csv,
    benchmark_phases,
    joy_message_for_target,
    scored_phase_windows,
    total_phase_duration,
    write_summary_files,
)


def test_joy_message_maps_target_axes_to_existing_pen_controls():
    plus_x = joy_message_for_target(1.0, 0.0)
    plus_y = joy_message_for_target(0.0, 1.0)
    quit_msg = joy_message_for_target(0.0, 0.0, quit_requested=True)

    assert list(plus_x.axes) == [0.0, -1.0]
    assert list(plus_x.buttons) == [0, 0]
    assert list(plus_y.axes) == [-1.0, -0.0]
    assert list(quit_msg.buttons) == [0, 1]


def test_benchmark_sequence_has_scored_direction_windows():
    phases = benchmark_phases()
    windows = scored_phase_windows(phases)

    assert total_phase_duration(phases) == pytest.approx(15.5)
    assert list(windows) == ["plus_x", "minus_x", "plus_y", "minus_y"]
    assert windows["plus_x"] == pytest.approx((3.0, 4.0))
    assert windows["minus_x"] == pytest.approx((5.5, 6.5))
    assert windows["plus_y"] == pytest.approx((8.0, 9.0))
    assert windows["minus_y"] == pytest.approx((10.5, 11.5))


def test_alignment_csv_analysis_passes_good_tracking(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(
        csv_path,
        [
            (0.0, 0.020, 15.0, 15.1),
            (3.0, 0.002, 2.0, 2.2),
            (3.5, 0.003, 4.0, 4.1),
            (6.0, 0.004, 5.0, 5.3),
            (9.0, 0.003, 6.0, 6.5),
            (11.0, 0.002, 7.0, 7.2),
            (15.4, 0.001, 2.5, 2.7),
        ],
    )

    result = analyze_alignment_csv(csv_path)

    assert result["status"] == "PASS"
    assert result["sample_count"] == 7
    assert result["post_initial_sample_count"] == 6
    assert result["phases"]["plus_x"]["finite_sample_count"] == 2
    assert all(check["passed"] for check in result["checks"])


def test_alignment_csv_analysis_marks_metric_fail_without_infrastructure_error(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(
        csv_path,
        [
            (0.0, 0.020, 15.0, 15.0),
            (3.0, 0.004, 10.0, 10.1),
            (4.0, 0.004, 9.0, 9.1),
            (15.4, 0.001, 2.0, 2.1),
        ],
    )

    result = analyze_alignment_csv(csv_path)

    assert result["status"] == "FAIL"
    assert any(not check["passed"] for check in result["checks"])


def test_alignment_csv_analysis_fails_no_nan_check(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(
        csv_path,
        [
            (0.0, 0.020, 15.0, 15.0),
            (3.0, math.nan, math.nan, math.nan),
            (3.5, 0.002, 2.0, 2.0),
        ],
    )

    result = analyze_alignment_csv(csv_path)

    assert result["status"] == "FAIL"
    no_nan = next(
        check for check in result["checks"] if check["name"] == "no_nan_after_initial"
    )
    assert not no_nan["passed"]


def test_alignment_csv_analysis_rejects_missing_required_columns(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["elapsed_sec", "position_m"])
        writer.writerow([0.0, 0.0])

    with pytest.raises(BenchmarkInfrastructureError):
        analyze_alignment_csv(csv_path)


def test_alignment_csv_analysis_rejects_no_post_initial_samples(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(csv_path, [(0.0, 0.020, 15.0, 15.0)])

    with pytest.raises(BenchmarkInfrastructureError):
        analyze_alignment_csv(csv_path)


def test_write_summary_files_records_fail_report(tmp_path):
    result = {
        "status": "FAIL",
        "csv_path": "tool_alignment_error.csv",
        "sample_count": 2,
        "post_initial_sample_count": 1,
        "initial_exclusion_sec": 3.0,
        "overall": {
            "position_m": {"avg": 0.001, "max": 0.002},
            "z_axis_deg": {"avg": 10.0, "max": 12.0},
            "full_quaternion_deg": {"avg": 10.2, "max": 12.2},
        },
        "phases": {},
        "checks": [
            {
                "name": "z_axis_max",
                "passed": False,
                "value": 12.0,
                "threshold": 8.0,
                "unit": "deg",
            }
        ],
    }

    json_path = tmp_path / "tracking_summary.json"
    markdown_path = tmp_path / "tracking_summary.md"
    write_summary_files(
        result=result,
        json_path=json_path,
        markdown_path=markdown_path,
    )

    assert '"status": "FAIL"' in json_path.read_text(encoding="utf-8")
    assert "Status: **FAIL**" in markdown_path.read_text(encoding="utf-8")


def test_custom_thresholds_can_make_same_csv_fail(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(csv_path, [(3.0, 0.002, 2.0, 2.0)])

    result = analyze_alignment_csv(
        csv_path,
        thresholds=MetricThresholds(z_axis_avg_deg=1.0),
    )

    assert result["status"] == "FAIL"


def _write_alignment_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "elapsed_sec",
                "position_m",
                "z_axis_deg",
                "full_quaternion_deg",
                "pose_command_armed",
                "pose_command_published",
                "has_motion_intent",
                "virtual_pen_settling",
            ]
        )
        for elapsed, position, z_axis, quaternion in rows:
            writer.writerow([elapsed, position, z_axis, quaternion, 1, 1, 0, 0])
