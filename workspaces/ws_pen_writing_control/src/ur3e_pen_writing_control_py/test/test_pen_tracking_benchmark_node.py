import csv
import math

import pytest

from ur3e_pen_writing_control_py.pen_tracking_benchmark_node import (
    BenchmarkInfrastructureError,
    LONG_PLUS_X_PROFILE,
    LONG_MINUS_Y_PLUS_XY_PROFILE,
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

    assert total_phase_duration(phases) == pytest.approx(19.0)
    assert list(windows) == [
        "plus_x",
        "minus_x",
        "plus_y",
        "minus_y",
        "plus_xy",
        "minus_xy",
        "plus_x_minus_y",
        "minus_x_plus_y",
    ]
    assert windows["plus_x"] == pytest.approx((2.0, 3.5))
    assert windows["minus_x"] == pytest.approx((3.5, 5.0))
    assert windows["plus_y"] == pytest.approx((5.0, 6.5))
    assert windows["minus_y"] == pytest.approx((6.5, 8.0))
    assert windows["plus_xy"] == pytest.approx((8.0, 9.5))
    assert windows["minus_xy"] == pytest.approx((9.5, 11.0))
    assert windows["plus_x_minus_y"] == pytest.approx((11.0, 12.5))
    assert windows["minus_x_plus_y"] == pytest.approx((12.5, 14.0))


def test_long_profile_runs_minus_y_and_plus_xy_as_five_second_segments():
    phases = benchmark_phases(LONG_MINUS_Y_PLUS_XY_PROFILE)
    windows = scored_phase_windows(phases)

    assert total_phase_duration(phases) == pytest.approx(19.0)
    assert list(windows) == ["minus_y", "plus_xy"]
    assert windows["minus_y"] == pytest.approx((2.0, 7.0))
    assert windows["plus_xy"] == pytest.approx((9.0, 14.0))


def test_long_plus_x_profile_runs_one_eight_second_segment():
    phases = benchmark_phases(LONG_PLUS_X_PROFILE)
    windows = scored_phase_windows(phases)

    assert total_phase_duration(phases) == pytest.approx(15.0)
    assert list(windows) == ["plus_x"]
    assert windows["plus_x"] == pytest.approx((2.0, 10.0))


def test_alignment_csv_analysis_passes_good_tracking(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(
        csv_path,
        [
            (0.0, 0.020, 15.0, 15.1),
            (2.0, 0.002, 2.0, 2.2),
            (3.0, 0.003, 4.0, 4.1),
            (4.0, 0.004, 5.0, 5.3),
            (5.5, 0.003, 6.0, 6.5),
            (7.0, 0.002, 7.0, 7.2),
            (9.0, 0.0025, 4.5, 4.7),
            (11.5, 0.0028, 4.8, 5.0),
            (13.0, 0.0026, 4.2, 4.4),
            (18.9, 0.001, 2.5, 2.7),
        ],
    )

    result = analyze_alignment_csv(csv_path)

    assert result["status"] == "PASS"
    assert result["sample_count"] == 10
    assert result["post_initial_sample_count"] == 9
    assert result["phases"]["plus_x"]["finite_sample_count"] == 2
    assert result["phases"]["plus_xy"]["finite_sample_count"] == 1
    assert all(check["passed"] for check in result["checks"])


def test_alignment_csv_analysis_offsets_score_windows_after_real_alignment(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(
        csv_path,
        [
            (0.0, 0.200, 80.0, 80.0),
            (5.0, 0.002, 1.0, 1.0),
            (7.0, 0.003, 2.0, 2.0),
            (8.0, 0.004, 3.0, 3.0),
            (23.9, 0.001, 1.0, 1.0),
            (25.0, 0.300, 90.0, 90.0),
        ],
    )

    result = analyze_alignment_csv(csv_path, score_start_sec=5.0)

    assert result["status"] == "PASS"
    assert result["score_start_sec"] == pytest.approx(5.0)
    assert result["score_end_sec"] == pytest.approx(24.0)
    assert result["initial_exclusion_sec"] == pytest.approx(7.0)
    assert result["phases"]["plus_x"]["finite_sample_count"] == 2
    assert result["overall"]["position_m"]["max"] == pytest.approx(0.004)


def test_alignment_csv_analysis_reports_long_phase_convergence(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(
        csv_path,
        [
            (0.0, 0.020, 15.0, 15.0),
            (2.1, 0.008, 2.0, 2.0),
            (2.8, 0.006, 2.0, 2.0),
            (6.2, 0.002, 1.0, 1.0),
            (6.8, 0.001, 1.0, 1.0),
            (9.1, 0.010, 2.0, 2.0),
            (9.8, 0.008, 2.0, 2.0),
            (13.2, 0.003, 1.0, 1.0),
            (13.8, 0.002, 1.0, 1.0),
            (18.5, 0.001, 1.0, 1.0),
        ],
    )

    result = analyze_alignment_csv(
        csv_path,
        phases=benchmark_phases(LONG_MINUS_Y_PLUS_XY_PROFILE),
    )

    minus_y = result["phases"]["minus_y"]["convergence_1s"]
    plus_xy = result["phases"]["plus_xy"]["convergence_1s"]
    assert minus_y["first"]["position_m"]["avg"] == pytest.approx(0.007)
    assert minus_y["last"]["position_m"]["avg"] == pytest.approx(0.0015)
    assert minus_y["position_avg_delta_m"] < 0.0
    assert plus_xy["position_avg_delta_m"] < 0.0


def test_alignment_csv_analysis_marks_metric_fail_without_infrastructure_error(tmp_path):
    csv_path = tmp_path / "tool_alignment_error.csv"
    _write_alignment_csv(
        csv_path,
        [
            (0.0, 0.020, 15.0, 15.0),
            (2.0, 0.004, 10.0, 10.1),
            (3.0, 0.004, 9.0, 9.1),
            (18.9, 0.001, 2.0, 2.1),
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
            (2.0, math.nan, math.nan, math.nan),
            (3.0, 0.002, 2.0, 2.0),
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
        "initial_exclusion_sec": 2.0,
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
    _write_alignment_csv(csv_path, [(2.0, 0.002, 2.0, 2.0)])

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
