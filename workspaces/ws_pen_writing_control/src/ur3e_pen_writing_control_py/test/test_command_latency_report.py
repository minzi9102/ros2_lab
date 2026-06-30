import pytest

from ur3e_pen_writing_control_py.command_latency_report import (
    TimedVector,
    estimate_joint_follow_delay,
    stats_seconds,
    successor_latencies_ns,
)


def test_successor_latencies_match_next_controller_command():
    latencies = successor_latencies_ns(
        [1_000_000_000, 2_000_000_000],
        [1_010_000_000, 2_012_000_000],
    )

    stats = stats_seconds(latencies)

    assert stats["sample_count"] == 2
    assert stats["avg_sec"] == pytest.approx(0.011)
    assert stats["max_sec"] == pytest.approx(0.012)


def test_joint_follow_delay_finds_known_shift():
    commands = [
        TimedVector(index * 100_000_000, (float(index), float(index) * 0.5))
        for index in range(10)
    ]
    states = [
        TimedVector(sample.time_ns + 40_000_000, sample.values)
        for sample in commands
    ]

    result = estimate_joint_follow_delay(
        commands,
        states,
        max_delay_sec=0.1,
        step_sec=0.02,
    )

    assert result["best_delay_sec"] == pytest.approx(0.04)
    assert result["rms_rad"] == pytest.approx(0.0)
    assert result["sample_count"] == 10
