import argparse
import bisect
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


FORWARD_CONTROLLER_JOINTS = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


@dataclass(frozen=True)
class TimedVector:
    time_ns: int
    values: tuple[float, ...]


def successor_latencies_ns(
    source_times_ns: Iterable[int],
    target_times_ns: Iterable[int],
    *,
    max_latency_sec: float = 0.2,
) -> list[int]:
    targets = sorted(target_times_ns)
    max_latency_ns = int(max_latency_sec * 1e9)
    latencies = []
    for source_time in sorted(source_times_ns):
        index = bisect.bisect_left(targets, source_time)
        if index >= len(targets):
            continue
        latency = targets[index] - source_time
        if 0 <= latency <= max_latency_ns:
            latencies.append(latency)
    return latencies


def stats_seconds(latencies_ns: list[int]) -> dict:
    if not latencies_ns:
        return {
            "sample_count": 0,
            "avg_sec": math.nan,
            "max_sec": math.nan,
            "min_sec": math.nan,
            "p95_sec": math.nan,
        }
    values = sorted(latency / 1e9 for latency in latencies_ns)
    p95_index = min(len(values) - 1, math.ceil(0.95 * len(values)) - 1)
    return {
        "sample_count": len(values),
        "avg_sec": sum(values) / len(values),
        "max_sec": max(values),
        "min_sec": min(values),
        "p95_sec": values[p95_index],
    }


def estimate_joint_follow_delay(
    commands: list[TimedVector],
    states: list[TimedVector],
    *,
    max_delay_sec: float = 0.3,
    step_sec: float = 0.004,
) -> dict:
    if not commands or not states:
        return _empty_delay_result()

    commands = sorted(commands, key=lambda sample: sample.time_ns)
    states = sorted(states, key=lambda sample: sample.time_ns)
    state_times = [sample.time_ns for sample in states]
    best = None
    steps = int(max_delay_sec / step_sec) + 1
    for step in range(steps + 1):
        delay_ns = int(step * step_sec * 1e9)
        sse = 0.0
        value_count = 0
        sample_count = 0
        for command in commands:
            state_values = interpolate_vector(states, state_times, command.time_ns + delay_ns)
            if state_values is None:
                continue
            for command_value, state_value in zip(command.values, state_values):
                sse += (command_value - state_value) ** 2
                value_count += 1
            sample_count += 1
        if value_count == 0:
            continue
        rms = math.sqrt(sse / value_count)
        candidate = {
            "best_delay_sec": delay_ns / 1e9,
            "rms_rad": rms,
            "sample_count": sample_count,
        }
        if best is None or rms < best["rms_rad"]:
            best = candidate
    return _empty_delay_result() if best is None else best


def interpolate_vector(
    samples: list[TimedVector],
    times_ns: list[int],
    target_time_ns: int,
) -> tuple[float, ...] | None:
    index = bisect.bisect_left(times_ns, target_time_ns)
    if index < len(samples) and times_ns[index] == target_time_ns:
        return samples[index].values
    if index == 0 or index >= len(samples):
        return None
    before = samples[index - 1]
    after = samples[index]
    span = after.time_ns - before.time_ns
    if span <= 0:
        return None
    ratio = (target_time_ns - before.time_ns) / span
    return tuple(
        start + (end - start) * ratio
        for start, end in zip(before.values, after.values)
    )


def build_latency_report(
    *,
    bag_path: Path,
    summary_json_path: Path,
    profile: str,
) -> dict:
    pose_times, commands, states = read_bag_streams(bag_path)
    if pose_times:
        start_ns = min(pose_times)
        end_ns = max(pose_times)
        commands = [
            sample for sample in commands if start_ns <= sample.time_ns <= end_ns
        ]
        states = [
            sample
            for sample in states
            if start_ns <= sample.time_ns <= end_ns + int(0.3 * 1e9)
        ]
    command_times = [sample.time_ns for sample in commands]
    summary = json.loads(summary_json_path.read_text(encoding="utf-8"))
    return {
        "bag_path": str(bag_path),
        "summary_json_path": str(summary_json_path),
        "benchmark_status": summary.get("status"),
        "profile": profile,
        "topics": {
            "pose_target": "/servo_node/pose_target_cmds",
            "controller_command": "/forward_position_controller/commands",
            "joint_states": "/joint_states",
        },
        "pose_target_to_controller_command": stats_seconds(
            successor_latencies_ns(pose_times, command_times)
        ),
        "controller_command_to_joint_state": estimate_joint_follow_delay(
            commands,
            states,
        ),
        "sample_counts": {
            "pose_target": len(pose_times),
            "controller_command": len(commands),
            "joint_states": len(states),
        },
    }


def read_bag_streams(
    bag_path: Path,
) -> tuple[list[int], list[TimedVector], list[TimedVector]]:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    pose_times = []
    commands = []
    states = []
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in {
            "/servo_node/pose_target_cmds",
            "/forward_position_controller/commands",
            "/joint_states",
        }:
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        if topic == "/servo_node/pose_target_cmds":
            pose_times.append(timestamp)
        elif topic == "/forward_position_controller/commands" and len(msg.data) >= 6:
            commands.append(TimedVector(timestamp, tuple(float(v) for v in msg.data[:6])))
        elif topic == "/joint_states":
            values = _joint_values(msg.name, msg.position)
            if values is not None:
                states.append(TimedVector(timestamp, values))
    return pose_times, commands, states


def write_report(report: dict, json_path: Path, markdown_path: Path) -> None:
    json_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# Command Latency Report",
        "",
        f"Bag: `{report['bag_path']}`",
        f"Tracking summary: `{report['summary_json_path']}`",
        f"Profile: `{report['profile']}`",
        "",
        "| Segment | Samples | Avg s | P95 s | Max s |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    pose = report["pose_target_to_controller_command"]
    lines.append(
        "| pose_target -> controller_command | "
        f"{pose['sample_count']} | {_fmt(pose['avg_sec'])} | "
        f"{_fmt(pose['p95_sec'])} | {_fmt(pose['max_sec'])} |"
    )
    joint = report["controller_command_to_joint_state"]
    lines.extend(
        [
            "",
            "## Joint Follow",
            "",
            f"Best delay: `{_fmt(joint['best_delay_sec'])} s`",
            f"RMS at best delay: `{_fmt(joint['rms_rad'])} rad`",
            f"Samples: `{joint['sample_count']}`",
            "",
        ]
    )
    markdown_path.write_text("\n".join(lines), encoding="utf-8")


def _joint_values(names, positions) -> tuple[float, ...] | None:
    by_name = dict(zip(names, positions))
    if not all(joint in by_name for joint in FORWARD_CONTROLLER_JOINTS):
        return None
    return tuple(float(by_name[joint]) for joint in FORWARD_CONTROLLER_JOINTS)


def _empty_delay_result() -> dict:
    return {
        "best_delay_sec": math.nan,
        "rms_rad": math.nan,
        "sample_count": 0,
    }


def _fmt(value) -> str:
    if not isinstance(value, float):
        return str(value)
    if math.isnan(value):
        return "nan"
    return f"{value:.6f}"


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--summary-json", required=True, type=Path)
    parser.add_argument("--profile", default="eight_direction")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    args = parser.parse_args(argv)

    output_json = args.output_json or args.summary_json.parent / "command_latency_report.json"
    output_md = args.output_md or args.summary_json.parent / "command_latency_report.md"
    report = build_latency_report(
        bag_path=args.bag,
        summary_json_path=args.summary_json,
        profile=args.profile,
    )
    write_report(report, output_json, output_md)
    print(f"Wrote {output_json}")
    print(f"Wrote {output_md}")


if __name__ == "__main__":
    main()
