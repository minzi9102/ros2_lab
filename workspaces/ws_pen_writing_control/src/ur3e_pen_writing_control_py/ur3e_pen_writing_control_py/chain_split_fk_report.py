import argparse
import bisect
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path

from .command_latency_report import (
    TimedVector,
    estimate_joint_follow_delay,
    interpolate_vector,
)
from .constant_twist_diagnostic_node import UrdfFk
from .pose_math import Quaternion


PRIMARY_TARGET_TOPIC = "/pen_writing/target_pose"
FALLBACK_TARGET_TOPIC = "/servo_node/pose_target_cmds"
COMMANDED_JOINT_TOPIC = "/forward_position_controller/commands"
ACTUAL_JOINT_TOPIC = "/joint_states"
MAX_TARGET_TO_COMMAND_LATENCY_SEC = 0.05


@dataclass(frozen=True)
class TimedPose:
    time_ns: int
    position: tuple[float, float, float]
    orientation: Quaternion


def load_robot_description(urdf_xacro: Path, ur_type: str) -> str:
    text = urdf_xacro.read_text(encoding="utf-8")
    if urdf_xacro.suffix != ".xacro" and "xacro:" not in text:
        return text
    result = subprocess.run(
        ["xacro", str(urdf_xacro), "name:=ur", f"ur_type:={ur_type}"],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def read_bag_streams(
    bag_path: Path,
    *,
    joint_names: tuple[str, ...],
) -> tuple[list[TimedPose], list[TimedPose], list[TimedVector], list[TimedVector]]:
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
    primary_targets: list[TimedPose] = []
    fallback_targets: list[TimedPose] = []
    commands: list[TimedVector] = []
    states: list[TimedVector] = []
    tracked_topics = {
        PRIMARY_TARGET_TOPIC,
        FALLBACK_TARGET_TOPIC,
        COMMANDED_JOINT_TOPIC,
        ACTUAL_JOINT_TOPIC,
    }
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in tracked_topics:
            continue
        msg = deserialize_message(data, get_message(type_map[topic]))
        if topic in (PRIMARY_TARGET_TOPIC, FALLBACK_TARGET_TOPIC):
            sample = TimedPose(
                time_ns=timestamp,
                position=(
                    float(msg.pose.position.x),
                    float(msg.pose.position.y),
                    float(msg.pose.position.z),
                ),
                orientation=Quaternion(
                    x=float(msg.pose.orientation.x),
                    y=float(msg.pose.orientation.y),
                    z=float(msg.pose.orientation.z),
                    w=float(msg.pose.orientation.w),
                ),
            )
            if topic == PRIMARY_TARGET_TOPIC:
                primary_targets.append(sample)
            else:
                fallback_targets.append(sample)
        elif topic == COMMANDED_JOINT_TOPIC and len(msg.data) >= len(joint_names):
            commands.append(
                TimedVector(
                    time_ns=timestamp,
                    values=tuple(float(value) for value in msg.data[: len(joint_names)]),
                )
            )
        elif topic == ACTUAL_JOINT_TOPIC:
            ordered = ordered_joint_values(msg.name, msg.position, joint_names)
            if ordered is not None:
                states.append(TimedVector(time_ns=timestamp, values=ordered))
    return primary_targets, fallback_targets, commands, states


def ordered_joint_values(
    names,
    positions,
    joint_names: tuple[str, ...],
) -> tuple[float, ...] | None:
    by_name = dict(zip(names, positions))
    if not all(name in by_name for name in joint_names):
        return None
    return tuple(float(by_name[name]) for name in joint_names)


def stats(values: list[float]) -> dict:
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return {
            "count": 0,
            "avg": math.nan,
            "rms": math.nan,
            "max": math.nan,
            "p95": math.nan,
            "min": math.nan,
        }
    p95_index = min(len(finite) - 1, math.ceil(0.95 * len(finite)) - 1)
    mean = sum(finite) / len(finite)
    return {
        "count": len(finite),
        "avg": mean,
        "rms": math.sqrt(sum(value * value for value in finite) / len(finite)),
        "max": max(finite),
        "p95": finite[p95_index],
        "min": min(finite),
    }


def vector_norm(vector: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return vector_norm(tuple(a[index] - b[index] for index in range(3)))


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def angle_between(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    a_norm = vector_norm(a)
    b_norm = vector_norm(b)
    if a_norm == 0.0 or b_norm == 0.0:
        return 0.0
    cosine = max(-1.0, min(1.0, dot(a, b) / (a_norm * b_norm)))
    return math.acos(cosine)


def quaternion_to_matrix(
    quaternion: Quaternion,
) -> tuple[tuple[float, float, float], ...]:
    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return (
        (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ),
        (
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ),
        (
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ),
    )


def rotation_angle(
    start: tuple[tuple[float, float, float], ...],
    end: tuple[tuple[float, float, float], ...],
) -> float:
    relative_trace = sum(
        sum(start[k][row] * end[k][row] for k in range(3))
        for row in range(3)
    )
    value = max(-1.0, min(1.0, (relative_trace - 1.0) / 2.0))
    return math.acos(value)


def matrix_z_axis(
    rotation: tuple[tuple[float, float, float], ...],
) -> tuple[float, float, float]:
    return (rotation[0][2], rotation[1][2], rotation[2][2])


def path_length(points: list[tuple[float, float, float]]) -> float:
    return sum(
        distance(points[index - 1], points[index]) for index in range(1, len(points))
    )


def axis_correlation(
    source: list[tuple[float, float, float]],
    target: list[tuple[float, float, float]],
) -> dict[str, float]:
    result = {}
    for axis, label in enumerate(("x", "y", "z")):
        source_values = [point[axis] for point in source]
        target_values = [point[axis] for point in target]
        result[label] = pearson_correlation(source_values, target_values)
    return result


def pearson_correlation(a: list[float], b: list[float]) -> float:
    if len(a) != len(b) or len(a) < 2:
        return 0.0
    mean_a = sum(a) / len(a)
    mean_b = sum(b) / len(b)
    numerator = sum((x - mean_a) * (y - mean_b) for x, y in zip(a, b))
    denom_a = math.sqrt(sum((x - mean_a) ** 2 for x in a))
    denom_b = math.sqrt(sum((y - mean_b) ** 2 for y in b))
    if denom_a == 0.0 or denom_b == 0.0:
        return 0.0
    return numerator / (denom_a * denom_b)


def summarize_matched_target_to_command(
    targets: list[TimedPose],
    commands: list[TimedVector],
    *,
    fk: UrdfFk,
) -> tuple[dict, dict, list[dict], dict]:
    command_times = [sample.time_ns for sample in commands]
    matches: list[dict] = []
    max_latency_ns = int(MAX_TARGET_TO_COMMAND_LATENCY_SEC * 1e9)
    for target in targets:
        index = bisect.bisect_left(command_times, target.time_ns)
        if index >= len(commands):
            continue
        command = commands[index]
        latency_ns = command.time_ns - target.time_ns
        if latency_ns < 0 or latency_ns > max_latency_ns:
            continue
        command_position, command_rotation = fk.pose(
            dict(zip(fk.joint_names, command.values))
        )
        target_rotation = quaternion_to_matrix(target.orientation)
        matches.append(
            {
                "target": target,
                "command": command,
                "latency_sec": latency_ns / 1e9,
                "command_position": command_position,
                "command_rotation": command_rotation,
                "position_error": tuple(
                    command_position[axis] - target.position[axis] for axis in range(3)
                ),
                "position_error_norm": distance(command_position, target.position),
                "z_axis_error_rad": angle_between(
                    matrix_z_axis(target_rotation),
                    matrix_z_axis(command_rotation),
                ),
                "full_quaternion_error_rad": rotation_angle(
                    target_rotation,
                    command_rotation,
                ),
            }
        )

    if not matches:
        raise RuntimeError("no target/command FK matches found in rosbag")

    target_points = [match["target"].position for match in matches]
    command_points = [match["command_position"] for match in matches]
    mean_offset = tuple(
        sum(match["position_error"][axis] for match in matches) / len(matches)
        for axis in range(3)
    )
    target_center = tuple(
        sum(point[axis] for point in target_points) / len(target_points)
        for axis in range(3)
    )
    command_center = tuple(
        sum(point[axis] for point in command_points) / len(command_points)
        for axis in range(3)
    )
    centered_errors = [
        distance(
            tuple(point[axis] - target_center[axis] for axis in range(3)),
            tuple(
                command_points[index][axis] - command_center[axis]
                for axis in range(3)
            ),
        )
        for index, point in enumerate(target_points)
    ]
    target_path_length = path_length(target_points)
    command_path_length = path_length(command_points)
    target_to_command = {
        "matching": (
            "first commanded joint sample at or after target pose "
            "timestamp, max 0.05s"
        ),
        "latency_sec": stats([match["latency_sec"] for match in matches]),
        "position_error_m": stats([match["position_error_norm"] for match in matches]),
        "axis_position_error_m": {
            axis: stats([match["position_error"][idx] for match in matches])
            for idx, axis in enumerate(("x", "y", "z"))
        },
        "z_axis_error_deg": stats(
            [math.degrees(match["z_axis_error_rad"]) for match in matches]
        ),
        "full_quaternion_error_deg": stats(
            [math.degrees(match["full_quaternion_error_rad"]) for match in matches]
        ),
    }
    shape_check = {
        "centered_position_error_m": stats(centered_errors),
        "mean_offset_command_minus_target_m": list(mean_offset),
        "bbox_target_xyz_m": bbox_size(target_points),
        "bbox_commanded_fk_xyz_m": bbox_size(command_points),
        "path_length_target_m": target_path_length,
        "path_length_commanded_fk_m": command_path_length,
        "path_length_ratio_commanded_over_target": (
            command_path_length / target_path_length
            if target_path_length > 0.0
            else math.nan
        ),
        "axis_correlation": axis_correlation(target_points, command_points),
    }
    sample_counts = {
        "target_pose": len(targets),
        "commanded_joints": len(commands),
        "target_to_command_fk_matches": len(matches),
    }
    return target_to_command, shape_check, matches, sample_counts


def summarize_command_to_actual(
    commands: list[TimedVector],
    actual_states: list[TimedVector],
    *,
    fk: UrdfFk,
) -> tuple[dict, list[dict], int]:
    delay = estimate_joint_follow_delay(commands, actual_states)
    delay_sec = delay["best_delay_sec"]
    if not math.isfinite(delay_sec):
        raise RuntimeError("unable to estimate commanded-to-actual follow delay")
    actual_times = [sample.time_ns for sample in actual_states]
    matches: list[dict] = []
    for command in commands:
        actual = interpolate_vector(
            actual_states,
            actual_times,
            command.time_ns + int(delay_sec * 1e9),
        )
        if actual is None:
            continue
        commanded_pose = fk.pose(dict(zip(fk.joint_names, command.values)))
        actual_pose = fk.pose(dict(zip(fk.joint_names, actual)))
        joint_errors = tuple(
            actual[index] - command.values[index]
            for index in range(len(command.values))
        )
        matches.append(
            {
                "joint_errors": joint_errors,
                "joint_norm_error": vector_norm(joint_errors),
                "fk_position_error": distance(commanded_pose[0], actual_pose[0]),
                "fk_z_axis_error_rad": angle_between(
                    matrix_z_axis(commanded_pose[1]),
                    matrix_z_axis(actual_pose[1]),
                ),
            }
        )
    if not matches:
        raise RuntimeError("no commanded/actual FK matches found at best delay")
    per_joint = {}
    for index, joint_name in enumerate(fk.joint_names):
        per_joint[joint_name] = stats(
            [match["joint_errors"][index] for match in matches]
        )
    summary = {
        "alignment_delay_sec": delay_sec,
        "joint_norm_error_rad": stats(
            [match["joint_norm_error"] for match in matches]
        ),
        "per_joint_error_rad": per_joint,
        "fk_position_error_m": stats(
            [match["fk_position_error"] for match in matches]
        ),
        "fk_z_axis_error_deg": stats(
            [math.degrees(match["fk_z_axis_error_rad"]) for match in matches]
        ),
    }
    return summary, matches, len(actual_states)


def bbox_size(points: list[tuple[float, float, float]]) -> list[float]:
    return [
        max(point[axis] for point in points) - min(point[axis] for point in points)
        for axis in range(3)
    ]


def build_report(
    *,
    bag_path: Path,
    summary_json_path: Path,
    urdf_xacro: Path,
    ur_type: str,
) -> tuple[dict, list[dict], list[dict]]:
    robot_description = load_robot_description(urdf_xacro, ur_type)
    fk = UrdfFk(robot_description, "base_link", "tool0")
    primary_targets, fallback_targets, commands, actual_states = read_bag_streams(
        bag_path,
        joint_names=fk.joint_names,
    )
    target_topic = PRIMARY_TARGET_TOPIC if primary_targets else FALLBACK_TARGET_TOPIC
    targets = primary_targets or fallback_targets
    if not targets:
        raise RuntimeError("no target pose samples found in rosbag")
    if not commands:
        raise RuntimeError("no controller command samples found in rosbag")
    if not actual_states:
        raise RuntimeError("no joint state samples found in rosbag")
    (
        target_to_command,
        shape_check,
        target_matches,
        sample_counts,
    ) = summarize_matched_target_to_command(targets, commands, fk=fk)
    commanded_to_actual, actual_matches, actual_count = summarize_command_to_actual(
        commands,
        actual_states,
        fk=fk,
    )
    sample_counts["actual_joints"] = actual_count
    sample_counts["command_to_actual_matches"] = len(actual_matches)
    report = {
        "bag_path": str(bag_path),
        "summary_json_path": str(summary_json_path),
        "urdf_xacro": str(urdf_xacro),
        "ur_type": ur_type,
        "topics": {
            "target_pose": target_topic,
            "commanded_joints": COMMANDED_JOINT_TOPIC,
            "actual_joints": ACTUAL_JOINT_TOPIC,
        },
        "sample_counts": sample_counts,
        "target_pose_to_commanded_joint_fk": target_to_command,
        "shape_check_target_vs_commanded_fk": shape_check,
        "commanded_joints_to_actual_joints": commanded_to_actual,
    }
    report["plots"] = {}
    return report, target_matches, actual_matches


def generate_plots(
    *,
    report: dict,
    target_matches: list[dict],
    actual_matches: list[dict],
    output_dir: Path,
) -> dict[str, str]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plots = {}

    xy_path = output_dir / "target_vs_commanded_fk_xy.png"
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.plot(
        [match["target"].position[0] for match in target_matches],
        [match["target"].position[1] for match in target_matches],
        label="target",
    )
    ax.plot(
        [match["command_position"][0] for match in target_matches],
        [match["command_position"][1] for match in target_matches],
        label="commanded_fk",
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_title("Target vs commanded FK XY")
    fig.tight_layout()
    fig.savefig(xy_path)
    plt.close(fig)
    plots["target_vs_commanded_fk_xy"] = str(xy_path)

    time_path = output_dir / "target_vs_commanded_fk_time.png"
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    start_ns = target_matches[0]["target"].time_ns
    times_sec = [(match["target"].time_ns - start_ns) / 1e9 for match in target_matches]
    for axis, label in enumerate(("x", "y", "z")):
        axes[axis].plot(
            times_sec,
            [match["target"].position[axis] for match in target_matches],
            label=f"target_{label}",
        )
        axes[axis].plot(
            times_sec,
            [match["command_position"][axis] for match in target_matches],
            label=f"commanded_{label}",
        )
        axes[axis].set_ylabel(f"{label} (m)")
        axes[axis].legend()
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(time_path)
    plt.close(fig)
    plots["target_vs_commanded_fk_time"] = str(time_path)

    error_path = output_dir / "commanded_vs_actual_fk_error.png"
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(
        range(len(actual_matches)),
        [match["joint_norm_error"] for match in actual_matches],
        label="joint_norm_error_rad",
    )
    axes[0].legend()
    axes[0].set_ylabel("joint err (rad)")
    axes[1].plot(
        range(len(actual_matches)),
        [match["fk_position_error"] for match in actual_matches],
        label="fk_position_error_m",
    )
    axes[1].plot(
        range(len(actual_matches)),
        [math.degrees(match["fk_z_axis_error_rad"]) for match in actual_matches],
        label="fk_z_axis_error_deg",
    )
    axes[1].legend()
    axes[1].set_ylabel("pose err")
    axes[1].set_xlabel("sample")
    fig.tight_layout()
    fig.savefig(error_path)
    plt.close(fig)
    plots["commanded_vs_actual_fk_error"] = str(error_path)
    report["plots"] = plots
    return plots


def write_report(report: dict, json_path: Path, markdown_path: Path) -> None:
    json_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    markdown_path.write_text(format_markdown_report(report), encoding="utf-8")


def format_markdown_report(report: dict) -> str:
    target = report["target_pose_to_commanded_joint_fk"]
    shape = report["shape_check_target_vs_commanded_fk"]
    actual = report["commanded_joints_to_actual_joints"]
    lines = [
        "# URSim Chain Split FK Report",
        "",
        f"Bag: `{report['bag_path']}`",
        f"Tracking summary: `{report['summary_json_path']}`",
        "",
        "## Target pose -> commanded-joint FK",
        "",
        "| Metric | Avg/RMS | P95 | Max |",
        "| --- | ---: | ---: | ---: |",
        "| Position error m | "
        f"{_pair(target['position_error_m'])} | "
        f"{_fmt(target['position_error_m']['p95'])} | "
        f"{_fmt(target['position_error_m']['max'])} |",
        "| Z-axis error deg | "
        f"{_pair(target['z_axis_error_deg'])} | "
        f"{_fmt(target['z_axis_error_deg']['p95'])} | "
        f"{_fmt(target['z_axis_error_deg']['max'])} |",
        "",
        "## Shape check",
        "",
        f"Centered shape RMS: `{_fmt(shape['centered_position_error_m']['rms'])} m`",
        f"Mean offset command-target XYZ: `{shape['mean_offset_command_minus_target_m']} m`",
        f"BBox target XYZ: `{shape['bbox_target_xyz_m']} m`",
        f"BBox commanded FK XYZ: `{shape['bbox_commanded_fk_xyz_m']} m`",
        "Path length ratio commanded/target: "
        f"`{_fmt(shape['path_length_ratio_commanded_over_target'])}`",
        f"Axis correlation XYZ: `{shape['axis_correlation']}`",
        "",
        "## Commanded joints -> actual joints",
        "",
        f"Best delay: `{_fmt(actual['alignment_delay_sec'])} s`",
        f"Joint norm RMS: `{_fmt(actual['joint_norm_error_rad']['rms'])} rad`",
        f"FK position RMS: `{_fmt(actual['fk_position_error_m']['rms'])} m`",
        f"FK position max: `{_fmt(actual['fk_position_error_m']['max'])} m`",
        f"FK Z-axis RMS: `{_fmt(actual['fk_z_axis_error_deg']['rms'])} deg`",
        "",
        "## Plots",
        "",
    ]
    for path in report.get("plots", {}).values():
        lines.append(f"- `{path}`")
    return "\n".join(lines) + "\n"


def _pair(summary: dict) -> str:
    return f"{_fmt(summary['avg'])} / {_fmt(summary['rms'])}"


def _fmt(value: float) -> str:
    if not math.isfinite(value):
        return "nan"
    return f"{value:.6f}"


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--summary-json", required=True, type=Path)
    parser.add_argument("--urdf-xacro", required=True, type=Path)
    parser.add_argument("--ur-type", required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-md", type=Path)
    args = parser.parse_args(argv)

    output_json = args.output_json or args.summary_json.parent / "chain_split_fk_report.json"
    output_md = args.output_md or args.summary_json.parent / "chain_split_fk_report.md"
    report, target_matches, actual_matches = build_report(
        bag_path=args.bag,
        summary_json_path=args.summary_json,
        urdf_xacro=args.urdf_xacro,
        ur_type=args.ur_type,
    )
    generate_plots(
        report=report,
        target_matches=target_matches,
        actual_matches=actual_matches,
        output_dir=output_json.parent,
    )
    write_report(report, output_json, output_md)
    print(f"Wrote {output_json}")
    print(f"Wrote {output_md}")


if __name__ == "__main__":
    main()
