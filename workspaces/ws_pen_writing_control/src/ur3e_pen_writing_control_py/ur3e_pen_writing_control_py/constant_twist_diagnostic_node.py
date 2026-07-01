import json
import math
import time
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from urdf_parser_py.urdf import URDF


PROFILES = {
    "pure_x": ((0.03, 0.0, 0.0), (0.0, 0.0, 0.0)),
    "pure_y": ((0.0, 0.03, 0.0), (0.0, 0.0, 0.0)),
    "pure_yaw": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.3)),
}


@dataclass
class PoseSample:
    time_sec: float
    joints: tuple[float, ...]
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]


def matmul(a, b):
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(4)) for col in range(4))
        for row in range(4)
    )


def translation_matrix(xyz):
    x, y, z = xyz
    return (
        (1.0, 0.0, 0.0, x),
        (0.0, 1.0, 0.0, y),
        (0.0, 0.0, 1.0, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def rotation_rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0),
        (-sp, cp * sr, cp * cr, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def axis_angle_matrix(axis, angle):
    x, y, z = axis
    norm = math.sqrt(x * x + y * y + z * z)
    if norm == 0.0:
        return identity_matrix()
    x, y, z = x / norm, y / norm, z / norm
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def identity_matrix():
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def distance(a, b):
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def rotation_angle(start, end):
    relative_trace = sum(
        sum(start[k][row] * end[k][row] for k in range(3)) for row in range(3)
    )
    value = max(-1.0, min(1.0, (relative_trace - 1.0) / 2.0))
    return math.acos(value)


class UrdfFk:
    def __init__(self, robot_description: str, base_link: str, tip_link: str):
        robot = URDF.from_xml_string(robot_description)
        child_to_joint = {joint.child: joint for joint in robot.joints}
        chain = []
        link = tip_link
        while link != base_link:
            joint = child_to_joint.get(link)
            if joint is None:
                raise ValueError(f"No URDF chain from {base_link} to {tip_link}")
            chain.append(joint)
            link = joint.parent
        self.chain = list(reversed(chain))
        self.joint_names = tuple(
            joint.name
            for joint in self.chain
            if joint.type in ("revolute", "continuous", "prismatic")
        )

    def pose(self, positions_by_joint: dict[str, float]):
        transform = identity_matrix()
        for joint in self.chain:
            origin = joint.origin
            xyz = (0.0, 0.0, 0.0) if origin is None else tuple(origin.xyz)
            rpy = (0.0, 0.0, 0.0) if origin is None else tuple(origin.rpy)
            transform = matmul(transform, translation_matrix(xyz))
            transform = matmul(transform, rotation_rpy_matrix(rpy))
            value = positions_by_joint.get(joint.name, 0.0)
            axis = (1.0, 0.0, 0.0) if joint.axis is None else tuple(joint.axis)
            if joint.type in ("revolute", "continuous"):
                transform = matmul(transform, axis_angle_matrix(axis, value))
            elif joint.type == "prismatic":
                transform = matmul(
                    transform,
                    translation_matrix(tuple(value * component for component in axis)),
                )
        position = (transform[0][3], transform[1][3], transform[2][3])
        rotation = tuple(tuple(transform[row][col] for col in range(3)) for row in range(3))
        return position, rotation


class ConstantTwistDiagnosticNode(Node):
    def __init__(self):
        super().__init__("constant_twist_diagnostic")
        self.profile = str(self.declare_parameter("twist_profile", "pure_x").value)
        if self.profile not in PROFILES:
            raise ValueError(f"twist_profile must be one of {sorted(PROFILES)}")
        self.duration_sec = float(self.declare_parameter("duration_sec", 5.0).value)
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 125.0).value
        )
        self.base_link = str(self.declare_parameter("base_link", "base_link").value)
        self.tip_link = str(self.declare_parameter("tip_link", "tool0").value)
        self.robot_description = str(
            self.declare_parameter("robot_description", "").value
        )
        self.report_json_path = Path(
            str(self.declare_parameter("report_json_path", "").value)
        )
        self.report_markdown_path = Path(
            str(self.declare_parameter("report_markdown_path", "").value)
        )
        if self.duration_sec <= 0.0:
            raise ValueError("duration_sec must be greater than 0.0")
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than 0.0")
        if not self.robot_description:
            raise ValueError("robot_description parameter is required")

        self.fk = UrdfFk(self.robot_description, self.base_link, self.tip_link)
        self.linear, self.angular = PROFILES[self.profile]
        self.command_samples: list[PoseSample] = []
        self.actual_samples: list[PoseSample] = []
        self.status_counts = Counter()
        self._active = False
        self._latest_actual_by_name: dict[str, float] | None = None
        self._latest_command: tuple[float, ...] | None = None

        self.command_type_client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )
        self.twist_publisher = self.create_publisher(
            TwistStamped,
            "/servo_node/delta_twist_cmds",
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/forward_position_controller/commands",
            self._on_command,
            100,
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 100)
        self.create_subscription(
            ServoStatus,
            "/servo_node/status",
            self._on_status,
            100,
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_command(self, msg: Float64MultiArray) -> None:
        values = tuple(float(value) for value in msg.data[: len(self.fk.joint_names)])
        self._latest_command = values
        if self._active and len(values) == len(self.fk.joint_names):
            self.command_samples.append(self._sample_from_ordered_joints(values))

    def _on_joint_state(self, msg: JointState) -> None:
        if len(msg.position) < len(msg.name):
            return
        positions = dict(zip(msg.name, (float(value) for value in msg.position)))
        self._latest_actual_by_name = positions
        if self._active and all(name in positions for name in self.fk.joint_names):
            joints = tuple(positions[name] for name in self.fk.joint_names)
            self.actual_samples.append(self._sample_from_ordered_joints(joints))

    def _on_status(self, msg: ServoStatus) -> None:
        self.status_counts[int(msg.code)] += 1

    def _sample_from_ordered_joints(self, joints: tuple[float, ...]) -> PoseSample:
        positions_by_joint = dict(zip(self.fk.joint_names, joints))
        position, rotation = self.fk.pose(positions_by_joint)
        return PoseSample(
            time_sec=self._now_sec(),
            joints=joints,
            position=position,
            rotation=rotation,
        )

    def run(self) -> int:
        if not self.command_type_client.wait_for_service(timeout_sec=60.0):
            self.get_logger().error("Timed out waiting for Servo command type service.")
            return 2
        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST
        future = self.command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None or not future.result().success:
            self.get_logger().error("Servo rejected TWIST command mode.")
            return 3

        deadline = time.monotonic() + 5.0
        while self._latest_actual_by_name is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._latest_actual_by_name is None:
            self.get_logger().error("Timed out waiting for joint state samples.")
            return 4

        self._active = True
        period = 1.0 / self.publish_rate_hz
        end_time = time.monotonic() + self.duration_sec
        while time.monotonic() < end_time:
            self.twist_publisher.publish(self._twist_message(self.linear, self.angular))
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self._active = False
        for _ in range(10):
            self.twist_publisher.publish(
                self._twist_message((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            )
            rclpy.spin_once(self, timeout_sec=0.01)

        report = self._make_report()
        self._write_report(report)
        self.get_logger().info(f"Wrote constant Twist report: {self.report_markdown_path}")
        return 0

    def _twist_message(self, linear, angular) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_link
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        return msg

    def _make_report(self) -> dict:
        linear_speed = math.sqrt(sum(value * value for value in self.linear))
        angular_speed = math.sqrt(sum(value * value for value in self.angular))
        expected_linear_m = linear_speed * self.duration_sec
        expected_angular_rad = angular_speed * self.duration_sec
        return {
            "profile": self.profile,
            "duration_sec": self.duration_sec,
            "publish_rate_hz": self.publish_rate_hz,
            "linear_mps": self.linear,
            "angular_radps": self.angular,
            "expected_linear_m": expected_linear_m,
            "expected_angular_rad": expected_angular_rad,
            "commanded": self._summarize_samples(
                self.command_samples,
                expected_linear_m,
                expected_angular_rad,
            ),
            "actual": self._summarize_samples(
                self.actual_samples,
                expected_linear_m,
                expected_angular_rad,
            ),
            "servo_status_counts": dict(sorted(self.status_counts.items())),
        }

    def _summarize_samples(self, samples, expected_linear_m, expected_angular_rad):
        if len(samples) < 2:
            return {"sample_count": len(samples)}
        path_length = sum(
            distance(samples[index - 1].position, samples[index].position)
            for index in range(1, len(samples))
        )
        joint_steps = [
            max(
                abs(samples[index].joints[joint] - samples[index - 1].joints[joint])
                for joint in range(len(samples[index].joints))
            )
            for index in range(1, len(samples))
        ]
        linear_displacement = distance(samples[0].position, samples[-1].position)
        angular_displacement = rotation_angle(samples[0].rotation, samples[-1].rotation)
        return {
            "sample_count": len(samples),
            "start_xyz_m": samples[0].position,
            "end_xyz_m": samples[-1].position,
            "linear_displacement_m": linear_displacement,
            "path_length_m": path_length,
            "path_ratio": (
                path_length / expected_linear_m if expected_linear_m > 0.0 else None
            ),
            "angular_displacement_rad": angular_displacement,
            "angular_ratio": (
                angular_displacement / expected_angular_rad
                if expected_angular_rad > 0.0
                else None
            ),
            "linear_drift_m": linear_displacement,
            "max_joint_step_rad": max(joint_steps),
            "mean_joint_step_rad": sum(joint_steps) / len(joint_steps),
        }

    def _write_report(self, report: dict) -> None:
        if self.report_json_path:
            self.report_json_path.parent.mkdir(parents=True, exist_ok=True)
            self.report_json_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        if self.report_markdown_path:
            self.report_markdown_path.parent.mkdir(parents=True, exist_ok=True)
            self.report_markdown_path.write_text(
                self._markdown_report(report),
                encoding="utf-8",
            )

    def _markdown_report(self, report: dict) -> str:
        lines = [
            "# Constant Twist Diagnostic",
            "",
            f"- profile: `{report['profile']}`",
            f"- duration: `{report['duration_sec']:.3f} s`",
            f"- publish rate: `{report['publish_rate_hz']:.1f} Hz`",
            f"- linear: `{tuple(round(v, 6) for v in report['linear_mps'])} m/s`",
            f"- angular: `{tuple(round(v, 6) for v in report['angular_radps'])} rad/s`",
            f"- expected linear: `{report['expected_linear_m'] * 1000.0:.2f} mm`",
            f"- expected angular: `{report['expected_angular_rad']:.3f} rad`",
            f"- Servo status counts: `{report['servo_status_counts']}`",
            "",
        ]
        for name in ("commanded", "actual"):
            summary = report[name]
            lines.extend([f"## {name.title()} FK", ""])
            if summary.get("sample_count", 0) < 2:
                lines.append(f"- samples: `{summary.get('sample_count', 0)}`")
                lines.append("")
                continue
            lines.extend(
                [
                    f"- samples: `{summary['sample_count']}`",
                    f"- start XYZ: `{_mm_tuple(summary['start_xyz_m'])} mm`",
                    f"- end XYZ: `{_mm_tuple(summary['end_xyz_m'])} mm`",
                    f"- linear displacement: `{summary['linear_displacement_m'] * 1000.0:.2f} mm`",
                    f"- path length: `{summary['path_length_m'] * 1000.0:.2f} mm`",
                    f"- path ratio: `{_optional_float(summary['path_ratio'])}`",
                    f"- angular displacement: `{summary['angular_displacement_rad']:.4f} rad`",
                    f"- angular ratio: `{_optional_float(summary['angular_ratio'])}`",
                    f"- linear drift: `{summary['linear_drift_m'] * 1000.0:.2f} mm`",
                    f"- max joint step: `{summary['max_joint_step_rad']:.6f} rad`",
                    f"- mean joint step: `{summary['mean_joint_step_rad']:.6f} rad`",
                    "",
                ]
            )
        return "\n".join(lines) + "\n"


def _mm_tuple(values) -> tuple[float, float, float]:
    return tuple(round(value * 1000.0, 3) for value in values)


def _optional_float(value) -> str:
    return "n/a" if value is None else f"{value:.4f}"


def main(args=None):
    rclpy.init(args=args)
    node = ConstantTwistDiagnosticNode()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
