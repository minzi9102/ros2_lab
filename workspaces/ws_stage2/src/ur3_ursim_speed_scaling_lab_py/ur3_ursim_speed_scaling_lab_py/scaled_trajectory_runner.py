from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur3_ursim_speed_scaling_lab_py.speed_scaling_monitor import speed_scaling_qos_profile


class Ur3ScaledTrajectoryRunner(Node):
    def __init__(self) -> None:
        super().__init__("ur3_scaled_trajectory_runner")

        self.action_name = self.declare_parameter(
            "action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        ).value
        self.joint_state_topic = self.declare_parameter("joint_state_topic", "/joint_states").value
        self.speed_scaling_topic = self.declare_parameter(
            "speed_scaling_topic", "/speed_scaling_state_broadcaster/speed_scaling"
        ).value
        self.server_wait_timeout_sec = float(
            self.declare_parameter("server_wait_timeout_sec", 10.0).value
        )
        self.start_delay_sec = float(self.declare_parameter("start_delay_sec", 1.0).value)
        self.goal_time_tolerance_sec = float(
            self.declare_parameter("goal_time_tolerance_sec", 1.0).value
        )
        self.feedback_log_stride = int(self.declare_parameter("feedback_log_stride", 20).value)
        self.joint_names = list(
            self.declare_parameter(
                "joint_names",
                [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            ).value
        )

        if self.feedback_log_stride <= 0:
            self.get_logger().warn("feedback_log_stride <= 0, fallback to 1")
            self.feedback_log_stride = 1

        self._action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self._latest_positions_by_name: dict[str, float] = {}
        self._latest_speed_scaling: float | None = None
        self._feedback_count = 0
        self._goal_sent = False
        self._shutdown_requested = False
        self._goal_sent_monotonic: float | None = None
        self._speed_scaling_before_send: float | None = None

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self._on_joint_state,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.create_subscription(
            Float64,
            self.speed_scaling_topic,
            self._on_speed_scaling,
            speed_scaling_qos_profile(),
        )
        self._start_timer = self.create_timer(self.start_delay_sec, self._maybe_send_goal)

        self.get_logger().info(
            "scaled_trajectory_runner started. "
            f"action={self.action_name} joint_state_topic={self.joint_state_topic} "
            f"speed_scaling_topic={self.speed_scaling_topic}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest_positions_by_name = {
            name: msg.position[idx]
            for idx, name in enumerate(msg.name)
            if idx < len(msg.position)
        }

    def _on_speed_scaling(self, msg: Float64) -> None:
        self._latest_speed_scaling = float(msg.data)

    def _maybe_send_goal(self) -> None:
        if self._goal_sent or self._shutdown_requested:
            return

        try:
            current_positions = self._ordered_current_positions()
            current_speed_scaling = self._current_speed_scaling()
        except RuntimeError as exc:
            self.get_logger().warn(str(exc))
            return

        self.get_logger().info("Waiting for FollowJointTrajectory action server...")
        if not self._action_client.wait_for_server(timeout_sec=self.server_wait_timeout_sec):
            self.get_logger().error(
                f"Action server unavailable: {self.action_name} "
                f"(waited {self.server_wait_timeout_sec:.1f}s)"
            )
            self._request_shutdown("Action server unavailable")
            return

        try:
            goal_msg = self.build_demo_goal(current_positions)
        except NotImplementedError as exc:
            self.get_logger().error(str(exc))
            self._request_shutdown("TODO(human) not finished")
            return
        except ValueError as exc:
            self.get_logger().error(f"Invalid trajectory: {exc}")
            self._request_shutdown("Trajectory validation failed")
            return

        self._goal_sent = True
        self._start_timer.cancel()
        self._goal_sent_monotonic = time.monotonic()
        self._speed_scaling_before_send = current_speed_scaling

        self.get_logger().info(
            "Sending FollowJointTrajectory goal... "
            f"speed_scaling_before_send={current_speed_scaling:.1f}%"
        )
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        send_goal_future.add_done_callback(self._on_goal_response)

    def _ordered_current_positions(self) -> list[float]:
        if not self._latest_positions_by_name:
            raise RuntimeError("Waiting for current /joint_states before sending trajectory.")

        ordered_positions: list[float] = []
        missing_names: list[str] = []
        for joint_name in self.joint_names:
            if joint_name not in self._latest_positions_by_name:
                missing_names.append(joint_name)
                continue
            ordered_positions.append(self._latest_positions_by_name[joint_name])

        if missing_names:
            raise RuntimeError(
                "Current /joint_states is missing joints: " + ", ".join(missing_names)
            )

        return ordered_positions

    def _current_speed_scaling(self) -> float:
        if self._latest_speed_scaling is None:
            raise RuntimeError("Waiting for current speed scaling before sending trajectory.")
        return self._latest_speed_scaling

    def build_demo_goal(self, current_positions: list[float]) -> FollowJointTrajectory.Goal:
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points = self.plan_demo_points(current_positions)
        self._validate_trajectory_points(trajectory.points)

        tolerance_sec = max(self.goal_time_tolerance_sec, 0.0)
        tolerance_nsec = int((tolerance_sec - int(tolerance_sec)) * 1_000_000_000)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(sec=int(tolerance_sec), nanosec=tolerance_nsec)
        return goal

    def plan_demo_points(self, current_positions: list[float]) -> list[JointTrajectoryPoint]:
        # 以 current_positions 为起点，设计 2 到 3 个保守轨迹点，用于比较
        # `100%` 与降速条件下的执行耗时。建议：
        # 1. 只让 1 到 2 个关节发生小幅变化，避免排障面过大；
        # 2. 每个 point 的 time_from_start 必须严格递增；
        # 3. 终点最好回到 current_positions，便于比较发送前后状态。
        start = self._make_point(current_positions, 1)
        middle_positions = list(current_positions)
        middle_positions[0] += 0.5
        middle_positions[1] -= 0.5
        middle = self._make_point(middle_positions, 3)
        end = self._make_point(current_positions, 5)
        return [start, middle, end]


    def _make_point(self, positions: list[float], sec: int) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * len(self.joint_names)
        point.time_from_start = Duration(sec=sec, nanosec=0)
        return point

    def _validate_trajectory_points(self, points: list[JointTrajectoryPoint]) -> None:
        if not points:
            raise ValueError("trajectory.points is empty")

        expected_len = len(self.joint_names)
        prev_time_ns = -1
        for idx, point in enumerate(points):
            if len(point.positions) != expected_len:
                raise ValueError(
                    f"point[{idx}] positions length={len(point.positions)} expected={expected_len}"
                )
            if point.velocities and len(point.velocities) != expected_len:
                raise ValueError(
                    f"point[{idx}] velocities length={len(point.velocities)} expected={expected_len}"
                )

            time_ns = point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec
            if time_ns <= prev_time_ns:
                raise ValueError(
                    f"point[{idx}] time_from_start={time_ns}ns is not strictly increasing"
                )
            prev_time_ns = time_ns

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by controller.")
            self._request_shutdown("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        self._feedback_count += 1
        if self._feedback_count == 1 or self._feedback_count % self.feedback_log_stride == 0:
            latest_speed_scaling = (
                "N/A"
                if self._latest_speed_scaling is None
                else f"{self._latest_speed_scaling:.1f}%"
            )
            self.get_logger().info(
                "Feedback received: "
                f"count={self._feedback_count} "
                f"desired_len={len(feedback_msg.feedback.desired.positions)} "
                f"actual_len={len(feedback_msg.feedback.actual.positions)} "
                f"latest_speed_scaling={latest_speed_scaling}"
            )

    def _on_result(self, future) -> None:
        wrapped_result = future.result()
        result = wrapped_result.result
        elapsed_sec = 0.0
        if self._goal_sent_monotonic is not None:
            elapsed_sec = time.monotonic() - self._goal_sent_monotonic

        speed_scaling_before_send = (
            "N/A"
            if self._speed_scaling_before_send is None
            else f"{self._speed_scaling_before_send:.1f}%"
        )
        self.get_logger().info(
            f"Result received: status={wrapped_result.status} "
            f"error_code={result.error_code} "
            f"error_string='{result.error_string}' "
            f"elapsed_sec={elapsed_sec:.2f} "
            f"speed_scaling_before_send={speed_scaling_before_send}"
        )
        self._request_shutdown("Goal finished")

    def _request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.get_logger().info(f"Shutting down node: {reason}")
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur3ScaledTrajectoryRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
