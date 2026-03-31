import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Ur3FollowJointTrajectoryClientNode(Node):
    def __init__(self) -> None:
        super().__init__("ur3_follow_joint_trajectory_client_node")

        self.action_name = self.declare_parameter(
            "action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        ).value
        self.server_wait_timeout_sec = float(
            self.declare_parameter("server_wait_timeout_sec", 10.0).value
        )
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

        if self.server_wait_timeout_sec <= 0.0:
            self.get_logger().warn("server_wait_timeout_sec <= 0.0, fallback to 10.0")
            self.server_wait_timeout_sec = 10.0

        self._action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self._goal_sent = False
        self._shutdown_requested = False
        self._start_timer = self.create_timer(0.3, self._maybe_send_goal)

        self.get_logger().info(
            f"Action client started. action_name={self.action_name} timeout={self.server_wait_timeout_sec:.1f}s"
        )

    def build_demo_goal(self) -> FollowJointTrajectory.Goal:
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        points: list[JointTrajectoryPoint] = []

        # TODO(human): 请补写轨迹点（2~3个点）并赋值给 trajectory.points。
        # 要求：
        # 1) 每个点的 positions 长度必须等于 len(self.joint_names)
        # 2) time_from_start 必须严格递增，例如 2s / 4s / 6s
        # 3) 建议首点是接近当前位姿的温和动作，避免大幅突变
        # 4) 建议至少一个点设置 velocity 全 0，便于观察停稳行为
        # 参考骨架（可删改）：
        # p1 = JointTrajectoryPoint()
        # p1.positions = [ ... 6个关节值 ... ]
        # p1.velocities = [0.0] * len(self.joint_names)
        # p1.time_from_start = Duration(sec=2, nanosec=0)
        # points.append(p1)
        # trajectory.points = points
        p1 = JointTrajectoryPoint()
        p1.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p1.velocities = [0.0] * len(self.joint_names)
        p1.time_from_start = Duration(sec=2, nanosec=0)
        points.append(p1)

        p2 = JointTrajectoryPoint()
        p2.positions = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        p2.velocities = [0.0] * len(self.joint_names)
        p2.time_from_start = Duration(sec=4, nanosec=0)
        points.append(p2)

        p3 = JointTrajectoryPoint()
        p3.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p3.velocities = [0.0] * len(self.joint_names)
        p3.time_from_start = Duration(sec=6, nanosec=0)
        points.append(p3)
        trajectory.points = points
        self._validate_trajectory_points(trajectory.points)

        goal.trajectory = trajectory
        goal.goal_time_tolerance = Duration(sec=1, nanosec=0)
        return goal

    def _maybe_send_goal(self) -> None:
        if self._goal_sent:
            return

        self._goal_sent = True
        self._start_timer.cancel()

        self.get_logger().info("Waiting for action server...")
        if not self._action_client.wait_for_server(timeout_sec=self.server_wait_timeout_sec):
            self.get_logger().error(
                f"Action server not available: {self.action_name} "
                f"(waited {self.server_wait_timeout_sec:.1f}s)"
            )
            self._request_shutdown("Action server unavailable")
            return

        try:
            goal_msg = self.build_demo_goal()
        except NotImplementedError as exc:
            self.get_logger().error(str(exc))
            self._request_shutdown("Demo goal not implemented yet")
            return
        except ValueError as exc:
            self.get_logger().error(f"Invalid goal trajectory: {exc}")
            self._request_shutdown("Invalid trajectory data")
            return

        self.get_logger().info("Sending FollowJointTrajectory goal...")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_feedback,
        )
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by action server")
            self._request_shutdown("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg) -> None:
        desired_len = len(feedback_msg.feedback.desired.positions)
        actual_len = len(feedback_msg.feedback.actual.positions)
        self.get_logger().info(
            f"Feedback: desired_positions={desired_len} actual_positions={actual_len}"
        )

    def _on_result(self, future) -> None:
        result = future.result().result
        status = future.result().status
        self.get_logger().info(
            f"Result received: status={status} error_code={result.error_code} "
            f"error_string='{result.error_string}'"
        )
        self._request_shutdown("Goal finished")

    def _request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self.get_logger().info(f"Shutting down node: {reason}")
        rclpy.shutdown()

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ur3FollowJointTrajectoryClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
