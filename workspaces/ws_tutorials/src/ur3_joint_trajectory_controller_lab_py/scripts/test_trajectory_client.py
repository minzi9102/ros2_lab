#!/usr/bin/env python3
"""
Minimal trajectory client for testing joint_trajectory_controller.
Sends a simple 3-point trajectory to the FollowJointTrajectory action.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration


class TrajectoryClientNode(Node):
    def __init__(self):
        super().__init__("trajectory_client")
        self._action_client = ActionClient(
            self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def send_trajectory(self):
        """Send a simple 3-point trajectory."""
        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Point 1: Start position (all zeros)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * len(self.joint_names)
        point1.velocities = [0.0] * len(self.joint_names)
        point1.time_from_start = Duration(sec=0, nanosec=0)
        trajectory.points.append(point1)

        # Point 2: Middle position
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, -0.5, 0.5, 0.0, 0.0, 0.0]
        point2.velocities = [0.0] * len(self.joint_names)
        point2.time_from_start = Duration(sec=2, nanosec=0)
        trajectory.points.append(point2)

        # Point 3: End position
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0] * len(self.joint_names)
        point3.velocities = [0.0] * len(self.joint_names)
        point3.time_from_start = Duration(sec=4, nanosec=0)
        trajectory.points.append(point3)

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info("Sending trajectory goal...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(
            f"Feedback: desired={feedback.feedback.desired.positions}, "
            f"actual={feedback.feedback.actual.positions}"
        )

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryClientNode()
    node.send_trajectory()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
