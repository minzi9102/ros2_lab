"""
Task 4G: forward_command_controller 对比实验节点

以 10 Hz 向 forward_position_controller 发布关节位置指令，
用于与 joint_trajectory_controller 的行为进行横向对比。

使用方法：
  ros2 run ur3_forward_command_controller_lab_py forward_cmd_publisher
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# 测试目标：shoulder_pan_joint 移动到 0.5 rad，其余关节保持 0
TARGET_POSITIONS = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
PUBLISH_RATE_HZ = 10.0


class ForwardCmdPublisher(Node):
    def __init__(self):
        super().__init__('forward_cmd_publisher')
        self._pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10,
        )
        self._timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)
        self.get_logger().info(
            f'Publishing to /forward_position_controller/commands '
            f'at {PUBLISH_RATE_HZ} Hz, target={TARGET_POSITIONS}'
        )

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = TARGET_POSITIONS
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
