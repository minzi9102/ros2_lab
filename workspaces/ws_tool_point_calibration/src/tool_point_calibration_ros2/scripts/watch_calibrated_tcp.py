#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer
from tf2_ros import TransformException
from tf2_ros import TransformListener


class CalibratedTcpWatcher(Node):
    def __init__(self):
        super().__init__('calibrated_tcp_watcher')
        self.base_frame = self.declare_parameter('base_frame', 'base').value
        self.tcp_frame = self.declare_parameter('tcp_frame', 'calibrated_tcp').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.first_xyz = None
        self.timer = self.create_timer(1.0, self.report_tcp)

    def report_tcp(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}')
            return

        translation = transform.transform.translation
        xyz = (translation.x, translation.y, translation.z)
        if self.first_xyz is None:
            self.first_xyz = xyz

        drift_m = math.dist(xyz, self.first_xyz)
        self.get_logger().info(
            'xyz(m)=[%.6f %.6f %.6f] drift=%.3f mm'
            % (xyz[0], xyz[1], xyz[2], drift_m * 1000.0)
        )


def main():
    rclpy.init()
    node = CalibratedTcpWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
