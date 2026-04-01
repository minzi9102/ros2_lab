import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ModeServiceClient(Node):
    def __init__(self) -> None:
        super().__init__('mode_service_client')
        self.cli = self.create_client(SetBool, '/ur3/set_mode')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('waiting for /ur3/set_mode ...')

    def call(self, to_auto: bool):
        req = SetBool.Request()
        req.data = to_auto
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            self.get_logger().error('service call failed or timed out')
            return
        res = future.result()
        self.get_logger().info(f"success={res.success}, message='{res.message}'")


def main(args=None):
    rclpy.init(args=args)
    node = ModeServiceClient()
    node.call(to_auto=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
