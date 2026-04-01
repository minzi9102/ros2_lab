import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class ModeServiceServer(Node):
    def __init__(self) -> None:
        super().__init__('mode_service_server')
        self.current_mode = 'MANUAL'
        self.srv = self.create_service(SetBool, '/ur3/set_mode', self.handle_set_mode)
        self.get_logger().info('Service ready: /ur3/set_mode (SetBool)')

    def handle_set_mode(self, request: SetBool.Request, response: SetBool.Response):
        # TODO(human):
        # 1) request.data=True -> AUTO, False -> MANUAL
        # 2) 幂等请求（重复设置同模式）要返回 success=True，但 message 说明“already in target mode”
        # 3) 预留一条失败分支（例如未来安全锁定时 success=False）
        safety_lock_engaged = False  # TODO: replace with real safety lock status
        if safety_lock_engaged:
            response.success = False
            response.message = 'cannot switch mode: safety lock is engaged'
            return response
        target_mode = 'AUTO' if request.data else 'MANUAL'

        # ---- 在这里补你的核心判断 ----
        if target_mode == self.current_mode:
            response.success = True
            response.message = f'already in {self.current_mode}'
            return response

        self.current_mode = target_mode
        response.success = True
        response.message = f'mode switched to {self.current_mode}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ModeServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
