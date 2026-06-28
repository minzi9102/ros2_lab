import json
import sys
from pathlib import Path

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
import rclpy
from rclpy.node import Node


def controller_states_match(
    states: dict[str, str],
    *,
    activate: list[str],
    deactivate: list[str],
) -> bool:
    return all(states.get(name) == "active" for name in activate) and all(
        states.get(name) == "inactive" for name in deactivate
    )


class ControllerSwitchOnceNode(Node):
    def __init__(self) -> None:
        super().__init__("controller_switch_once")
        self.activate = list(
            self.declare_parameter("activate_controllers", []).value
        )
        self.deactivate = list(
            self.declare_parameter("deactivate_controllers", []).value
        )
        self.timeout_sec = float(
            self.declare_parameter("timeout_sec", 10.0).value
        )
        self.result_path = Path(
            str(self.declare_parameter("result_path", "").value)
        )
        self._switch_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
        )
        self._list_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )

    def run(self) -> int:
        if not self.activate or self.timeout_sec <= 0.0:
            return self._finish(False, "invalid switch parameters")
        if not self._switch_client.wait_for_service(timeout_sec=self.timeout_sec):
            return self._finish(False, "switch_controller service unavailable")

        request = SwitchController.Request()
        request.activate_controllers = self.activate
        request.deactivate_controllers = self.deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(
            sec=int(self.timeout_sec),
            nanosec=int((self.timeout_sec % 1.0) * 1e9),
        )
        future = self._switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        response = future.result()
        if response is None or not response.ok:
            message = "controller switch failed"
            if response is not None and response.message:
                message += f": {response.message}"
            return self._finish(False, message)

        if not self._list_client.wait_for_service(timeout_sec=self.timeout_sec):
            return self._finish(False, "list_controllers service unavailable")
        list_future = self._list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(
            self,
            list_future,
            timeout_sec=self.timeout_sec,
        )
        list_response = list_future.result()
        if list_response is None:
            return self._finish(False, "controller state verification timed out")
        states = {
            controller.name: controller.state
            for controller in list_response.controller
        }
        if not controller_states_match(
            states,
            activate=self.activate,
            deactivate=self.deactivate,
        ):
            return self._finish(False, f"unexpected controller states: {states}")
        return self._finish(True, "controller switch verified")

    def _finish(self, success: bool, reason: str) -> int:
        level = self.get_logger().info if success else self.get_logger().error
        level(
            f"Controller switch result: success={success} "
            f"activate={self.activate} deactivate={self.deactivate} reason={reason}"
        )
        if str(self.result_path):
            self.result_path.parent.mkdir(parents=True, exist_ok=True)
            content = json.dumps(
                {
                    "success": success,
                    "activate": self.activate,
                    "deactivate": self.deactivate,
                    "reason": reason,
                },
                indent=2,
            )
            content += "\n"
            self.result_path.write_text(
                content,
                encoding="utf-8",
            )
        return 0 if success else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerSwitchOnceNode()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
