#!/usr/bin/env python3

from __future__ import annotations

import signal
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ur_dashboard_msgs.srv import IsInRemoteControl, IsProgramRunning, Load


class ExternalControlManager(Node):
    def __init__(self) -> None:
        super().__init__('task8b_external_control_manager')
        self.declare_parameter('enabled', True)
        self.declare_parameter('program_path', '/programs/external_control.urp')
        self.declare_parameter('require_remote_control', True)
        self.declare_parameter('startup_timeout_sec', 20.0)
        self.declare_parameter('stop_on_shutdown', True)
        self.declare_parameter(
            'remote_control_service', '/dashboard_client/is_in_remote_control'
        )
        self.declare_parameter('program_running_service', '/dashboard_client/program_running')
        self.declare_parameter('load_program_service', '/dashboard_client/load_program')
        self.declare_parameter('play_service', '/dashboard_client/play')
        self.declare_parameter('stop_service', '/dashboard_client/stop')

        self._owns_program = False
        self._shutdown_requested = False
        self._remote_client = self.create_client(
            IsInRemoteControl,
            self._param_string('remote_control_service'),
        )
        self._program_running_client = self.create_client(
            IsProgramRunning,
            self._param_string('program_running_service'),
        )
        self._load_program_client = self.create_client(
            Load,
            self._param_string('load_program_service'),
        )
        self._play_client = self.create_client(Trigger, self._param_string('play_service'))
        self._stop_client = self.create_client(Trigger, self._param_string('stop_service'))

    def request_shutdown(self) -> None:
        self._shutdown_requested = True

    def run(self) -> int:
        if not self.get_parameter('enabled').get_parameter_value().bool_value:
            self.get_logger().info('External Control manager is disabled.')
            return 0

        timeout_sec = self.get_parameter('startup_timeout_sec').get_parameter_value().double_value
        deadline = time.monotonic() + timeout_sec
        self.get_logger().info('Task 8B External Control lifecycle manager started.')

        if not self._wait_for_dashboard_services(deadline):
            return 1

        program_running = self._query_program_running()
        if program_running is None:
            return 1
        if program_running:
            self.get_logger().info(
                'External Control program is already running; this manager will not stop it '
                'on shutdown.'
            )
            return 0

        remote_control = self._query_remote_control()
        if remote_control is None:
            return 1

        require_remote_control = self.get_parameter(
            'require_remote_control'
        ).get_parameter_value().bool_value
        if require_remote_control and not remote_control:
            self.get_logger().error(
                'Teach pendant is not in Remote Control mode. '
                'Please switch to Remote Control mode, then restart Task 8B bringup.'
            )
            return 1

        if not self._load_external_control():
            return 1
        if not self._play_external_control():
            return 1
        if not self._wait_until_program_running(deadline):
            return 1

        self._owns_program = True
        self.get_logger().info(
            'External Control program started by this manager. '
            'It will be stopped automatically when Task 8B bringup shuts down.'
        )
        self._wait_for_shutdown()
        return 0

    def stop_if_owned(self) -> None:
        stop_on_shutdown = self.get_parameter('stop_on_shutdown').get_parameter_value().bool_value
        if not self._owns_program:
            return
        if not stop_on_shutdown:
            self.get_logger().info('stop_on_shutdown=false; leaving External Control running.')
            return

        self.get_logger().warn('Stopping External Control program owned by this bringup.')
        if not self._stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Stop service unavailable during shutdown.')
            return

        response = self._call(self._stop_client, Trigger.Request(), timeout_sec=3.0)
        if response is None:
            self.get_logger().error('Timed out calling stop service during shutdown.')
            return
        if response.success:
            self.get_logger().info(f'External Control stop succeeded: {response.message}')
        else:
            self.get_logger().error(f'External Control stop failed: {response.message}')

    def _wait_for_dashboard_services(self, deadline: float) -> bool:
        clients = [
            (self._remote_client, self._param_string('remote_control_service')),
            (self._program_running_client, self._param_string('program_running_service')),
            (self._load_program_client, self._param_string('load_program_service')),
            (self._play_client, self._param_string('play_service')),
            (self._stop_client, self._param_string('stop_service')),
        ]
        for client, name in clients:
            if not self._wait_for_service(client, name, deadline):
                return False
        return True

    def _wait_for_service(self, client: Any, name: str, deadline: float) -> bool:
        while rclpy.ok() and not self._shutdown_requested and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.5):
                return True
            self.get_logger().info(f'Waiting for dashboard service: {name}')
        self.get_logger().error(f'Dashboard service unavailable before timeout: {name}')
        return False

    def _query_program_running(self) -> bool | None:
        response = self._call(
            self._program_running_client,
            IsProgramRunning.Request(),
            timeout_sec=3.0,
        )
        if response is None or not response.success:
            self.get_logger().error('Failed to query External Control running state.')
            return None
        self.get_logger().info(response.answer)
        return response.program_running

    def _query_remote_control(self) -> bool | None:
        response = self._call(
            self._remote_client,
            IsInRemoteControl.Request(),
            timeout_sec=3.0,
        )
        if response is None or not response.success:
            self.get_logger().error('Failed to query Remote Control state.')
            return None
        self.get_logger().info(f'remote_control={response.remote_control}')
        return response.remote_control

    def _load_external_control(self) -> bool:
        program_path = self.get_parameter('program_path').get_parameter_value().string_value
        request = Load.Request()
        request.filename = program_path
        self.get_logger().info(f'Loading External Control program: {program_path}')
        response = self._call(self._load_program_client, request, timeout_sec=5.0)
        if response is None or not response.success:
            answer = 'no response' if response is None else response.answer
            self.get_logger().error(f'Failed to load External Control program: {answer}')
            return False
        self.get_logger().info(response.answer)
        return True

    def _play_external_control(self) -> bool:
        self.get_logger().info('Starting External Control program.')
        response = self._call(self._play_client, Trigger.Request(), timeout_sec=5.0)
        if response is None or not response.success:
            message = 'no response' if response is None else response.message
            self.get_logger().error(f'Failed to start External Control program: {message}')
            return False
        self.get_logger().info(response.message)
        return True

    def _wait_until_program_running(self, deadline: float) -> bool:
        while rclpy.ok() and not self._shutdown_requested and time.monotonic() < deadline:
            program_running = self._query_program_running()
            if program_running:
                return True
            time.sleep(0.5)
        self.get_logger().error('External Control did not report running before timeout.')
        return False

    def _wait_for_shutdown(self) -> None:
        self.get_logger().info('External Control manager is waiting for bringup shutdown.')
        while rclpy.ok() and not self._shutdown_requested:
            rclpy.spin_once(self, timeout_sec=0.2)

    def _call(self, client: Any, request: Any, timeout_sec: float) -> Any:
        future = client.call_async(request)
        result = rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if result != rclpy.FutureReturnCode.SUCCESS or not future.done():
            return None
        return future.result()

    def _param_string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value


def main() -> int:
    rclpy.init()
    node = ExternalControlManager()

    def handle_signal(_signum: int, _frame: object) -> None:
        node.request_shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    exit_code = 1
    try:
        exit_code = node.run()
    finally:
        node.stop_if_owned()
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
