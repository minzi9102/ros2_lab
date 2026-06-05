from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ur3e_controller_msgs.srv import ExecuteNamedTarget


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SERVICE_NAME = '/ur3e_named_motion_controller/execute_named_target'


@contextmanager
def sim_bringup(execute: bool):
    command = [
        'ros2',
        'launch',
        'ur3e_named_motion_controller_cpp',
        'sim_named_motion_bringup.launch.py',
        'runtime_mode:=sim',
        f'execute:={str(execute).lower()}',
        'launch_rviz:=false',
    ]
    env = os.environ.copy()
    process = subprocess.Popen(
        command,
        cwd=WORKSPACE_ROOT,
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    try:
        yield process
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=10.0)


def call_execute_named_target(target_name: str, execute: bool):
    rclpy.init()
    node = rclpy.create_node('test_execute_named_target_client')
    try:
        client = node.create_client(ExecuteNamedTarget, SERVICE_NAME)

        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f'Service {SERVICE_NAME} is not available')

        request = ExecuteNamedTarget.Request()
        request.target_name = target_name
        request.execute = execute
        request.human_confirmation = ''

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

        if not future.done():
            raise RuntimeError(f'Service call to {SERVICE_NAME} did not complete in time')
        error = future.exception()
        if error is not None:
            raise RuntimeError(f'Service call to {SERVICE_NAME} failed') from error

        response = future.result()
        if response is None:
            raise RuntimeError(f'Service call to {SERVICE_NAME} failed: {future.exception()}')

        return response
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_ready_plan_only_with_full_moveit_sim():
    with sim_bringup(execute=False):
        time.sleep(1.0)

        response = call_execute_named_target('ready', execute=False)

    assert response.accepted
    assert response.planned
    assert not response.executed
    assert response.status == 'planned'


def test_execute_request_is_rejected_when_launch_execution_disabled():
    with sim_bringup(execute=False):
        time.sleep(1.0)

        response = call_execute_named_target('ready', execute=True)

    assert not response.accepted
    assert not response.planned
    assert not response.executed
    assert response.status == 'rejected_execution_disabled'
