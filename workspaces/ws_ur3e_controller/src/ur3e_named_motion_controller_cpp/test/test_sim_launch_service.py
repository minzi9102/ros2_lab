from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


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
        stdout=subprocess.PIPE,
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
    # TODO(human): 用 rclpy 创建测试节点，等待 SERVICE_NAME 可用，
    # 发送 ExecuteNamedTarget 请求，并返回 response。
    # 这里是 launch + service 测试最关键的练习点：
    # 需要处理 service 等待超时、future 完成超时，以及失败时打印清晰诊断。
    raise NotImplementedError


@pytest.mark.skip(
    reason='TODO(human): 补完 call_execute_named_target 后启用此测试。'
)
def test_ready_plan_only_with_full_moveit_sim():
    with sim_bringup(execute=False):
        time.sleep(1.0)

        response = call_execute_named_target('ready', execute=False)

    assert response.accepted
    assert response.planned
    assert not response.executed
    assert response.status == 'planned'
