from contextlib import contextmanager
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from sensor_msgs.msg import JointState
from ur3e_controller_msgs.srv import ExecuteNamedTarget
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
CATALOG = PACKAGE_ROOT / 'config' / 'ur3e_named_targets.yaml'
SERVICE_NAME = '/ur3e_named_motion_controller/execute_named_target'
TEST_JOINT_STATE_TOPIC = '/test_joint_states'


SIM_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
SIM_HOME_POSITIONS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]


@contextmanager
def temporary_catalog(mutator):
    with CATALOG.open() as stream:
        catalog = yaml.safe_load(stream)
    mutator(catalog)

    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(catalog, stream)
        catalog_path = stream.name

    try:
        yield catalog_path
    finally:
        Path(catalog_path).unlink(missing_ok=True)


@contextmanager
def launch_process(command):
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


@contextmanager
def named_controller_bringup(
    *,
    execute: bool = False,
    target_catalog: str | None = None,
    joint_state_topic: str = TEST_JOINT_STATE_TOPIC,
):
    command = [
        'ros2',
        'launch',
        'ur3e_named_motion_controller_cpp',
        'named_motion_controller.launch.py',
        'runtime_mode:=sim',
        f'execute:={str(execute).lower()}',
        f'joint_state_topic:={joint_state_topic}',
    ]
    if target_catalog is not None:
        command.append(f'target_catalog:={target_catalog}')

    with launch_process(command) as process:
        yield process


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

    with launch_process(command) as process:
        yield process


def make_joint_state(names=None, positions=None):
    message = JointState()
    message.name = names or SIM_JOINT_NAMES
    message.position = positions or SIM_HOME_POSITIONS
    return message


def call_execute_named_target(target_name: str, execute: bool, joint_state=None):
    rclpy.init()
    node = rclpy.create_node('test_execute_named_target_client')
    try:
        client = node.create_client(ExecuteNamedTarget, SERVICE_NAME)
        publisher = None
        if joint_state is not None:
            publisher = node.create_publisher(JointState, TEST_JOINT_STATE_TOPIC, 10)

        if not client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(f'Service {SERVICE_NAME} is not available')

        request = ExecuteNamedTarget.Request()
        request.target_name = target_name
        request.execute = execute
        request.human_confirmation = ''

        future = client.call_async(request)
        deadline = time.monotonic() + 30.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if publisher is not None:
                joint_state.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(joint_state)
            rclpy.spin_once(node, timeout_sec=0.1)

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


def test_ready_execute_with_full_moveit_sim():
    with sim_bringup(execute=True):
        time.sleep(5.0)

        response = call_execute_named_target('ready', execute=True)

    assert response.accepted
    assert response.planned
    assert response.executed
    assert response.status == 'executed'
    assert 'final-target gate passed' in response.message


def test_disabled_target_is_rejected_before_joint_state_gate():
    def disable_ready(catalog):
        catalog['runtime_modes']['sim']['targets']['ready']['enabled'] = False
        catalog['runtime_modes']['sim']['targets']['ready']['reviewed_by'] = 'test disabled'

    with temporary_catalog(disable_ready) as catalog_path:
        with named_controller_bringup(target_catalog=catalog_path):
            response = call_execute_named_target('ready', execute=False)

    assert not response.accepted
    assert not response.planned
    assert not response.executed
    assert response.status == 'rejected_disabled_target'


def test_missing_joint_state_is_rejected():
    with named_controller_bringup():
        response = call_execute_named_target('ready', execute=False)

    assert not response.accepted
    assert not response.planned
    assert not response.executed
    assert response.status == 'rejected_joint_state'
    assert 'timed out waiting for any JointState' in response.message


def test_incomplete_joint_state_is_rejected():
    joint_state = make_joint_state(
        names=SIM_JOINT_NAMES[:-1],
        positions=SIM_HOME_POSITIONS[:-1],
    )

    with named_controller_bringup():
        response = call_execute_named_target('ready', execute=False, joint_state=joint_state)

    assert not response.accepted
    assert not response.planned
    assert not response.executed
    assert response.status == 'rejected_joint_state'
    assert 'JointState is missing required joints' in response.message


def test_delta_gate_rejects_large_target_delta():
    def tighten_delta(catalog):
        sim_config = catalog['runtime_modes']['sim']
        sim_config['max_joint_delta_rad'] = 0.01

    with temporary_catalog(tighten_delta) as catalog_path:
        with named_controller_bringup(target_catalog=catalog_path):
            response = call_execute_named_target(
                'ready',
                execute=False,
                joint_state=make_joint_state(),
            )

    assert not response.accepted
    assert not response.planned
    assert not response.executed
    assert response.status == 'rejected_delta'
    assert 'delta gate failed' in response.message
