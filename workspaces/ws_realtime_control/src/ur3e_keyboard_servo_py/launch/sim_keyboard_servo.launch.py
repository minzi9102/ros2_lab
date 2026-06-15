import os
import yaml

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def generate_launch_description() -> LaunchDescription:
    log_root_dir = Path.cwd() / 'logs' / 'sim_keyboard_servo'
    run_log_dir = log_root_dir / datetime.now().strftime('%Y%m%d-%H%M%S')
    run_log_dir.mkdir(parents=True, exist_ok=True)

    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur3',
        description='UR robot type used by the realtime keyboard Servo simulation.',
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='Robot or URSim IP address. Fake hardware keeps this for launch compatibility.',
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use fake hardware when true; use URSim/driver path when false.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Optionally launch RViz with MoveIt configuration.',
    )
    servo_log_level_arg = DeclareLaunchArgument(
        'servo_log_level',
        default_value='info',
        description='Log level for moveit_servo/servo_node.',
    )
    joint_states_wait_timeout_arg = DeclareLaunchArgument(
        'joint_states_wait_timeout_sec',
        default_value='15.0',
        description='Maximum time to wait for /joint_states and active controllers.',
    )
    servo_startup_settle_arg = DeclareLaunchArgument(
        'servo_startup_settle_sec',
        default_value='5.0',
        description='Settle time after joint state gate passes before launching Servo.',
    )
    servo_status_wait_timeout_arg = DeclareLaunchArgument(
        'servo_status_wait_timeout_sec',
        default_value='15.0',
        description='Maximum time to wait for /servo_node/status before starting keyboard input.',
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
        .robot_description_semantic(
            Path('srdf') / 'ur.srdf.xacro',
            {'name': LaunchConfiguration('ur_type')},
        )
        .to_moveit_configs()
    )

    servo_yaml = load_yaml('ur_moveit_config', 'config/ur_servo.yaml')
    servo_yaml['joint_topic'] = '/task7e/joint_states_fresh'
    servo_params = {'moveit_servo': servo_yaml}

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'initial_joint_controller': 'forward_position_controller',
            'launch_rviz': 'false',
            'description_launchfile': PathJoinSubstitution(
                [FindPackageShare('ur3_moveit_servo_lab_cpp'), 'launch', 'task7E_ur_rsp.launch.py']
            ),
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('ur_moveit_config'), 'launch', 'ur_moveit.launch.py']
            )
        ),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': 'false',
            'launch_servo': 'false',
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        output='log',
        arguments=[
            '-d',
            PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'config', 'moveit.rviz']),
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': False},
        ],
    )

    joint_state_relay = Node(
        package='ur3_moveit_servo_lab_cpp',
        executable='joint_state_stamp_relay_node',
        name='realtime_joint_state_stamp_relay',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            {
                'source_topic': '/joint_states',
                'target_topic': '/task7e/joint_states_fresh',
                'publish_period_sec': 0.02,
            }
        ],
    )

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            servo_params,
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('servo_log_level')],
    )

    joint_states_gate = Node(
        package='ur3_moveit_servo_lab_cpp',
        executable='wait_for_joint_states.py',
        name='realtime_joint_states_gate',
        output='screen',
        parameters=[
            {
                'topic': '/joint_states',
                'timeout_sec': LaunchConfiguration('joint_states_wait_timeout_sec'),
                'required_active_controllers': [
                    'joint_state_broadcaster',
                    'forward_position_controller',
                ],
            }
        ],
    )

    servo_status_gate = Node(
        package='ur3_moveit_servo_lab_cpp',
        executable='wait_for_servo_status.py',
        name='realtime_servo_status_gate',
        output='screen',
        parameters=[
            {
                'topic': '/servo_node/status',
                'timeout_sec': LaunchConfiguration('servo_status_wait_timeout_sec'),
            }
        ],
    )

    keyboard_node = Node(
        package='ur3e_keyboard_servo_py',
        executable='keyboard_servo_node',
        name='ur3e_keyboard_servo',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('ur3e_keyboard_servo_py'),
                    'config',
                    'sim_keyboard_servo.yaml',
                ]
            )
        ],
    )

    def on_joint_states_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg='Detected /joint_states and active controllers. Starting MoveIt Servo soon.'
                ),
                TimerAction(
                    period=LaunchConfiguration('servo_startup_settle_sec'),
                    actions=[
                        LogInfo(msg='Starting MoveIt Servo node.'),
                        servo_node,
                        LogInfo(msg='Waiting for /servo_node/status before keyboard input.'),
                        servo_status_gate,
                    ],
                ),
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason='Realtime keyboard Servo launch timed out before Servo startup.'
                )
            )
        ]

    def on_servo_status_gate_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg='Detected Servo status traffic. Starting keyboard Servo node.'),
                keyboard_node,
            ]

        return [
            EmitEvent(
                event=Shutdown(
                    reason='Realtime keyboard Servo launch timed out before keyboard startup.'
                )
            )
        ]

    def on_keyboard_node_exit(event, _context):
        return [
            EmitEvent(
                event=Shutdown(
                    reason=f'Keyboard Servo node exited with return code {event.returncode}.'
                )
            )
        ]

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_mock_hardware_arg,
            launch_rviz_arg,
            servo_log_level_arg,
            joint_states_wait_timeout_arg,
            servo_startup_settle_arg,
            servo_status_wait_timeout_arg,
            SetEnvironmentVariable(name='ROS_LOG_DIR', value=str(run_log_dir)),
            LogInfo(msg=f'Realtime keyboard Servo logs will be written to: {run_log_dir}'),
            driver_launch,
            moveit_launch,
            rviz_node,
            joint_state_relay,
            LogInfo(
                msg='Waiting for /joint_states and active controllers before starting MoveIt Servo...'
            ),
            joint_states_gate,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_states_gate,
                    on_exit=on_joint_states_gate_exit,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=servo_status_gate,
                    on_exit=on_servo_status_gate_exit,
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=keyboard_node,
                    on_exit=on_keyboard_node_exit,
                )
            ),
        ]
    )
