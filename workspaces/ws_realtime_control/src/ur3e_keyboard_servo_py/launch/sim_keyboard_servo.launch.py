import os
import yaml

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


SUPPORTED_INPUT_BACKENDS = ('terminal', 'evdev', 'joy')
SUPPORTED_COMMAND_FRAMES = ('base_link', 'tool0')


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def validate_sim_arguments(context: LaunchContext, *_args, **_kwargs):
    input_backend = context.perform_substitution(LaunchConfiguration('input_backend'))
    input_device = context.perform_substitution(LaunchConfiguration('input_device'))
    command_frame = context.perform_substitution(LaunchConfiguration('command_frame'))

    if input_backend not in SUPPORTED_INPUT_BACKENDS:
        return _refuse_launch(
            f'input_backend must be one of {SUPPORTED_INPUT_BACKENDS}, got {input_backend!r}'
        )
    if command_frame not in SUPPORTED_COMMAND_FRAMES:
        return _refuse_launch(
            f'command_frame must be one of {SUPPORTED_COMMAND_FRAMES}, got {command_frame!r}'
        )
    if input_backend == 'evdev':
        if not input_device:
            return _refuse_launch('input_device is required when input_backend=evdev')
        if not os.path.exists(input_device):
            return _refuse_launch(f'evdev input device does not exist: {input_device}')
        if not os.access(input_device, os.R_OK):
            return _refuse_launch(
                f'evdev input device is not readable: {input_device}; '
                'check input group membership'
            )

    for argument_name in (
        'linear_speed_mps',
        'publish_rate_hz',
        'acceleration_mps2',
        'deceleration_mps2',
        'joy_deadzone',
        'joy_autorepeat_rate',
    ):
        raw_value = context.perform_substitution(LaunchConfiguration(argument_name))
        try:
            value = float(raw_value)
        except ValueError:
            return _refuse_launch(
                f'{argument_name} must be a number, got {raw_value!r}'
            )
        if argument_name == 'joy_deadzone':
            if value < 0.0 or value >= 1.0:
                return _refuse_launch(
                    f'{argument_name} must be in [0.0, 1.0), got {value}'
                )
            continue
        if value <= 0.0:
            return _refuse_launch(
                f'{argument_name} must be greater than 0.0, got {value}'
            )
    return [SetLaunchConfiguration(name='sim_args_valid', value='true')]


def set_realtime_launch_joy(context: LaunchContext, *_args, **_kwargs):
    input_backend = context.perform_substitution(LaunchConfiguration('input_backend'))
    return [
        SetLaunchConfiguration(
            name='realtime_launch_joy',
            value='true' if input_backend == 'joy' else 'false',
        )
    ]


def _refuse_launch(reason: str):
    return [
        SetLaunchConfiguration(name='sim_args_valid', value='false'),
        LogInfo(msg=f'Refusing realtime keyboard Servo launch: {reason}'),
        EmitEvent(event=Shutdown(reason='Invalid realtime keyboard Servo argument')),
    ]


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
    input_backend_arg = DeclareLaunchArgument(
        'input_backend',
        default_value='terminal',
        description='Realtime input backend: terminal, evdev, or joy.',
    )
    input_device_arg = DeclareLaunchArgument(
        'input_device',
        default_value='',
        description='Readable /dev/input event device required by the evdev backend.',
    )
    joy_topic_arg = DeclareLaunchArgument(
        'joy_topic',
        default_value='/joy',
        description='sensor_msgs/Joy topic used by the joy input backend.',
    )
    joy_device_id_arg = DeclareLaunchArgument(
        'joy_device_id',
        default_value='0',
        description='Joystick device id passed to joy_node.',
    )
    joy_device_name_arg = DeclareLaunchArgument(
        'joy_device_name',
        default_value='',
        description='Optional joystick device name passed to joy_node.',
    )
    joy_deadzone_arg = DeclareLaunchArgument(
        'joy_deadzone',
        default_value='0.08',
        description='Joystick deadzone used by joy_node and command mapping.',
    )
    joy_autorepeat_rate_arg = DeclareLaunchArgument(
        'joy_autorepeat_rate',
        default_value='100.0',
        description='Joystick autorepeat rate passed to joy_node.',
    )
    command_frame_arg = DeclareLaunchArgument(
        'command_frame',
        default_value='base_link',
        description='Twist command reference frame: base_link or tool0.',
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed_mps',
        default_value='0.20',
        description='Target x/y linear speed for fake hardware testing.',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='100.0',
        description='Twist command publish rate.',
    )
    acceleration_arg = DeclareLaunchArgument(
        'acceleration_mps2',
        default_value='0.50',
        description='Linear acceleration limit for evdev smooth control.',
    )
    deceleration_arg = DeclareLaunchArgument(
        'deceleration_mps2',
        default_value='0.80',
        description='Linear deceleration limit for evdev smooth control.',
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
        condition=IfCondition(LaunchConfiguration('realtime_launch_rviz')),
        output='both',
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
        output='both',
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
        output='both',
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
        output='both',
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
        output='both',
        parameters=[
            {
                'topic': '/servo_node/status',
                'timeout_sec': LaunchConfiguration('servo_status_wait_timeout_sec'),
            }
        ],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='realtime_joy_node',
        output='both',
        condition=IfCondition(LaunchConfiguration('realtime_launch_joy')),
        parameters=[
            {
                'device_id': ParameterValue(
                    LaunchConfiguration('joy_device_id'),
                    value_type=int,
                ),
                'device_name': LaunchConfiguration('joy_device_name'),
                'deadzone': ParameterValue(
                    LaunchConfiguration('joy_deadzone'),
                    value_type=float,
                ),
                'autorepeat_rate': ParameterValue(
                    LaunchConfiguration('joy_autorepeat_rate'),
                    value_type=float,
                ),
                'sticky_buttons': False,
            }
        ],
        remappings=[('/joy', LaunchConfiguration('joy_topic'))],
    )

    keyboard_node = Node(
        package='ur3e_keyboard_servo_py',
        executable='keyboard_servo_node',
        name='ur3e_keyboard_servo',
        output='both',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('ur3e_keyboard_servo_py'),
                    'config',
                    'sim_keyboard_servo.yaml',
                ]
            ),
            {
                'input_backend': LaunchConfiguration('input_backend'),
                'input_device': LaunchConfiguration('input_device'),
                'joy_topic': LaunchConfiguration('joy_topic'),
                'joy_deadzone': ParameterValue(
                    LaunchConfiguration('joy_deadzone'),
                    value_type=float,
                ),
                'frame_id': LaunchConfiguration('command_frame'),
                'linear_speed_mps': ParameterValue(
                    LaunchConfiguration('linear_speed_mps'),
                    value_type=float,
                ),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_rate_hz'),
                    value_type=float,
                ),
                'acceleration_mps2': ParameterValue(
                    LaunchConfiguration('acceleration_mps2'),
                    value_type=float,
                ),
                'deceleration_mps2': ParameterValue(
                    LaunchConfiguration('deceleration_mps2'),
                    value_type=float,
                ),
            },
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
                        LogInfo(msg='Waiting for /servo_node/status before realtime input.'),
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
                LogInfo(msg='Detected Servo status traffic. Starting realtime input node.'),
                joy_node,
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
            input_backend_arg,
            input_device_arg,
            joy_topic_arg,
            joy_device_id_arg,
            joy_device_name_arg,
            joy_deadzone_arg,
            joy_autorepeat_rate_arg,
            command_frame_arg,
            linear_speed_arg,
            publish_rate_arg,
            acceleration_arg,
            deceleration_arg,
            SetLaunchConfiguration(name='sim_args_valid', value='false'),
            OpaqueFunction(function=validate_sim_arguments),
            GroupAction(
                scoped=False,
                condition=IfCondition(LaunchConfiguration('sim_args_valid')),
                actions=[
                    SetEnvironmentVariable(name='ROS_LOG_DIR', value=str(run_log_dir)),
                    SetEnvironmentVariable(
                        name='RCUTILS_LOGGING_BUFFERED_STREAM',
                        value='1',
                    ),
                    SetEnvironmentVariable(
                        name='RCUTILS_LOGGING_USE_STDOUT',
                        value='1',
                    ),
                    SetLaunchConfiguration(
                        name='realtime_launch_rviz',
                        value=LaunchConfiguration('launch_rviz'),
                    ),
                    SetLaunchConfiguration(
                        name='realtime_launch_joy',
                        value='false',
                    ),
                    OpaqueFunction(function=set_realtime_launch_joy),
                    LogInfo(
                        msg=f'Realtime Servo input logs will be written to: {run_log_dir}'
                    ),
                    driver_launch,
                    moveit_launch,
                    rviz_node,
                    joint_state_relay,
                    LogInfo(
                        msg=(
                            'Waiting for /joint_states and active controllers before '
                            'starting MoveIt Servo...'
                        )
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
                ],
            ),
        ]
    )
