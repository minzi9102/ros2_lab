import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name: str, file_path: str):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path) as file:
        return yaml.safe_load(file)


def generate_launch_description() -> LaunchDescription:
    runtime_mode_arg = DeclareLaunchArgument(
        'runtime_mode',
        default_value='sim',
        description='Named controller runtime mode: sim or real.',
    )
    execute_arg = DeclareLaunchArgument(
        'execute',
        default_value='false',
        description='Allow service requests with execute=true. Keep false for plan-only bringup.',
    )
    target_catalog_arg = DeclareLaunchArgument(
        'target_catalog',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('ur3e_named_motion_controller_cpp'),
                'config',
                'ur3e_named_targets.yaml',
            ]
        ),
        description='YAML catalog containing sim/real named joint targets.',
    )
    joint_state_topic_arg = DeclareLaunchArgument(
        'joint_state_topic',
        default_value='/joint_states',
        description='JointState topic used for delta and final gates.',
    )

    kinematics_params = {
        'robot_description_kinematics': load_yaml('ur_moveit_config', 'config/kinematics.yaml'),
    }

    controller_node = Node(
        package='ur3e_named_motion_controller_cpp',
        executable='named_motion_controller_node',
        name='ur3e_named_motion_controller',
        output='screen',
        parameters=[
            {
                'runtime_mode': LaunchConfiguration('runtime_mode'),
                'allow_execution': LaunchConfiguration('execute'),
                'target_catalog': LaunchConfiguration('target_catalog'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
            },
            kinematics_params,
        ],
    )

    return LaunchDescription(
        [
            runtime_mode_arg,
            execute_arg,
            target_catalog_arg,
            joint_state_topic_arg,
            controller_node,
        ]
    )
