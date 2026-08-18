import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path)) as config_file:
        return yaml.safe_load(config_file)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='ur3e_handeye_motion',
                executable='handeye_motion_node',
                namespace='handeye_motion',
                output='screen',
                remappings=[
                    ('joint_states', '/joint_states'),
                    ('monitored_planning_scene', '/monitored_planning_scene'),
                    ('robot_description', '/robot_description'),
                    ('robot_description_semantic', '/robot_description_semantic'),
                ],
                parameters=[
                    {
                        'planning_group': 'ur_manipulator',
                        'base_frame': 'base_link',
                        'end_effector_link': 'tool0',
                    },
                    {
                        'robot_description_kinematics': load_yaml(
                            'ur_moveit_config', 'config/kinematics.yaml'
                        )
                    },
                ],
            )
        ]
    )
