from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='/ur3e_named_motion_controller/execute_named_target',
        description='Named motion service used by the GUI.',
    )
    human_confirmation_arg = DeclareLaunchArgument(
        'human_confirmation',
        default_value='',
        description='Confirmation token. Sim mode keeps this empty.',
    )

    gui_node = Node(
        package='ur3e_named_motion_gui_py',
        executable='named_motion_gui',
        name='ur3e_named_motion_gui',
        output='screen',
        parameters=[
            {
                'service_name': LaunchConfiguration('service_name'),
                'human_confirmation': LaunchConfiguration('human_confirmation'),
            }
        ],
    )

    return LaunchDescription(
        [
            service_name_arg,
            human_confirmation_arg,
            gui_node,
        ]
    )
