from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    demo_mode_arg = DeclareLaunchArgument(
        "demo_mode",
        default_value="both",
        description="Task 7C demo mode: pose, cartesian or both.",
    )
    execute_plan_arg = DeclareLaunchArgument(
        "execute_plan",
        default_value="false",
        description="Execute trajectories when planning succeeds.",
    )

    scene_node = Node(
        package="ur3_moveit_scene_lab_cpp",
        executable="scene_cartesian_demo_node",
        name="ur3_scene_cartesian_demo",
        output="screen",
        parameters=[
            {
                "demo_mode": LaunchConfiguration("demo_mode"),
                "execute_plan": LaunchConfiguration("execute_plan"),
            }
        ],
    )

    return LaunchDescription(
        [
            demo_mode_arg,
            execute_plan_arg,
            scene_node,
        ]
    )
