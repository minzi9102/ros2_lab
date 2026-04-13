from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    goal_topic_arg = DeclareLaunchArgument(
        "goal_topic",
        default_value="/goal_pose",
        description="PoseStamped topic used by RViz 2D Goal Pose.",
    )
    execute_plan_arg = DeclareLaunchArgument(
        "execute_plan",
        default_value="true",
        description="Execute automatically after planning succeeds.",
    )

    executor_node = Node(
        package="ur3_moveit_goal_pose_lab_cpp",
        executable="goal_pose_executor_node",
        name="ur3_goal_pose_executor",
        output="screen",
        parameters=[
            {
                "goal_topic": LaunchConfiguration("goal_topic"),
                "execute_plan": LaunchConfiguration("execute_plan"),
            }
        ],
    )

    return LaunchDescription(
        [
            goal_topic_arg,
            execute_plan_arg,
            executor_node,
        ]
    )
