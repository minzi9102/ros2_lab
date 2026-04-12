from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    action_name_arg = DeclareLaunchArgument(
        "action_name",
        default_value="/scaled_joint_trajectory_controller/follow_joint_trajectory",
        description="FollowJointTrajectory action used for Task 6A.",
    )
    joint_state_topic_arg = DeclareLaunchArgument(
        "joint_state_topic",
        default_value="/joint_states",
        description="JointState topic used to fetch current positions before sending a goal.",
    )

    sender_node = Node(
        package="ur3_minimal_control_lab_cpp",
        executable="joint_trajectory_sender",
        name="ur3_joint_trajectory_sender_cpp",
        output="screen",
        parameters=[
            {
                "action_name": LaunchConfiguration("action_name"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
            }
        ],
    )

    return LaunchDescription(
        [
            action_name_arg,
            joint_state_topic_arg,
            sender_node,
        ]
    )
