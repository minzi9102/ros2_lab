from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    joint_state_topic_arg = DeclareLaunchArgument(
        "joint_state_topic",
        default_value="/joint_states",
        description="JointState topic published by UR mock hardware.",
    )
    action_name_arg = DeclareLaunchArgument(
        "action_name",
        default_value="/scaled_joint_trajectory_controller/follow_joint_trajectory",
        description="FollowJointTrajectory action used in Task 5C.",
    )
    observer_rate_hz_arg = DeclareLaunchArgument(
        "observer_rate_hz",
        default_value="1.0",
        description="Print frequency for the joint state observer.",
    )
    run_sender_arg = DeclareLaunchArgument(
        "run_sender",
        default_value="false",
        description="Whether to start the trajectory sender together with the observer.",
    )

    observer_node = Node(
        package="ur3_minimal_control_lab_py",
        executable="joint_state_observer",
        name="ur3_joint_state_observer",
        output="screen",
        parameters=[
            {
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "print_rate_hz": LaunchConfiguration("observer_rate_hz"),
                "expected_joints": joint_names,
            }
        ],
    )

    sender_node = Node(
        package="ur3_minimal_control_lab_py",
        executable="joint_trajectory_sender",
        name="ur3_joint_trajectory_sender",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_sender")),
        parameters=[
            {
                "action_name": LaunchConfiguration("action_name"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "server_wait_timeout_sec": 10.0,
                "start_delay_sec": 1.0,
                "joint_names": joint_names,
            }
        ],
    )

    return LaunchDescription(
        [
            joint_state_topic_arg,
            action_name_arg,
            observer_rate_hz_arg,
            run_sender_arg,
            observer_node,
            sender_node,
        ]
    )
