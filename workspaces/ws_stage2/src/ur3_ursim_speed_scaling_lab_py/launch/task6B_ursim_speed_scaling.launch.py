from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("ur3_ursim_speed_scaling_lab_py"),
            "config",
            "task6B_experiment.yaml",
        ]
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=config_file,
        description="Parameter file for Task 6B experiment nodes.",
    )
    speed_scaling_topic_arg = DeclareLaunchArgument(
        "speed_scaling_topic",
        default_value="/speed_scaling_state_broadcaster/speed_scaling",
        description="Speed scaling topic published by the UR broadcaster.",
    )
    joint_state_topic_arg = DeclareLaunchArgument(
        "joint_state_topic",
        default_value="/joint_states",
        description="JointState topic used by the trajectory runner.",
    )
    action_name_arg = DeclareLaunchArgument(
        "action_name",
        default_value="/scaled_joint_trajectory_controller/follow_joint_trajectory",
        description="FollowJointTrajectory action used in Task 6B.",
    )
    monitor_rate_hz_arg = DeclareLaunchArgument(
        "monitor_rate_hz",
        default_value="1.0",
        description="Heartbeat print frequency for the speed scaling monitor.",
    )
    run_runner_arg = DeclareLaunchArgument(
        "run_runner",
        default_value="false",
        description="Whether to launch the trajectory runner together with the monitor.",
    )

    monitor_node = Node(
        package="ur3_ursim_speed_scaling_lab_py",
        executable="speed_scaling_monitor",
        name="ur3_speed_scaling_monitor",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "speed_scaling_topic": LaunchConfiguration("speed_scaling_topic"),
                "print_rate_hz": LaunchConfiguration("monitor_rate_hz"),
            },
        ],
    )

    runner_node = Node(
        package="ur3_ursim_speed_scaling_lab_py",
        executable="scaled_trajectory_runner",
        name="ur3_scaled_trajectory_runner",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_runner")),
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "speed_scaling_topic": LaunchConfiguration("speed_scaling_topic"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "action_name": LaunchConfiguration("action_name"),
            },
        ],
    )

    return LaunchDescription(
        [
            config_file_arg,
            speed_scaling_topic_arg,
            joint_state_topic_arg,
            action_name_arg,
            monitor_rate_hz_arg,
            run_runner_arg,
            monitor_node,
            runner_node,
        ]
    )
