from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
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


def generate_launch_description() -> LaunchDescription:
    run_dir = (
        Path.cwd()
        / "logs"
        / "stage2_ursim_speedl_benchmark"
        / datetime.now().strftime("%Y%m%d-%H%M%S")
    )
    run_dir.mkdir(parents=True, exist_ok=True)

    arguments = [
        DeclareLaunchArgument("robot_ip", default_value="172.17.0.2"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("speed_mps", default_value="0.06"),
        DeclareLaunchArgument("acceleration_mps2", default_value="0.2"),
        DeclareLaunchArgument("motion_sec", default_value="5.0"),
        DeclareLaunchArgument("paper_width_m", default_value="0.60"),
        DeclareLaunchArgument("paper_height_m", default_value="0.16"),
        DeclareLaunchArgument("initial_tip_x_m", default_value="-0.24"),
        DeclareLaunchArgument("run_log_dir", default_value=str(run_dir)),
    ]

    stop_external_control = ExecuteProcess(
        cmd=[
            "python3",
            "-c",
            (
                "import socket, sys\n"
                "sock = socket.create_connection((sys.argv[1], 29999), timeout=3)\n"
                "sock.settimeout(3)\n"
                "print(sock.recv(4096).decode(errors='replace').strip(), flush=True)\n"
                "sock.sendall(b'stop\\n')\n"
                "print(sock.recv(4096).decode(errors='replace').strip(), flush=True)\n"
                "sock.close()\n"
            ),
            LaunchConfiguration("robot_ip"),
        ],
        output="screen",
    )

    driver = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("ur_robot_driver"),
                            "launch",
                            "ur_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "ur_type": "ur3",
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "use_mock_hardware": "false",
                    "activate_joint_controller": "false",
                    "initial_joint_controller": "joint_trajectory_controller",
                    "launch_rviz": "false",
                    "headless_mode": "false",
                    "description_launchfile": PathJoinSubstitution(
                        [
                            FindPackageShare("ur3_moveit_servo_lab_cpp"),
                            "launch",
                            "task7E_ur_rsp.launch.py",
                        ]
                    ),
                }.items(),
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_speedl_benchmark",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("ur3e_pen_writing_control_py"),
                    "rviz",
                    "stage2_fakehardware_pen_servo.rviz",
                ]
            ),
        ],
    )

    benchmark = Node(
        package="ur3e_pen_writing_control_py",
        executable="speedl_benchmark_node",
        name="stage2_ursim_speedl_benchmark",
        output="screen",
        parameters=[
            {
                "speed_mps": LaunchConfiguration("speed_mps"),
                "acceleration_mps2": LaunchConfiguration("acceleration_mps2"),
                "motion_sec": LaunchConfiguration("motion_sec"),
                "paper_width_m": LaunchConfiguration("paper_width_m"),
                "paper_height_m": LaunchConfiguration("paper_height_m"),
                "initial_tip_x_m": LaunchConfiguration("initial_tip_x_m"),
                "output_dir": LaunchConfiguration("run_log_dir"),
            }
        ],
    )

    start_after_stop = RegisterEventHandler(
        OnProcessExit(
            target_action=stop_external_control,
            on_exit=[rviz, driver, TimerAction(period=8.0, actions=[benchmark])],
        )
    )
    stop_launch_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=benchmark,
            on_exit=[
                EmitEvent(event=Shutdown(reason="SpeedL benchmark completed"))
            ],
        )
    )

    return LaunchDescription(
        [
            *arguments,
            SetEnvironmentVariable(
                "ROS_LOG_DIR", LaunchConfiguration("run_log_dir")
            ),
            start_after_stop,
            stop_launch_when_done,
            stop_external_control,
        ]
    )
