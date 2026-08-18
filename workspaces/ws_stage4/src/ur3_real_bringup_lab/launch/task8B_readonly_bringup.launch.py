from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, Shutdown
from launch.actions import GroupAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        description="Real robot type, for example ur3 or ur3e.",
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        description="Real robot IP address confirmed by Task 8A.",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Keep RViz off by default during readonly bringup.",
    )
    reverse_ip_arg = DeclareLaunchArgument(
        "reverse_ip",
        default_value="192.168.56.2",
        description="ROS PC IP on the robot network, confirmed by Task 8A.",
    )
    activate_joint_controller_arg = DeclareLaunchArgument(
        "activate_joint_controller",
        default_value="false",
        description="Keep trajectory controllers inactive during Task 8B readonly bringup.",
    )
    launch_dashboard_client_arg = DeclareLaunchArgument(
        "launch_dashboard_client",
        default_value="true",
        description=(
            "Launch dashboard client only after the local hardware ready gate passes."
        ),
    )
    hardware_ready_timeout_arg = DeclareLaunchArgument(
        "hardware_ready_timeout_sec",
        default_value="30.0",
        description="Timeout for waiting controller manager and /joint_states before dashboard startup.",
    )
    dashboard_receive_timeout_arg = DeclareLaunchArgument(
        "dashboard_receive_timeout",
        default_value="20.0",
        description="Timeout passed to the delayed dashboard client.",
    )
    manage_external_control_arg = DeclareLaunchArgument(
        "manage_external_control",
        default_value="true",
        description="After dashboard startup, manage External Control lifecycle.",
    )
    external_control_program_arg = DeclareLaunchArgument(
        "external_control_program",
        default_value="/programs/external_control.urp",
        description="Program path passed to dashboard load_program.",
    )
    require_remote_control_for_external_control_arg = DeclareLaunchArgument(
        "require_remote_control_for_external_control",
        default_value="true",
        description="Fail bringup when External Control management needs Remote Control but it is unavailable.",
    )
    stop_external_control_on_shutdown_arg = DeclareLaunchArgument(
        "stop_external_control_on_shutdown",
        default_value="true",
        description="Stop External Control on shutdown only when this bringup started it.",
    )
    description_launchfile = PathJoinSubstitution(
        [
            FindPackageShare("ur3_real_bringup_lab"),
            "launch",
            "task8B_real_calibrated_rsp.launch.py",
        ]
    )

    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "activate_joint_controller": LaunchConfiguration("activate_joint_controller"),
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "launch_dashboard_client": "false",
            "description_launchfile": description_launchfile,
        }.items(),
    )

    hardware_ready_gate = Node(
        package="ur3_real_bringup_lab",
        executable="wait_for_hardware_ready.py",
        name="task8b_hardware_ready_gate",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_dashboard_client")),
        parameters=[
            {
                "timeout_sec": ParameterValue(
                    LaunchConfiguration("hardware_ready_timeout_sec"),
                    value_type=float,
                ),
                "expected_joint_names": [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint",
                ],
            }
        ],
    )

    dashboard_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "launch",
                    "ur_dashboard_client.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "dashboard_receive_timeout": LaunchConfiguration("dashboard_receive_timeout"),
        }.items(),
    )

    external_control_manager = Node(
        package="ur3_real_bringup_lab",
        executable="manage_external_control.py",
        name="task8b_external_control_manager",
        output="screen",
        condition=IfCondition(LaunchConfiguration("manage_external_control")),
        parameters=[
            {
                "robot_ip": LaunchConfiguration("robot_ip"),
                "program_path": LaunchConfiguration("external_control_program"),
                "require_remote_control": LaunchConfiguration(
                    "require_remote_control_for_external_control"
                ),
                "stop_on_shutdown": LaunchConfiguration("stop_external_control_on_shutdown"),
                "startup_timeout_sec": ParameterValue(
                    LaunchConfiguration("dashboard_receive_timeout"),
                    value_type=float,
                ),
            }
        ],
    )

    def launch_dashboard_after_ready(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Task 8B hardware ready gate passed; launching dashboard client."
                    )
                ),
                dashboard_client_launch,
                external_control_manager,
            ]
        return [
            LogInfo(
                msg=(
                    "Task 8B hardware ready gate failed; dashboard client will not be launched."
                )
            )
        ]

    dashboard_after_ready_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=hardware_ready_gate,
            on_exit=launch_dashboard_after_ready,
        )
    )

    def shutdown_if_external_control_manager_failed(event, _context):
        if event.returncode == 0:
            return []
        return [
            LogInfo(
                msg=(
                    "Task 8B External Control manager failed; shutting down bringup. "
                    "If the teach pendant is not in Remote Control mode, switch modes and restart."
                )
            ),
            Shutdown(reason="Task 8B External Control manager failed"),
        ]

    external_control_manager_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=external_control_manager,
            on_exit=shutdown_if_external_control_manager_failed,
        )
    )

    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            launch_rviz_arg,
            reverse_ip_arg,
            activate_joint_controller_arg,
            launch_dashboard_client_arg,
            hardware_ready_timeout_arg,
            dashboard_receive_timeout_arg,
            manage_external_control_arg,
            external_control_program_arg,
            require_remote_control_for_external_control_arg,
            stop_external_control_on_shutdown_arg,
            GroupAction(actions=[driver_launch], scoped=True),
            hardware_ready_gate,
            dashboard_after_ready_handler,
            external_control_manager_exit_handler,
        ]
    )
