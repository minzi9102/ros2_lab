from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    topic_name = LaunchConfiguration("topic_name")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    payload_size_bytes = LaunchConfiguration("payload_size_bytes")

    pub_reliability = LaunchConfiguration("pub_reliability")
    pub_history = LaunchConfiguration("pub_history")
    pub_depth = LaunchConfiguration("pub_depth")

    sub_reliability = LaunchConfiguration("sub_reliability")
    sub_history = LaunchConfiguration("sub_history")
    sub_depth = LaunchConfiguration("sub_depth")

    report_period_sec = LaunchConfiguration("report_period_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("topic_name", default_value="/ur3/qos_lab"),
            DeclareLaunchArgument("publish_rate_hz", default_value="500.0"),
            DeclareLaunchArgument("payload_size_bytes", default_value="64"),
            DeclareLaunchArgument("pub_reliability", default_value="reliable"),
            DeclareLaunchArgument("pub_history", default_value="keep_last"),
            DeclareLaunchArgument("pub_depth", default_value="10"),
            DeclareLaunchArgument("sub_reliability", default_value="reliable"),
            DeclareLaunchArgument("sub_history", default_value="keep_last"),
            DeclareLaunchArgument("sub_depth", default_value="10"),
            DeclareLaunchArgument("report_period_sec", default_value="1.0"),
            Node(
                package="ur3_qos_lab_py",
                executable="qos_publisher_node",
                name="qos_publisher_node",
                output="screen",
                parameters=[
                    {
                        "topic_name": topic_name,
                        "publish_rate_hz": publish_rate_hz,
                        "payload_size_bytes": payload_size_bytes,
                        "reliability": pub_reliability,
                        "history": pub_history,
                        "depth": pub_depth,
                        "report_period_sec": report_period_sec,
                    }
                ],
            ),
            Node(
                package="ur3_qos_lab_py",
                executable="qos_subscriber_node",
                name="qos_subscriber_node",
                output="screen",
                parameters=[
                    {
                        "topic_name": topic_name,
                        "reliability": sub_reliability,
                        "history": sub_history,
                        "depth": sub_depth,
                        "report_period_sec": report_period_sec,
                    }
                ],
            ),
        ]
    )
