from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    source_frame = LaunchConfiguration("source_frame")
    target_frame = LaunchConfiguration("target_frame")
    query_rate_hz = LaunchConfiguration("query_rate_hz")
    timeout_sec = LaunchConfiguration("timeout_sec")
    query_time_offset_sec = LaunchConfiguration("query_time_offset_sec")
    use_sim_time = LaunchConfiguration("use_sim_time")
    tf_topic = LaunchConfiguration("tf_topic")
    tf_static_topic = LaunchConfiguration("tf_static_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("source_frame", default_value="tool0"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("query_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("query_time_offset_sec", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("tf_topic", default_value="/tf_demo"),
            DeclareLaunchArgument("tf_static_topic", default_value="/tf_static_demo"),
            Node(
                package="ur3_tf_lookup_py",
                executable="tf_lookup_node",
                name="ur3_tf_lookup_node",
                output="screen",
                parameters=[
                    {
                        "source_frame": source_frame,
                        "target_frame": target_frame,
                        "query_rate_hz": query_rate_hz,
                        "timeout_sec": timeout_sec,
                        "query_time_offset_sec": query_time_offset_sec,
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[
                    ("/tf", tf_topic),
                    ("/tf_static", tf_static_topic),
                ],
            ),
        ]
    )
