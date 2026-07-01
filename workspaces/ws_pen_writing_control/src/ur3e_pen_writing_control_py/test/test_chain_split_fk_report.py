import json
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ur3e_pen_writing_control_py.chain_split_fk_report import (
    FALLBACK_TARGET_TOPIC,
    PRIMARY_TARGET_TOPIC,
    build_report,
    main,
)
from ur3e_pen_writing_control_py.command_latency_report import (
    FORWARD_CONTROLLER_JOINTS,
)


URDF_TEXT = """\
<robot name="chain_split_test">
  <link name="base_link"/>
  <link name="link_1"/>
  <link name="link_2"/>
  <link name="link_3"/>
  <link name="link_4"/>
  <link name="link_5"/>
  <link name="link_6"/>
  <link name="tool0"/>
  <joint name="shoulder_pan_joint" type="prismatic">
    <parent link="base_link"/>
    <child link="link_1"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="shoulder_lift_joint" type="prismatic">
    <parent link="link_1"/>
    <child link="link_2"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="elbow_joint" type="prismatic">
    <parent link="link_2"/>
    <child link="link_3"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="wrist_1_joint" type="continuous">
    <parent link="link_3"/>
    <child link="link_4"/>
    <axis xyz="1 0 0"/>
  </joint>
  <joint name="wrist_2_joint" type="continuous">
    <parent link="link_4"/>
    <child link="link_5"/>
    <axis xyz="0 1 0"/>
  </joint>
  <joint name="wrist_3_joint" type="continuous">
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="tool0_fixed_joint" type="fixed">
    <parent link="link_6"/>
    <child link="tool0"/>
  </joint>
</robot>
"""


def _write_pose(topic: str, position: tuple[float, float, float], timestamp_ns: int):
    msg = PoseStamped()
    msg.pose.position.x = position[0]
    msg.pose.position.y = position[1]
    msg.pose.position.z = position[2]
    msg.pose.orientation.w = 1.0
    return topic, serialize_message(msg), timestamp_ns


def _write_command(values: tuple[float, ...], timestamp_ns: int):
    msg = Float64MultiArray()
    msg.data = list(values)
    return (
        "/forward_position_controller/commands",
        serialize_message(msg),
        timestamp_ns,
    )


def _write_state(values: tuple[float, ...], timestamp_ns: int):
    msg = JointState()
    msg.name = list(FORWARD_CONTROLLER_JOINTS)
    msg.position = list(values)
    return "/joint_states", serialize_message(msg), timestamp_ns


def _write_bag(
    bag_path: Path,
    *,
    target_topic: str,
) -> tuple[Path, Path]:
    import rosbag2_py

    summary_json = bag_path.parent / "tracking_summary.json"
    summary_json.write_text('{"status": "ok"}\n', encoding="utf-8")
    urdf_path = bag_path.parent / "chain_split_test.urdf"
    urdf_path.write_text(URDF_TEXT, encoding="utf-8")

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    def topic_metadata(name: str, topic_type: str):
        return rosbag2_py.TopicMetadata(0, name, topic_type, "cdr", [])

    writer.create_topic(
        topic_metadata(target_topic, "geometry_msgs/msg/PoseStamped")
    )
    writer.create_topic(
        topic_metadata(
            "/forward_position_controller/commands",
            "std_msgs/msg/Float64MultiArray",
        )
    )
    writer.create_topic(
        topic_metadata("/joint_states", "sensor_msgs/msg/JointState")
    )

    samples = [
        _write_pose(target_topic, (0.00, 0.00, 0.00), 1_000_000_000),
        _write_command((0.00, 0.00, 0.00, 0.00, 0.00, 0.00), 1_020_000_000),
        _write_state((0.00, 0.00, 0.00, 0.00, 0.00, 0.00), 1_024_000_000),
        _write_pose(target_topic, (0.05, -0.02, 0.01), 1_030_000_000),
        _write_command((0.05, -0.02, 0.01, 0.00, 0.00, 0.00), 1_040_000_000),
        _write_state((0.05, -0.02, 0.01, 0.00, 0.00, 0.00), 1_044_000_000),
    ]
    for topic, payload, timestamp_ns in samples:
        writer.write(topic, payload, timestamp_ns)
    return summary_json, urdf_path


def test_chain_split_fk_report_cli_reads_primary_target_topic_and_writes_outputs(
    tmp_path,
):
    bag_path = tmp_path / "bag"
    summary_json, urdf_path = _write_bag(
        bag_path,
        target_topic=PRIMARY_TARGET_TOPIC,
    )
    output_json = tmp_path / "chain_split_fk_report.json"
    output_md = tmp_path / "chain_split_fk_report.md"

    main(
        [
            "--bag",
            str(bag_path),
            "--summary-json",
            str(summary_json),
            "--urdf-xacro",
            str(urdf_path),
            "--ur-type",
            "ur3",
            "--output-json",
            str(output_json),
            "--output-md",
            str(output_md),
        ]
    )

    report = json.loads(output_json.read_text(encoding="utf-8"))

    assert output_md.exists()
    assert report["topics"]["target_pose"] == PRIMARY_TARGET_TOPIC
    assert report["sample_counts"]["target_pose"] == 2
    assert set(report["plots"]) == {
        "target_vs_commanded_fk_xy",
        "target_vs_commanded_fk_time",
        "commanded_vs_actual_fk_error",
    }
    for plot_path in report["plots"].values():
        assert Path(plot_path).exists()


def test_chain_split_fk_report_falls_back_to_servo_pose_target_topic(tmp_path):
    bag_path = tmp_path / "bag"
    summary_json, urdf_path = _write_bag(
        bag_path,
        target_topic=FALLBACK_TARGET_TOPIC,
    )

    report, _, _ = build_report(
        bag_path=bag_path,
        summary_json_path=summary_json,
        urdf_xacro=urdf_path,
        ur_type="ur3",
    )

    assert report["topics"]["target_pose"] == FALLBACK_TARGET_TOPIC
    assert report["sample_counts"]["target_to_command_fk_matches"] == 2
    assert "target_pose_to_commanded_joint_fk" in report
    assert "shape_check_target_vs_commanded_fk" in report
    assert "commanded_joints_to_actual_joints" in report
