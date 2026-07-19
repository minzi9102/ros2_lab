import importlib.util
import math
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "real_force_pen_writing.launch.py"
)
NODE_PATH = (
    Path(__file__).resolve().parents[1]
    / "ur3e_force_pen_writing_py"
    / "z_compliance_validation_node.py"
)
SETUP_PATH = Path(__file__).resolve().parents[1] / "setup.py"
PACKAGE_PATH = Path(__file__).resolve().parents[1] / "package.xml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "real_force_pen_writing_launch",
        LAUNCH_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _valid_configuration(module):
    return {
        "human_confirmation": module.REQUIRED_Z_COMPLIANCE_CONFIRMATION,
        "payload_mass_kg": 0.085,
        "target_force_n": 0.8,
        "direction_force_n": 0.2,
        "max_force_filtered_n": 1.5,
        "max_force_raw_n": 2.0,
        "max_z_speed_mps": 0.0005,
        "damping_factor": 0.5,
        "gain_scaling": 0.3,
        "max_acquire_travel_m": 0.004,
        "max_contact_z_offset_m": 0.0015,
        "max_xy_error_m": 0.003,
        "max_rotation_error_rad": math.radians(2.0),
        "max_pen_tilt_rad": math.radians(1.0),
        "steady_force_min_n": 0.5,
        "steady_force_max_n": 1.1,
        "lost_contact_force_n": 0.2,
        "lost_contact_duration_sec": 0.3,
        "retract_distance_m": 0.003,
        "contact_clearance_m": 0.002,
        "line_length_m": 0.01,
        "line_speed_mps": 0.003,
        "air_speed_mps": 0.005,
        "max_air_path_length_m": 0.2,
        "max_contact_stroke_length_m": 0.075,
        "max_contact_total_length_m": 0.12,
        "max_contact_execution_distance_m": 0.2,
        "max_contact_stroke_count": 12,
        "max_contact_run_sec": 180.0,
        "cartesian_step_m": 0.0005,
        "writing_width_m": 0.01,
        "writing_height_m": 0.01,
        "path_simplify_tolerance_m": 0.00025,
        "data_timeout_sec": 0.2,
        "baseline_duration_sec": 1.0,
        "baseline_settle_sec": 0.5,
        "max_baseline_stddev_n": 0.1,
        "contact_settle_sec": 1.0,
        "hold_duration_sec": 5.0,
        "air_hold_duration_sec": 2.0,
    }


def test_z_compliance_launch_requires_independent_real_robot_confirmation():
    module = _load_launch_module()
    values = _valid_configuration(module)
    values["human_confirmation"] = ""

    error = module.validate_z_compliance_configuration(values)

    assert error is not None
    assert module.REQUIRED_Z_COMPLIANCE_CONFIRMATION in error


def test_z_compliance_launch_uses_calibrated_tool_and_payload_defaults():
    module = _load_launch_module()

    assert module.DEFAULT_TOOL0_TO_PEN_TIP_XYZ == [
        0.00079,
        -0.00076,
        0.15172,
    ]
    assert module.DEFAULT_PAYLOAD_MASS_KG == 0.085
    assert module.DEFAULT_PAYLOAD_COG_XYZ == [0.0, 0.0, 0.0]
    assert module.DEFAULT_PEN_AXIS_TOOL_XYZ == [0.0, 0.0, 1.0]


def test_z_compliance_launch_exposes_bounded_safety_parameters():
    module = _load_launch_module()
    values = _valid_configuration(module)

    assert module.validate_z_compliance_configuration(values) is None

    values["max_z_speed_mps"] = 0.00051
    assert "max_z_speed_mps" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["target_force_n"] = 1.01
    assert "target_force_n" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["steady_force_max_n"] = 1.6
    assert "force thresholds" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["writing_width_m"] = 0.030001
    assert "writing_width_m" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["line_speed_mps"] = 0.0041
    assert "line_speed_mps" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["air_speed_mps"] = 0.0101
    assert "air_speed_mps" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["contact_clearance_m"] = 0.0031
    assert "contact_clearance_m" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["retract_distance_m"] = 0.0015
    assert "must not exceed" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["max_pen_tilt_rad"] = math.radians(2.01)
    assert "max_pen_tilt_rad" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["max_contact_total_length_m"] = 0.120001
    assert "max_contact_total_length_m" in module.validate_z_compliance_configuration(values)
    values = _valid_configuration(module)
    values["max_contact_stroke_count"] = 13
    assert "max_contact_stroke_count" in module.validate_z_compliance_configuration(values)


def test_z_compliance_launch_reuses_seek_bringup_without_joy_or_rviz():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    bringup_source = (
        LAUNCH_PATH.parent / "real_force_writing_bringup.launch.py"
    ).read_text(encoding="utf-8")

    assert "real_force_writing_bringup.launch.py" in source
    assert '"launch_rviz": "false"' in source
    assert '"launch_paper_seek": "true"' in source
    assert 'executable="paper_seek_servo_node"' in bringup_source
    assert "joy_node" not in bringup_source
    assert "pen_fakehardware_servo_node" not in bringup_source
    assert "z_compliance_validation_node" in source
    assert "I_CONFIRM_REAL_Z_COMPLIANCE_TEST" in source


def test_z_compliance_launch_does_not_start_motion_automatically():
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert "/pen_writing/z_compliance/start_" in source
    assert "No motion starts" in source
    assert "ExecuteProcess" not in source


def test_z_compliance_launch_uses_independent_session_log_directory():
    source = LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"force_pen_writing"' in source
    assert 'DeclareLaunchArgument("log_directory", default_value="")' in source


def test_z_compliance_launch_matches_node_safety_parameter_interface():
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    node_source = NODE_PATH.read_text(encoding="utf-8")

    for name in (
        "target_force_n",
        "direction_force_n",
        "max_force_filtered_n",
        "max_force_raw_n",
        "max_z_speed_mps",
        "damping_factor",
        "gain_scaling",
        "max_acquire_travel_m",
        "max_contact_z_offset_m",
        "max_xy_error_m",
        "max_rotation_error_rad",
        "max_pen_tilt_rad",
        "steady_force_min_n",
        "steady_force_max_n",
        "lost_contact_force_n",
        "lost_contact_duration_sec",
        "retract_distance_m",
        "contact_clearance_m",
        "line_length_m",
        "line_speed_mps",
        "air_speed_mps",
        "max_air_path_length_m",
        "max_contact_stroke_length_m",
        "max_contact_total_length_m",
        "max_contact_execution_distance_m",
        "max_contact_stroke_count",
        "max_contact_run_sec",
        "cartesian_step_m",
        "writing_width_m",
        "writing_height_m",
        "path_simplify_tolerance_m",
        "data_timeout_sec",
        "baseline_duration_sec",
        "baseline_settle_sec",
        "max_baseline_stddev_n",
        "contact_settle_sec",
        "hold_duration_sec",
        "air_hold_duration_sec",
        "pen_axis_tool_xyz",
    ):
        assert f'"{name}"' in node_source
        assert launch_source.count(f'"{name}"') >= 2


def test_z_compliance_executable_and_ros_message_dependencies_are_packaged():
    setup_source = SETUP_PATH.read_text(encoding="utf-8")
    package_source = PACKAGE_PATH.read_text(encoding="utf-8")

    assert "z_compliance_validation_node:main" in setup_source
    for dependency in ("action_msgs", "control_msgs", "trajectory_msgs"):
        assert f"<depend>{dependency}</depend>" in package_source


def test_force_mode_launch_uses_new_package_and_confirmation():
    source = (LAUNCH_PATH.parent / "real_force_mode_validation.launch.py").read_text(
        encoding="utf-8"
    )

    assert "I_CONFIRM_REAL_FORCE_MODE_TEST" in source
    assert 'package="ur3e_force_pen_writing_py"' in source
    assert '"launch_paper_seek": "false"' in source
    old_package = "ur3e_" + "pen_writing_control_py"
    assert old_package not in source


def test_new_workspace_has_no_runtime_dependency_on_old_pen_package():
    package_root = Path(__file__).resolve().parents[1]
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in package_root.rglob("*")
        if path.is_file() and path.suffix in {".py", ".xml"}
    )

    old_package = "ur3e_" + "pen_writing_control_py"
    assert old_package not in sources
