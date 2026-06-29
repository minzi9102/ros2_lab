import importlib.util
import math
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_fakehardware_pen_servo.launch.py"
)
URSIM_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_ursim_pen_servo.launch.py"
)
REAL_AIR_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage3_real_air_pen_servo.launch.py"
)
REAL_BENCHMARK_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage3_real_tracking_benchmark.launch.py"
)


def _load_stage2_launch_module():
    spec = importlib.util.spec_from_file_location(
        "stage2_fakehardware_pen_servo_launch",
        LAUNCH_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_stage2_ursim_launch_module():
    spec = importlib.util.spec_from_file_location(
        "stage2_ursim_pen_servo_launch",
        URSIM_LAUNCH_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_stage3_real_air_launch_module():
    spec = importlib.util.spec_from_file_location(
        "stage3_real_air_pen_servo_launch",
        REAL_AIR_LAUNCH_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_stage3_real_benchmark_launch_module():
    spec = importlib.util.spec_from_file_location(
        "stage3_real_tracking_benchmark_launch",
        REAL_BENCHMARK_LAUNCH_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_stage2_launch_sets_servo_rotational_scale_to_ur3_wrist_limit():
    module = _load_stage2_launch_module()

    assert module.STAGE2_SERVO_ROTATIONAL_SCALE_RADPS == math.tau
    servo_yaml = module.configured_stage2_servo_yaml()

    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["rotational"] == math.tau


def test_stage2_ursim_launch_uses_ursim_defaults_and_current_servo_scale():
    module = _load_stage2_ursim_launch_module()

    assert module.STAGE2_URSIM_DEFAULT_ROBOT_IP == "172.17.0.2"
    assert module.STAGE2_URSIM_DEFAULT_USE_MOCK_HARDWARE == "false"
    assert module.STAGE2_SERVO_ROTATIONAL_SCALE_RADPS == math.tau

    servo_yaml = module.configured_stage2_servo_yaml()
    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["rotational"] == math.tau


def test_stage2_ursim_pen_node_parameters_match_fakehardware_height_strategy():
    module = _load_stage2_ursim_launch_module()

    parameters = module.pen_ursim_node_parameters()

    assert parameters["start_from_current_tool0"] is True
    assert parameters["require_motion_before_pose_command"] is True
    assert parameters["paper_origin_xyz"] == [0.45, 0.0, 0.12]
    assert parameters["tool0_to_pen_tip_xyz"] == [0.0, 0.0, 0.14]
    assert parameters["servo_status_topic"] == "/servo_node/status"


def test_stage3_real_air_launch_requires_human_confirmation():
    module = _load_stage3_real_air_launch_module()

    error = module.validate_real_air_configuration(
        human_confirmation="",
        max_session_duration_sec=30.0,
    )

    assert error is not None
    assert module.REQUIRED_REAL_AIR_CONFIRMATION in error


def test_stage3_real_air_launch_rejects_session_duration_over_hard_limit():
    module = _load_stage3_real_air_launch_module()

    error = module.validate_real_air_configuration(
        human_confirmation=module.REQUIRED_REAL_AIR_CONFIRMATION,
        max_session_duration_sec=60.1,
    )

    assert error is not None
    assert "max_session_duration_sec" in error


def test_stage3_real_air_launch_uses_real_robot_driver_defaults():
    module = _load_stage3_real_air_launch_module()

    assert module.STAGE3_REAL_DEFAULT_USE_MOCK_HARDWARE == "false"
    assert module.STAGE3_REAL_INITIAL_JOINT_CONTROLLER == "forward_position_controller"
    assert (
        module.STAGE3_REAL_DESCRIPTION_LAUNCHFILE_NAME
        == "task8B_real_calibrated_rsp.launch.py"
    )


def test_stage3_real_air_launch_uses_current_servo_scale():
    module = _load_stage3_real_air_launch_module()

    servo_yaml = module.configured_stage3_servo_yaml()

    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["rotational"] == math.tau
    assert module.STAGE3_JOINT_STATE_RELAY_PERIOD_SEC == 0.004


def test_stage3_real_air_pen_parameters_match_fakehardware_motion_strategy():
    module = _load_stage3_real_air_launch_module()

    parameters = module.pen_real_air_node_parameters(30.0)

    assert parameters["start_from_current_tool0"] is True
    assert parameters["require_motion_before_pose_command"] is True
    assert parameters["paper_origin_xyz"] == [0.45, 0.0, 0.12]
    assert parameters["tool0_to_pen_tip_xyz"] == [0.0, 0.0, 0.14]
    assert parameters["max_planar_speed_mps"] == 0.03
    assert parameters["max_pen_axis_angular_speed_degps"] == 12.0
    assert parameters["tilt_rate_degps"] == 10.0
    assert parameters["untilt_rate_degps"] == 12.0
    assert parameters["max_session_duration_sec"] == 30.0


def test_stage3_real_benchmark_uses_reviewed_home_and_safe_defaults():
    module = _load_stage3_real_benchmark_launch_module()

    home = module.reviewed_home_parameters()

    assert len(home["home_joint_names"]) == 6
    assert home["home_positions_rad"] == [
        1.537635326385498,
        -2.0233602018370265,
        1.7531832336283040,
        -2.9421216450133265,
        -1.5928295294391077,
        -0.09980899492372686,
    ]
    assert home["home_reviewed_by"] == "用户现场确认"
    assert home["max_velocity_scaling"] == 0.10
    assert home["max_acceleration_scaling"] == 0.10
    assert module.INITIAL_CONTROLLER == "scaled_joint_trajectory_controller"
    assert module.SERVO_CONTROLLER == "forward_position_controller"
    assert module.JOINT_STATE_RELAY_PERIOD_SEC == 0.004
    assert module.STAGE3_SERVO_ROTATIONAL_SCALE_RADPS == math.tau
    assert module.RAW_JOINT_STATES_TOPIC == "/joint_states"
    assert module.FRESH_JOINT_STATES_TOPIC == "/task7e/joint_states_fresh"


def test_stage3_real_benchmark_uses_fresh_joint_states_for_prehome_and_servo():
    module = _load_stage3_real_benchmark_launch_module()

    relay_parameters = module.joint_state_relay_parameters()
    gate_parameters = module.trajectory_gate_parameters(timeout_sec=30.0)
    servo_yaml = module.load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_yaml["joint_topic"] = module.FRESH_JOINT_STATES_TOPIC

    assert relay_parameters["source_topic"] == module.RAW_JOINT_STATES_TOPIC
    assert relay_parameters["target_topic"] == module.FRESH_JOINT_STATES_TOPIC
    assert relay_parameters["publish_period_sec"] == module.JOINT_STATE_RELAY_PERIOD_SEC
    assert gate_parameters["topic"] == module.FRESH_JOINT_STATES_TOPIC
    assert gate_parameters["reliability"] == "best_effort"
    assert gate_parameters["required_active_controllers"] == [
        "joint_state_broadcaster",
        module.INITIAL_CONTROLLER,
    ]
    assert servo_yaml["joint_topic"] == module.FRESH_JOINT_STATES_TOPIC


def test_stage3_real_benchmark_move_group_uses_explicit_warehouse_config():
    module = _load_stage3_real_benchmark_launch_module()

    parameters = module.move_group_parameters(moveit_config={})

    assert parameters[1]["warehouse_plugin"] == "warehouse_ros_sqlite::DatabaseConnection"
    assert parameters[1]["warehouse_host"].endswith("/.ros/warehouse_ros.sqlite")
    assert parameters[2]["use_sim_time"] is False
    assert parameters[2]["publish_robot_description_semantic"] is True


def test_stage3_real_benchmark_starts_relay_before_prehome_only_once():
    source = REAL_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")

    hardware_start = source.index("def on_hardware_exit")
    servo_start = source.index("def on_switch_to_servo_exit")
    hardware_block = source[hardware_start:servo_start]
    servo_block = source[servo_start : source.index("def on_servo_gate_exit")]

    assert "joint_state_relay" in hardware_block
    assert "joint_state_relay" not in servo_block
    assert "ur_moveit.launch.py" not in source
    assert "move_group_node = Node(" in source
    assert "remappings=[(RAW_JOINT_STATES_TOPIC, FRESH_JOINT_STATES_TOPIC)]" in source


def test_stage3_real_benchmark_uses_runtime_rviz_launch_config():
    source = REAL_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'RVIZ_LAUNCH_CONFIG = "real_benchmark_launch_rviz"' in source
    assert "SetLaunchConfiguration(" in source
    assert 'value=LaunchConfiguration("launch_rviz")' in source
    assert "condition=IfCondition(LaunchConfiguration(RVIZ_LAUNCH_CONFIG))" in source


def test_stage3_real_benchmark_requires_confirmation_and_bounded_session():
    module = _load_stage3_real_benchmark_launch_module()

    assert module.validate_benchmark_configuration(
        human_confirmation="",
        max_session_duration_sec=60.0,
    )
    assert module.validate_benchmark_configuration(
        human_confirmation=module.REQUIRED_CONFIRMATION,
        max_session_duration_sec=60.1,
    )
    assert (
        module.validate_benchmark_configuration(
            human_confirmation=module.REQUIRED_CONFIRMATION,
            max_session_duration_sec=60.0,
        )
        is None
    )
