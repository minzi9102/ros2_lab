import importlib.util
import math
from pathlib import Path

from launch import LaunchContext
from launch.utilities import perform_substitutions


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_fakehardware_pen_servo.launch.py"
)
FAKE_BENCHMARK_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_fakehardware_tracking_benchmark.launch.py"
)
FAKE_CONSTANT_TWIST_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_fakehardware_constant_twist_diagnostic.launch.py"
)
URSIM_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_ursim_pen_servo.launch.py"
)
URSIM_BENCHMARK_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_ursim_tracking_benchmark.launch.py"
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
REAL_FORCE_MODE_LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage3_real_force_mode_validation.launch.py"
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
    runtime_parameters = dict(module.SERVO_RUNTIME_PARAMETERS)
    assert runtime_parameters["butterworth_filter_coeff"] == "Double value"
    assert (
        runtime_parameters["moveit_servo.low_pass_filter_coeff"]
        == "Parameter not set"
    )
    servo_yaml = module.configured_stage2_servo_yaml()

    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["rotational"] == math.tau
    assert servo_yaml["use_smoothing"] is True
    assert module.configured_stage2_servo_yaml(use_smoothing=False)[
        "use_smoothing"
    ] is False


def test_stage2_fakehardware_twist_and_relay_parameters_are_configurable():
    module = _load_stage2_launch_module()
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    benchmark_source = FAKE_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")

    for relay_period_sec in ("0.020", "0.008", "0.004"):
        context = LaunchContext()
        context.launch_configurations.update(
            {
                "use_mock_hardware": "true",
                "joy_deadzone": "0.08",
                "joy_autorepeat_rate": "100.0",
                "joint_states_wait_timeout_sec": "15.0",
                "servo_startup_settle_sec": "5.0",
                "servo_status_wait_timeout_sec": "15.0",
                "joint_state_relay_period_sec": relay_period_sec,
                "servo_command_mode": "twist_feedforward",
                "servo_butterworth_filter_coeff": "1.5",
                "twist_position_gain": "2.0",
                "twist_orientation_gain": "2.0",
                "twist_linear_correction_limit_mps": "0.03",
                "twist_angular_correction_limit_radps": "0.3",
            }
        )
        actions = module.validate_fakehardware_arguments(context)
        assert perform_substitutions(context, actions[0].value) == "true"

    assert '"joint_state_relay_period_sec"' in source
    assert 'default_value="0.020"' in source
    assert '"servo_command_mode"' in benchmark_source
    assert '"servo_use_smoothing"' in benchmark_source
    assert '"servo_butterworth_filter_coeff"' in benchmark_source
    assert '"launch_pen_node"' in source
    assert 'default_value="true"' in source
    assert 'default_value="1.5"' in source
    assert 'prefix="prlimit --rtprio=0:0 --"' in source

    context.launch_configurations["servo_butterworth_filter_coeff"] = "1.01"
    actions = module.validate_fakehardware_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"

    context.launch_configurations["servo_butterworth_filter_coeff"] = "1.0"
    actions = module.validate_fakehardware_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["servo_butterworth_filter_coeff"] = "1.5"
    context.launch_configurations["servo_command_mode"] = "twist_linear_only"
    actions = module.validate_fakehardware_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"

    context.launch_configurations["joint_state_relay_period_sec"] = "0.0"
    actions = module.validate_fakehardware_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"


def test_stage2_fakehardware_constant_twist_launch_disables_pen_node():
    source = FAKE_CONSTANT_TWIST_LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"twist_profile"' in source
    assert 'default_value="pure_x"' in source
    assert '"launch_pen_node": "false"' in source
    assert '"launch_joy_node": "false"' in source
    assert '"joint_state_relay_period_sec"' in source
    assert 'default_value="0.004"' in source
    assert "constant_twist_diagnostic_node" in source
    assert "constant_twist_report.md" in source


def test_stage2_ursim_launch_uses_ursim_defaults_and_current_servo_scale():
    module = _load_stage2_ursim_launch_module()
    source = URSIM_LAUNCH_PATH.read_text(encoding="utf-8")
    benchmark_source = URSIM_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")

    assert module.STAGE2_URSIM_DEFAULT_ROBOT_IP == "172.17.0.2"
    assert module.STAGE2_URSIM_DEFAULT_USE_MOCK_HARDWARE == "false"
    assert module.STAGE2_SERVO_ROTATIONAL_SCALE_RADPS == math.tau
    assert (
        source.count(
            '"servo_output_controller",\n'
            '        default_value="joint_trajectory_controller",'
        )
        == 1
    )
    assert (
        benchmark_source.count(
            '"servo_output_controller",\n'
            '        default_value="joint_trajectory_controller",'
        )
        == 1
    )

    servo_yaml = module.configured_stage2_servo_yaml()
    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["linear"] == 0.6
    assert servo_yaml["scale"]["rotational"] == math.tau
    assert servo_yaml["low_pass_filter_coeff"] == 10.0
    assert module.configured_stage2_servo_yaml(linear_scale=0.8)["scale"][
        "linear"
    ] == 0.8
    assert module.configured_stage2_servo_yaml(linear_scale=1.0)["scale"][
        "linear"
    ] == 1.0
    assert (
        module.configured_stage2_servo_yaml(low_pass_filter_coeff=3.0)[
            "low_pass_filter_coeff"
        ]
        == 3.0
    )
    assert (
        module.configured_stage2_servo_yaml(low_pass_filter_coeff=1.0)[
            "low_pass_filter_coeff"
        ]
        == 1.0
    )
    assert module.configured_stage2_servo_yaml()["publish_period"] == 0.004
    assert (
        module.configured_stage2_servo_yaml()["max_expected_latency"]
        == 0.1
    )
    assert (
        module.configured_stage2_servo_yaml(publish_period=0.008)[
            "publish_period"
        ]
        == 0.008
    )
    assert (
        module.configured_stage2_servo_yaml(max_expected_latency=0.12)[
            "max_expected_latency"
        ]
        == 0.12
    )


def test_stage2_ursim_pen_node_parameters_match_fakehardware_height_strategy():
    module = _load_stage2_ursim_launch_module()

    parameters = module.pen_ursim_node_parameters()

    assert parameters["start_from_current_tool0"] is True
    assert parameters["require_motion_before_pose_command"] is True
    assert parameters["paper_origin_xyz"] == [0.45, 0.0, 0.12]
    assert parameters["tool0_to_pen_tip_xyz"] == [0.0, 0.0, 0.14]
    assert parameters["servo_status_topic"] == "/servo_node/status"


def test_stage2_ursim_launch_stops_owned_external_control_on_shutdown():
    source = URSIM_LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"stop_external_control_on_shutdown"' in source
    assert "sock.sendall(b'stop\\\\n')" in source
    assert "auto_start_external_control" in source
    assert "OnShutdown" in source


def test_stage2_ursim_pose_target_publish_rate_is_configurable_and_positive():
    module = _load_stage2_ursim_launch_module()
    source = URSIM_LAUNCH_PATH.read_text(encoding="utf-8")
    benchmark_source = URSIM_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"pose_target_publish_rate_hz"' in source
    assert 'default_value="60.0"' in source
    assert 'LaunchConfiguration("pose_target_publish_rate_hz")' in source
    assert '"pose_target_publish_rate_hz"' in benchmark_source
    assert '"max_planar_speed_mps"' in source
    assert 'default_value="0.03"' in source
    assert 'LaunchConfiguration("max_planar_speed_mps")' in source
    assert '"max_planar_speed_mps"' in benchmark_source
    assert '"paper_width_m"' in source
    assert '"paper_height_m"' in source
    assert '"initial_tip_x_m"' in source
    assert '"initial_tip_y_m"' in source
    assert '"paper_width_m": LaunchConfiguration("paper_width_m")' in benchmark_source
    assert '"paper_height_m": LaunchConfiguration("paper_height_m")' in benchmark_source
    assert '"initial_tip_x_m": LaunchConfiguration("initial_tip_x_m")' in benchmark_source
    assert '"initial_tip_y_m": LaunchConfiguration("initial_tip_y_m")' in benchmark_source
    assert '"auto_start_external_control": "true"' in benchmark_source
    assert '"stop_external_control_on_shutdown": "true"' in benchmark_source
    assert '"launch_rviz": LaunchConfiguration("launch_rviz")' in benchmark_source
    assert '"fixed_tilt_deg": LaunchConfiguration("fixed_tilt_deg")' in benchmark_source
    assert (
        '"diagnostic_freeze_tip_xy": LaunchConfiguration(' in benchmark_source
    )
    assert (
        '"diagnostic_orientation_mode": LaunchConfiguration('
        in benchmark_source
    )
    assert '"fixed_tilt_deg",' in benchmark_source
    assert '"diagnostic_freeze_tip_xy",' in benchmark_source
    assert '"diagnostic_orientation_mode",' in benchmark_source

    for publish_rate_hz in ("100.0", "125.0"):
        context = LaunchContext()
        context.launch_configurations.update(
            {
                "use_mock_hardware": "false",
                "robot_ip": "172.17.0.2",
                "joy_deadzone": "0.08",
                "joy_autorepeat_rate": "100.0",
                "joint_states_wait_timeout_sec": "15.0",
                "servo_startup_settle_sec": "5.0",
                "servo_status_wait_timeout_sec": "15.0",
                    "servo_linear_scale": "0.6",
                    "servo_low_pass_filter_coeff": "10.0",
                    "servo_publish_period_sec": "0.004",
                    "servo_max_expected_latency_sec": "0.10",
                    "servo_output_controller": "forward_position_controller",
                    "pose_target_publish_rate_hz": publish_rate_hz,
                "max_planar_speed_mps": "0.06",
                "paper_width_m": "0.60",
                "paper_height_m": "0.16",
                "initial_tip_x_m": "-0.24",
                "initial_tip_y_m": "0.0",
                "joint_state_relay_period_sec": "0.004",
                "servo_command_mode": "pose",
                "twist_position_gain": "2.0",
                "twist_orientation_gain": "2.0",
                "twist_linear_correction_limit_mps": "0.03",
                "twist_angular_correction_limit_radps": "0.3",
                "dashboard_receive_timeout_sec": "20.0",
                "script_sender_port": "50002",
                "fixed_tilt_deg": "20.0",
                "diagnostic_orientation_mode": "dynamic",
            }
        )
        actions = module.validate_ursim_arguments(context)
        assert perform_substitutions(context, actions[0].value) == "true"

    context.launch_configurations["pose_target_publish_rate_hz"] = "0.0"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["pose_target_publish_rate_hz"] = "60.0"
    context.launch_configurations["max_planar_speed_mps"] = "0.0"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["max_planar_speed_mps"] = "0.03"
    context.launch_configurations["paper_width_m"] = "0.0"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["paper_width_m"] = "0.60"
    context.launch_configurations["initial_tip_x_m"] = "-0.24"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"


def test_stage2_ursim_twist_feedforward_launch_parameters_are_validated():
    module = _load_stage2_ursim_launch_module()
    source = URSIM_LAUNCH_PATH.read_text(encoding="utf-8")
    benchmark_source = URSIM_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")
    context = LaunchContext()
    context.launch_configurations.update(
        {
            "use_mock_hardware": "false",
            "robot_ip": "172.17.0.2",
            "joy_deadzone": "0.08",
            "joy_autorepeat_rate": "100.0",
            "joint_states_wait_timeout_sec": "15.0",
            "servo_startup_settle_sec": "5.0",
            "servo_status_wait_timeout_sec": "15.0",
            "servo_linear_scale": "0.6",
            "servo_low_pass_filter_coeff": "10.0",
            "servo_publish_period_sec": "0.004",
            "servo_max_expected_latency_sec": "0.10",
            "servo_output_controller": "forward_position_controller",
            "pose_target_publish_rate_hz": "60.0",
            "max_planar_speed_mps": "0.03",
            "paper_width_m": "0.24",
            "paper_height_m": "0.16",
            "initial_tip_x_m": "0.0",
            "initial_tip_y_m": "0.0",
            "joint_state_relay_period_sec": "0.004",
            "servo_command_mode": "twist_feedforward",
            "twist_position_gain": "2.0",
            "twist_orientation_gain": "2.0",
            "twist_linear_correction_limit_mps": "0.03",
            "twist_angular_correction_limit_radps": "0.3",
            "dashboard_receive_timeout_sec": "20.0",
            "script_sender_port": "50002",
            "fixed_tilt_deg": "20.0",
            "diagnostic_orientation_mode": "dynamic",
        }
    )

    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"
    assert perform_substitutions(context, actions[1].value) == (
        "std_msgs/Float64MultiArray"
    )
    assert perform_substitutions(context, actions[2].value) == (
        "/forward_position_controller/commands"
    )
    assert '"servo_command_mode"' in source
    assert '"servo_command_mode"' in benchmark_source
    assert 'default_value="pose"' in source
    assert '"joint_state_relay_period_sec"' in benchmark_source
    assert 'default_value="0.020"' in source
    assert '"diagnostic_freeze_tip_xy"' in source
    assert '"diagnostic_orientation_mode"' in source
    assert '"fixed_tilt_deg"' in source
    assert 'default_value="dynamic"' in source
    assert 'default_value="20.0"' in source
    assert 'prefix="prlimit --rtprio=0:0 --"' in source
    assert '"servo_publish_period_sec"' in source
    assert '"servo_publish_period_sec"' in benchmark_source
    assert '"servo_max_expected_latency_sec"' in source
    assert '"servo_max_expected_latency_sec"' in benchmark_source
    assert 'LaunchConfiguration("servo_publish_period_sec")' in source
    assert '"servo_publish_period_sec": LaunchConfiguration(' in benchmark_source
    assert 'LaunchConfiguration("servo_max_expected_latency_sec")' in source
    assert '"servo_max_expected_latency_sec": LaunchConfiguration(' in benchmark_source

    context.launch_configurations["servo_publish_period_sec"] = "0.0"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"
    context.launch_configurations["servo_publish_period_sec"] = "0.004"
    context.launch_configurations["servo_max_expected_latency_sec"] = "0.0"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"
    context.launch_configurations["servo_max_expected_latency_sec"] = "0.10"

    context.launch_configurations["servo_command_mode"] = "twist_linear_only"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"

    context.launch_configurations["servo_command_mode"] = "twist_constant_linear"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"

    context.launch_configurations["servo_output_controller"] = (
        "joint_trajectory_controller"
    )
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"
    assert perform_substitutions(context, actions[1].value) == (
        "trajectory_msgs/JointTrajectory"
    )
    assert perform_substitutions(context, actions[2].value) == (
        "/joint_trajectory_controller/joint_trajectory"
    )

    context.launch_configurations["servo_output_controller"] = "invalid"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["servo_output_controller"] = (
        "forward_position_controller"
    )
    context.launch_configurations["diagnostic_orientation_mode"] = "fixed_vertical"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "true"

    context.launch_configurations["diagnostic_orientation_mode"] = "invalid"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["diagnostic_orientation_mode"] = "dynamic"
    context.launch_configurations["servo_command_mode"] = "invalid"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["servo_command_mode"] = "twist_feedforward"
    context.launch_configurations["fixed_tilt_deg"] = "90.0"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"

    context.launch_configurations["fixed_tilt_deg"] = "20.0"
    context.launch_configurations["joint_state_relay_period_sec"] = "0.0"
    actions = module.validate_ursim_arguments(context)
    assert perform_substitutions(context, actions[0].value) == "false"


def test_stage2_ursim_benchmark_records_chain_split_inputs_and_outputs():
    source = URSIM_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")

    assert "chain_split_fk_report.json" in source
    assert "chain_split_fk_report.md" in source
    assert "chain_split_fk_report" in source
    assert "ros2" in source
    assert '"bag"' in source
    assert '"/pen_writing/target_pose"' in source
    assert '"/servo_node/pose_target_cmds"' in source
    assert '"/servo_node/delta_twist_cmds"' in source
    assert '"/forward_position_controller/commands"' in source
    assert '"/joint_trajectory_controller/joint_trajectory"' in source
    assert '"/joint_states"' in source


def test_stage3_real_air_launch_requires_human_confirmation():
    module = _load_stage3_real_air_launch_module()

    error = module.validate_real_air_configuration(
        human_confirmation="",
        max_session_duration_sec=30.0,
    )

    assert error is not None
    assert module.REQUIRED_REAL_AIR_CONFIRMATION in error


def test_stage3_force_mode_launch_is_manual_and_disables_pen_input():
    source = REAL_FORCE_MODE_LAUNCH_PATH.read_text(encoding="utf-8")

    assert "I_CONFIRM_REAL_FORCE_MODE_TEST" in source
    assert '"launch_pen_node": "false"' in source
    assert '"launch_joy_node": "false"' in source
    assert "force_mode_validation_node" in source
    assert '"human_confirmation": REQUIRED_FORCE_CONFIRMATION' in source
    assert '"base_frame": "base"' in source
    assert '"tool_frame": "tool0_controller"' in source
    assert '"servo_base_frame": "base_link"' in source
    assert '"servo_tool_frame": "tool0"' in source
    assert '"max_speed_mps": 0.002' in source
    assert '"max_force_n": 10.0' in source


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
    assert (
        module.STAGE3_REAL_INITIAL_JOINT_CONTROLLER
        == "joint_trajectory_controller"
    )
    assert (
        module.STAGE3_REAL_DESCRIPTION_LAUNCHFILE_NAME
        == "task8B_real_calibrated_rsp.launch.py"
    )


def test_stage3_real_air_launch_uses_current_servo_scale():
    module = _load_stage3_real_air_launch_module()

    servo_yaml = module.configured_stage3_servo_yaml()

    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["command_out_type"] == "trajectory_msgs/JointTrajectory"
    assert (
        servo_yaml["command_out_topic"]
        == "/joint_trajectory_controller/joint_trajectory"
    )
    assert servo_yaml["scale"]["rotational"] == math.tau
    assert servo_yaml["use_smoothing"] is True
    disabled_servo_yaml = module.configured_stage3_servo_yaml(use_smoothing=False)
    assert disabled_servo_yaml["use_smoothing"] is False
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


def test_stage3_real_air_paper_seek_requires_tool_payload_configuration():
    source = REAL_AIR_LAUNCH_PATH.read_text(encoding="utf-8")

    assert '"paper_seek_payload_mass_kg"' in source
    assert 'default_value="-1.0"' in source
    assert '"paper_seek_configure_payload": True' in source
    assert '"paper_seek_zero_ft_before_start": True' in source


def test_stage3_real_air_pen_can_use_configured_paper_origin_exactly():
    module = _load_stage3_real_air_launch_module()

    parameters = module.pen_real_air_node_parameters(
        30.0,
        paper_origin_xyz=[-0.168299, -0.355821, -0.0811198],
        tool0_to_pen_tip_xyz=[0.00121417, 0.0311535, 0.173598],
        start_from_current_tool0=False,
    )

    assert parameters["start_from_current_tool0"] is False
    assert parameters["paper_origin_xyz"] == [-0.168299, -0.355821, -0.0811198]
    assert parameters["tool0_to_pen_tip_xyz"] == [0.00121417, 0.0311535, 0.173598]


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
    servo_yaml = module.configured_stage3_servo_yaml()

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
    assert servo_yaml["use_smoothing"] is True
    disabled_servo_yaml = module.configured_stage3_servo_yaml(use_smoothing=False)
    assert disabled_servo_yaml["use_smoothing"] is False


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


def test_stage3_real_benchmark_exposes_orientation_diagnostic_knobs():
    source = REAL_BENCHMARK_LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("fixed_tilt_deg", default_value="20.0")' in source
    assert (
        'DeclareLaunchArgument("diagnostic_freeze_tip_xy", default_value="false")'
        in source
    )
    assert '"fixed_tilt_deg": fixed_tilt_deg' in source
    assert '"diagnostic_freeze_tip_xy": diagnostic_freeze_tip_xy' in source
    assert 'DeclareLaunchArgument("benchmark_profile", default_value="eight_direction")' in source
    assert '"benchmark_profile": benchmark_profile' in source


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
    assert module.validate_benchmark_configuration(
        human_confirmation=module.REQUIRED_CONFIRMATION,
        max_session_duration_sec=60.0,
        motion_scale=1.1,
    )
    assert module.validate_benchmark_configuration(
        human_confirmation=module.REQUIRED_CONFIRMATION,
        max_session_duration_sec=60.0,
        fixed_tilt_deg=90.0,
    )
    assert module.validate_benchmark_configuration(
        human_confirmation=module.REQUIRED_CONFIRMATION,
        max_session_duration_sec=60.0,
        benchmark_profile="unknown_profile",
    )
    assert (
        module.validate_benchmark_configuration(
            human_confirmation=module.REQUIRED_CONFIRMATION,
            max_session_duration_sec=60.0,
            motion_scale=0.25,
            fixed_tilt_deg=0.0,
            benchmark_profile="long_minus_y_plus_xy",
        )
        is None
    )
