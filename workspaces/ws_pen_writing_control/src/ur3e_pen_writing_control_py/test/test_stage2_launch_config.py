import importlib.util
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


def test_stage2_launch_sets_servo_rotational_scale_to_half_of_slowest_ur3_joint_limit():
    module = _load_stage2_launch_module()

    assert module.STAGE2_SERVO_ROTATIONAL_SCALE_RADPS == 1.5708
    servo_yaml = module.configured_stage2_servo_yaml()

    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["rotational"] == 1.5708


def test_stage2_ursim_launch_uses_ursim_defaults_and_current_servo_scale():
    module = _load_stage2_ursim_launch_module()

    assert module.STAGE2_URSIM_DEFAULT_ROBOT_IP == "172.17.0.2"
    assert module.STAGE2_URSIM_DEFAULT_USE_MOCK_HARDWARE == "false"
    assert module.STAGE2_SERVO_ROTATIONAL_SCALE_RADPS == 1.5708

    servo_yaml = module.configured_stage2_servo_yaml()
    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["rotational"] == 1.5708


def test_stage2_ursim_pen_node_parameters_match_fakehardware_height_strategy():
    module = _load_stage2_ursim_launch_module()

    parameters = module.pen_ursim_node_parameters()

    assert parameters["start_from_current_tool0"] is True
    assert parameters["require_motion_before_pose_command"] is True
    assert parameters["paper_origin_xyz"] == [0.45, 0.0, 0.12]
    assert parameters["tool0_to_pen_tip_xyz"] == [0.0, 0.0, 0.14]
    assert parameters["servo_status_topic"] == "/servo_node/status"
