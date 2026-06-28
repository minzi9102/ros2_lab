import importlib.util
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "stage2_fakehardware_pen_servo.launch.py"
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


def test_stage2_launch_sets_servo_rotational_scale_to_half_of_slowest_ur3_joint_limit():
    module = _load_stage2_launch_module()

    assert module.STAGE2_SERVO_ROTATIONAL_SCALE_RADPS == 1.5708
    servo_yaml = module.configured_stage2_servo_yaml()

    assert servo_yaml["joint_topic"] == "/task7e/joint_states_fresh"
    assert servo_yaml["scale"]["rotational"] == 1.5708
