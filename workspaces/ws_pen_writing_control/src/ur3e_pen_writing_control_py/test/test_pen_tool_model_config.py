from pathlib import Path

import yaml


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "pen_tool_model.yaml"
EXPECTED_LOW_SPEED_PARAMETERS = {
    "max_planar_speed_mps": 0.025,
    "tilt_rate_degps": 8.0,
    "untilt_rate_degps": 10.0,
    "max_pen_axis_angular_speed_degps": 10.0,
}


def test_stage1_and_stage2_share_low_speed_virtual_pen_parameters():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))

    for node_name in (
        "pen_writing_visualizer_node",
        "pen_fakehardware_servo",
    ):
        parameters = config[node_name]["ros__parameters"]
        for parameter_name, expected_value in EXPECTED_LOW_SPEED_PARAMETERS.items():
            assert parameters[parameter_name] == expected_value
