import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CATALOG = PACKAGE_ROOT / 'config' / 'ur3e_named_targets.yaml'


def load_catalog():
    with CATALOG.open() as stream:
        return yaml.safe_load(stream)


def test_runtime_modes_are_present():
    catalog = load_catalog()

    assert catalog['schema_version'] == 1
    assert set(catalog['runtime_modes']) == {'sim', 'real'}


def test_targets_match_joint_count():
    catalog = load_catalog()

    for mode_name, mode in catalog['runtime_modes'].items():
        joint_count = len(mode['joint_names'])
        assert joint_count == 6, mode_name

        for target_name, target in mode['targets'].items():
            assert len(target['positions_rad']) == joint_count, target_name
            assert 'reviewed_by' in target
            assert isinstance(target['enabled'], bool)


def test_real_targets_require_explicit_human_review():
    catalog = load_catalog()

    real_targets = catalog['runtime_modes']['real']['targets']
    assert real_targets
    reviewed_targets = ['home', 'ready']

    enabled_targets = [
        target_name
        for target_name, target in real_targets.items()
        if target['enabled']
    ]
    assert enabled_targets == reviewed_targets

    for target_name, target in real_targets.items():
        if target_name in reviewed_targets:
            assert 'TODO(human)' not in target['reviewed_by']
            continue
        assert not target['enabled'], target_name
        assert 'TODO(human)' in target['reviewed_by']


def test_sim_ready_moves_first_and_third_joints_by_30_degrees():
    catalog = load_catalog()

    sim_targets = catalog['runtime_modes']['sim']['targets']
    home = sim_targets['home']['positions_rad']
    ready = sim_targets['ready']['positions_rad']
    expected_delta = math.radians(30.0)

    deltas = [ready_value - home_value for home_value, ready_value in zip(home, ready)]
    expected_deltas = [
        expected_delta,
        0.0,
        -expected_delta,
        0.0,
        0.0,
        0.0,
    ]

    for actual, expected in zip(deltas, expected_deltas):
        assert math.isclose(actual, expected, abs_tol=1.0e-4)
