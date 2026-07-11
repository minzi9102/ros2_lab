import inspect
import math

import pytest

from ur3e_pen_writing_control_py.force_mode_validation_node import (
    controller_switch_request,
    controller_switch_verified,
    FORCE_CONTROLLER,
    ForceModeValidationNode,
    force_delta_norm,
    POSITION_CONTROLLER,
    position_distance,
    PROFILES,
    projected_displacement,
    quaternion_distance,
    retracted_pose,
)
from ur3e_pen_writing_control_py.pose_math import Point3, PoseTarget, Quaternion


def test_force_profiles_keep_hard_safety_limits():
    assert PROFILES["hold"].target_force_n == 0.0
    assert PROFILES["zero"].target_force_n == 0.0
    assert PROFILES["direction"].target_force_n == 0.5
    assert PROFILES["contact"].target_force_n == 2.0
    assert max(profile.duration_sec for profile in PROFILES.values()) <= 5.0
    assert (
        max(profile.max_displacement_m for profile in PROFILES.values()) <= 0.005
    )


@pytest.mark.parametrize(
    ("activate", "deactivate"),
    [
        (FORCE_CONTROLLER, POSITION_CONTROLLER),
        (POSITION_CONTROLLER, FORCE_CONTROLLER),
    ],
)
def test_controller_switches_are_atomic_and_strict(activate, deactivate):
    request = controller_switch_request(activate, deactivate)

    assert request.activate_controllers == [activate]
    assert request.deactivate_controllers == [deactivate]
    assert request.strictness == request.STRICT
    assert request.activate_asap is True
    assert request.timeout.sec == 5


def test_controller_switch_requires_verified_final_states():
    assert controller_switch_verified(
        {FORCE_CONTROLLER: "active", POSITION_CONTROLLER: "inactive"},
        FORCE_CONTROLLER,
        POSITION_CONTROLLER,
    )
    assert not controller_switch_verified(
        {FORCE_CONTROLLER: "active", POSITION_CONTROLLER: "active"},
        FORCE_CONTROLLER,
        POSITION_CONTROLLER,
    )


def test_force_stop_leaves_active_state_before_async_request():
    source = inspect.getsource(ForceModeValidationNode._finish_active)

    assert source.index("self._state = STATE_STOPPING_FORCE") < source.index(
        "call_async"
    )


def test_hold_gate_requires_success_and_skips_retraction():
    source = inspect.getsource(ForceModeValidationNode._monitor_hold)

    assert "self._hold_verified = self._pending_success" in source
    assert source.index('if self._profile.name == "hold"') < source.index(
        "self._retract_target = retracted_pose"
    )


def test_quaternion_distance_treats_sign_as_same_orientation():
    identity = Quaternion(0.0, 0.0, 0.0, 1.0)

    assert quaternion_distance(identity, Quaternion(0.0, 0.0, 0.0, -1.0)) == 0.0
    assert quaternion_distance(
        identity, Quaternion(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    ) == pytest.approx(math.pi / 2.0)


def test_projected_displacement_uses_captured_tool_axis():
    start = Point3(0.1, 0.2, 0.3)
    current = Point3(0.102, 0.203, 0.304)

    assert projected_displacement(
        start, current, (1.0, 0.0, 0.0)
    ) == pytest.approx(0.002)
    assert projected_displacement(
        start, current, (0.0, 0.0, 1.0)
    ) == pytest.approx(0.004)


def test_force_limit_uses_full_force_delta():
    assert force_delta_norm((1.0, 2.0, 3.0), (4.0, 6.0, 3.0)) == pytest.approx(5.0)


def test_retraction_moves_opposite_captured_tool_axis():
    current = PoseTarget(
        position=Point3(0.4, 0.2, 0.1),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
    axis = (math.sqrt(0.5), 0.0, math.sqrt(0.5))
    target = retracted_pose(current, axis, 0.003)

    assert projected_displacement(
        current.position, target.position, axis
    ) == pytest.approx(-0.003)
    assert position_distance(current.position, target.position) == pytest.approx(0.003)
    assert target.orientation == current.orientation
