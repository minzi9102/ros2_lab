import math

import pytest

from ur3e_pen2_control_py.kinematics import (
    VirtualPenConfig,
    VirtualPenKinematics,
    blend_angle,
    cross,
    quaternion_dot,
    smoothstep,
    subtract_vectors,
    vector_norm,
)


def test_smoothstep_is_bounded_and_continuous_at_edges():
    assert smoothstep(1.0, 2.0, 0.0) == 0.0
    assert smoothstep(1.0, 2.0, 1.0) == 0.0
    assert smoothstep(1.0, 2.0, 1.5) == pytest.approx(0.5)
    assert smoothstep(1.0, 2.0, 2.0) == 1.0
    assert smoothstep(1.0, 2.0, 3.0) == 1.0


def test_blend_angle_takes_short_path_across_pi():
    result = blend_angle(math.radians(179.0), math.radians(-179.0), 0.5)

    assert abs(abs(result) - math.pi) < math.radians(0.01)


def test_first_motion_initializes_yaw_without_rotating_upright_pen():
    pen = VirtualPenKinematics()
    initial_orientation = pen.state.orientation_world

    state = pen.update(0.008, 1.0, 0.0)

    assert state.motion_phase == "MOVING"
    assert state.yaw_rad == pytest.approx(0.0)
    assert state.tilt_rad == pytest.approx(0.0)
    assert quaternion_dot(initial_orientation, state.orientation_world) == pytest.approx(
        1.0
    )
    assert vector_norm(state.angular_velocity_world) == pytest.approx(0.0)


def test_direction_change_while_moving_keeps_yaw_rate_limit():
    config = VirtualPenConfig(paper_width_m=10.0, paper_height_m=10.0)
    pen = VirtualPenKinematics(config)
    dt = 0.008
    for _ in range(400):
        pen.update(dt, 1.0, 0.0)
    previous_yaw = pen.state.yaw_rad

    state = pen.update(dt, 0.0, 1.0)

    assert state.yaw_rad != pytest.approx(math.pi / 2.0)
    assert abs(state.yaw_rad - previous_yaw) <= config.max_yaw_rate_radps * dt


def test_planar_motion_respects_speed_acceleration_and_jerk_limits():
    config = VirtualPenConfig()
    pen = VirtualPenKinematics(config)
    dt = 0.008
    previous_acceleration = (0.0, 0.0, 0.0)

    for _ in range(500):
        state = pen.update(dt, 1.0, 0.0)
        jerk = vector_norm(
            tuple(
                (current - previous) / dt
                for current, previous in zip(
                    state.tip_acceleration_world,
                    previous_acceleration,
                )
            )
        )
        assert state.planar_speed_mps <= config.max_speed_mps + 1e-12
        assert vector_norm(state.tip_acceleration_world) <= config.max_accel_mps2 + 1e-12
        assert jerk <= config.max_jerk_mps3 + 1e-9
        previous_acceleration = state.tip_acceleration_world


def test_release_uses_deceleration_limit_and_eventually_stops():
    config = VirtualPenConfig(paper_width_m=10.0, paper_height_m=10.0)
    pen = VirtualPenKinematics(config)
    dt = 0.008
    for _ in range(200):
        pen.update(dt, 1.0, 0.0)

    states = [pen.update(dt, 0.0, 0.0) for _ in range(1000)]

    assert all(
        vector_norm(state.tip_acceleration_world)
        <= config.max_decel_mps2 + 1e-12
        for state in states
    )
    assert states[-1].planar_speed_mps == 0.0


def test_motion_state_machine_holds_then_returns_to_idle():
    config = VirtualPenConfig(
        hold_time_sec=0.03,
        max_tilt_rate_radps=math.radians(90.0),
        max_untilt_rate_radps=math.radians(90.0),
        max_tilt_accel_radps2=math.radians(720.0),
        max_axis_angular_speed_radps=math.radians(90.0),
        max_axis_angular_accel_radps2=math.radians(720.0),
        paper_width_m=10.0,
        paper_height_m=10.0,
    )
    pen = VirtualPenKinematics(config)
    dt = 0.008
    phases = []
    for _ in range(100):
        phases.append(pen.update(dt, 1.0, 0.0).motion_phase)
    for _ in range(1000):
        phases.append(pen.update(dt, 0.0, 0.0).motion_phase)

    assert phases[0] == "MOVING"
    assert "HOLDING" in phases
    assert "RETURNING" in phases
    assert phases[-1] == "IDLE"


def test_short_hold_preserves_tilt_before_returning():
    config = VirtualPenConfig(
        hold_time_sec=0.10,
        paper_width_m=10.0,
        paper_height_m=10.0,
    )
    pen = VirtualPenKinematics(config)
    dt = 0.008
    for _ in range(300):
        pen.update(dt, 1.0, 0.0)

    holding_states = []
    for _ in range(1000):
        state = pen.update(dt, 0.0, 0.0)
        if state.motion_phase == "HOLDING":
            holding_states.append(state)
        if state.motion_phase == "RETURNING":
            break

    assert holding_states
    assert max(state.tilt_rad for state in holding_states) == pytest.approx(
        min(state.tilt_rad for state in holding_states),
        abs=1e-12,
    )


def test_paper_boundary_clears_outward_velocity_and_acceleration():
    config = VirtualPenConfig(
        paper_width_m=0.01,
        paper_height_m=0.01,
        paper_origin_world=(0.0, 0.0, 0.0),
    )
    pen = VirtualPenKinematics(config)

    for _ in range(1000):
        state = pen.update(0.008, 1.0, 0.0)

    assert state.tip_position_world[0] == pytest.approx(0.005)
    assert state.tip_velocity_world[0] == 0.0
    assert state.tip_acceleration_world[0] == 0.0


def test_quaternion_sign_is_continuous_and_axis_speed_is_limited():
    config = VirtualPenConfig(paper_width_m=10.0, paper_height_m=10.0)
    pen = VirtualPenKinematics(config)
    previous = pen.state.orientation_world

    for index in range(600):
        angle = index * 0.03
        state = pen.update(0.008, math.cos(angle), math.sin(angle))
        assert quaternion_dot(previous, state.orientation_world) >= 0.0
        assert (
            state.axis_angular_speed_radps
            <= config.max_axis_angular_speed_radps + 1e-12
        )
        previous = state.orientation_world


def test_tool0_velocity_uses_same_frame_rigid_body_relation():
    pen = VirtualPenKinematics(
        VirtualPenConfig(paper_width_m=10.0, paper_height_m=10.0)
    )
    state = None
    for _ in range(400):
        state = pen.update(0.008, 0.0, 1.0)
    assert state is not None
    offset = subtract_vectors(
        state.tip_position_world,
        state.tool0_position_world,
    )
    reconstructed = subtract_vectors(
        state.tip_velocity_world,
        cross(state.angular_velocity_world, offset),
    )

    assert state.tool0_linear_velocity_world == pytest.approx(reconstructed)


@pytest.mark.parametrize(
    "trajectory",
    [
        [(1.0, 0.0)] * 100 + [(0.0, 0.0)] * 200,
        [(1.0, 0.0)] * 100 + [(0.0, 1.0)] * 100,
        [(1.0, 0.0)] * 100 + [(-1.0, 0.0)] * 100,
        [
            (math.cos(index * 0.05), math.sin(index * 0.05))
            for index in range(200)
        ],
        [(0.01, -0.01), (-0.01, 0.01)] * 100,
    ],
)
def test_offline_trajectories_remain_finite_and_inside_paper(trajectory):
    config = VirtualPenConfig()
    pen = VirtualPenKinematics(config)
    origin = config.paper_origin_world

    for intent in trajectory:
        state = pen.update(0.008, *intent)
        values = (
            *state.tip_position_world,
            *state.tip_velocity_world,
            *state.angular_velocity_world,
            *state.tool0_position_world,
            *state.tool0_linear_velocity_world,
        )
        assert all(math.isfinite(value) for value in values)
        assert abs(state.tip_position_world[0] - origin[0]) <= config.paper_width_m / 2.0
        assert abs(state.tip_position_world[1] - origin[1]) <= config.paper_height_m / 2.0
