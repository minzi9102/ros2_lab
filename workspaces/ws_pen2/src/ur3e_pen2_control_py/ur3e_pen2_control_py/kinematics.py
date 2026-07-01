import math
from dataclasses import dataclass

Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float


@dataclass(frozen=True)
class OrientationFrame:
    x_axis: Vector3
    y_axis: Vector3
    z_axis: Vector3


@dataclass(frozen=True)
class VirtualPenConfig:
    max_speed_mps: float = 0.03
    max_accel_mps2: float = 0.08
    max_decel_mps2: float = 0.16
    max_jerk_mps3: float = 0.80
    confidence_speed_low_mps: float = 0.003
    confidence_speed_high_mps: float = 0.015
    max_yaw_rate_radps: float = math.radians(30.0)
    max_yaw_accel_radps2: float = math.radians(120.0)
    max_tilt_rad: float = math.radians(20.0)
    tilt_speed_low_mps: float = 0.003
    tilt_speed_high_mps: float = 0.020
    max_tilt_rate_radps: float = math.radians(12.0)
    max_untilt_rate_radps: float = math.radians(12.0)
    max_tilt_accel_radps2: float = math.radians(80.0)
    max_axis_angular_speed_radps: float = math.radians(12.0)
    max_axis_angular_accel_radps2: float = math.radians(80.0)
    hold_time_sec: float = 0.30
    paper_width_m: float = 0.24
    paper_height_m: float = 0.16
    paper_origin_world: Vector3 = (0.45, 0.0, 0.12)
    tool0_to_pen_tip: Vector3 = (0.0, 0.0, 0.14)
    initial_tip_xy: tuple[float, float] = (0.0, 0.0)
    initial_yaw_rad: float = math.pi

    def __post_init__(self) -> None:
        positive = (
            self.max_speed_mps,
            self.max_accel_mps2,
            self.max_decel_mps2,
            self.max_jerk_mps3,
            self.max_yaw_rate_radps,
            self.max_yaw_accel_radps2,
            self.max_tilt_rate_radps,
            self.max_untilt_rate_radps,
            self.max_tilt_accel_radps2,
            self.max_axis_angular_speed_radps,
            self.max_axis_angular_accel_radps2,
            self.paper_width_m,
            self.paper_height_m,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("motion limits and paper dimensions must be positive")
        if self.hold_time_sec < 0.0:
            raise ValueError("hold_time_sec must be non-negative")
        if not 0.0 <= self.max_tilt_rad < math.pi / 2.0:
            raise ValueError("max_tilt_rad must be in [0, pi/2)")
        _validate_range(
            self.confidence_speed_low_mps,
            self.confidence_speed_high_mps,
            "confidence speed",
        )
        _validate_range(
            self.tilt_speed_low_mps,
            self.tilt_speed_high_mps,
            "tilt speed",
        )


@dataclass(frozen=True)
class VirtualPenKinematicState:
    time_sec: float
    tip_position_world: Vector3
    tip_velocity_world: Vector3
    tip_acceleration_world: Vector3
    orientation_world: Quaternion
    angular_velocity_world: Vector3
    angular_acceleration_world: Vector3
    planar_speed_mps: float
    direction_confidence: float
    yaw_rad: float
    tilt_rad: float
    motion_phase: str
    axis_angular_speed_radps: float
    tool0_position_world: Vector3
    tool0_orientation_world: Quaternion
    tool0_linear_velocity_world: Vector3
    tool0_angular_velocity_world: Vector3


class VirtualPenKinematics:
    def __init__(self, config: VirtualPenConfig | None = None) -> None:
        self.config = config or VirtualPenConfig()
        tip_x = clamp(
            self.config.initial_tip_xy[0],
            -self.config.paper_width_m / 2.0,
            self.config.paper_width_m / 2.0,
        )
        tip_y = clamp(
            self.config.initial_tip_xy[1],
            -self.config.paper_height_m / 2.0,
            self.config.paper_height_m / 2.0,
        )
        origin = self.config.paper_origin_world
        self._tip_position = [origin[0] + tip_x, origin[1] + tip_y, origin[2]]
        self._velocity = [0.0, 0.0]
        self._acceleration = [0.0, 0.0]
        self._yaw = self.config.initial_yaw_rad
        self._yaw_rate = 0.0
        self._tilt = 0.0
        self._tilt_rate = 0.0
        self._phase = "IDLE"
        self._hold_elapsed = 0.0
        self._held_tilt = 0.0
        self._time = 0.0
        self._axis_speed = 0.0
        self._frame = orientation_frame(self._yaw, self._tilt)
        self._orientation = quaternion_from_frame(self._frame)
        self._angular_velocity = (0.0, 0.0, 0.0)
        self._state = self._make_state(
            direction_confidence=0.0,
            angular_acceleration=(0.0, 0.0, 0.0),
        )

    @property
    def state(self) -> VirtualPenKinematicState:
        return self._state

    def emergency_stop(self) -> VirtualPenKinematicState:
        self._velocity[:] = (0.0, 0.0)
        self._acceleration[:] = (0.0, 0.0)
        self._yaw_rate = 0.0
        self._tilt_rate = 0.0
        self._axis_speed = 0.0
        self._angular_velocity = (0.0, 0.0, 0.0)
        self._held_tilt = self._tilt
        self._hold_elapsed = 0.0
        self._phase = "HOLDING"
        self._state = self._make_state(
            direction_confidence=0.0,
            angular_acceleration=(0.0, 0.0, 0.0),
        )
        return self._state

    def update(
        self,
        dt_sec: float,
        intent_x: float,
        intent_y: float,
    ) -> VirtualPenKinematicState:
        if dt_sec <= 0.0:
            raise ValueError("dt_sec must be positive")
        self._time += dt_sec

        intent_x, intent_y = normalize_intent(intent_x, intent_y)
        has_intent = intent_x != 0.0 or intent_y != 0.0
        desired_velocity = (
            intent_x * self.config.max_speed_mps,
            intent_y * self.config.max_speed_mps,
        )
        self._update_planar_velocity(desired_velocity, dt_sec)
        self._integrate_tip_and_apply_bounds(dt_sec)

        speed = math.hypot(*self._velocity)
        acceleration = math.hypot(*self._acceleration)
        self._update_phase(has_intent, speed, acceleration, dt_sec)

        confidence = smoothstep(
            self.config.confidence_speed_low_mps,
            self.config.confidence_speed_high_mps,
            speed,
        )
        self._update_yaw(confidence, speed, dt_sec)
        self._update_tilt(speed, dt_sec)

        previous_orientation = self._orientation
        previous_angular_velocity = self._angular_velocity
        self._update_orientation(dt_sec)
        self._angular_velocity = angular_velocity_from_delta(
            previous_orientation,
            self._orientation,
            dt_sec,
        )
        angular_acceleration = scale_vector(
            subtract_vectors(self._angular_velocity, previous_angular_velocity),
            1.0 / dt_sec,
        )
        self._state = self._make_state(
            direction_confidence=confidence,
            angular_acceleration=angular_acceleration,
        )
        return self._state

    def _update_planar_velocity(
        self,
        desired: tuple[float, float],
        dt_sec: float,
    ) -> None:
        delta = (
            desired[0] - self._velocity[0],
            desired[1] - self._velocity[1],
        )
        desired_acceleration = (delta[0] / dt_sec, delta[1] / dt_sec)
        accelerating = (
            delta[0] * self._velocity[0] + delta[1] * self._velocity[1]
        ) >= 0.0
        acceleration_limit = (
            self.config.max_accel_mps2
            if accelerating
            else self.config.max_decel_mps2
        )
        target_acceleration = limit_vector_norm(
            desired_acceleration,
            acceleration_limit,
        )
        next_acceleration = move_vector_toward(
            tuple(self._acceleration),
            target_acceleration,
            self.config.max_jerk_mps3 * dt_sec,
        )
        self._acceleration[:] = next_acceleration
        self._velocity[0] += self._acceleration[0] * dt_sec
        self._velocity[1] += self._acceleration[1] * dt_sec
        self._velocity[:] = limit_vector_norm(
            tuple(self._velocity),
            self.config.max_speed_mps,
        )

        if desired == (0.0, 0.0):
            speed = math.hypot(*self._velocity)
            acceleration = math.hypot(*self._acceleration)
            if (
                speed <= max(acceleration * dt_sec, self.config.max_jerk_mps3 * dt_sec**2)
                and acceleration <= self.config.max_jerk_mps3 * dt_sec
            ):
                self._velocity[:] = (0.0, 0.0)
                self._acceleration[:] = (0.0, 0.0)

    def _integrate_tip_and_apply_bounds(self, dt_sec: float) -> None:
        origin_x, origin_y, _ = self.config.paper_origin_world
        min_x = origin_x - self.config.paper_width_m / 2.0
        max_x = origin_x + self.config.paper_width_m / 2.0
        min_y = origin_y - self.config.paper_height_m / 2.0
        max_y = origin_y + self.config.paper_height_m / 2.0
        next_x = self._tip_position[0] + self._velocity[0] * dt_sec
        next_y = self._tip_position[1] + self._velocity[1] * dt_sec
        clamped_x = clamp(next_x, min_x, max_x)
        clamped_y = clamp(next_y, min_y, max_y)
        self._tip_position[0] = clamped_x
        self._tip_position[1] = clamped_y
        if clamped_x != next_x:
            self._velocity[0] = 0.0
            self._acceleration[0] = 0.0
        if clamped_y != next_y:
            self._velocity[1] = 0.0
            self._acceleration[1] = 0.0

    def _update_phase(
        self,
        has_intent: bool,
        speed: float,
        acceleration: float,
        dt_sec: float,
    ) -> None:
        moving = speed > 1e-9 or acceleration > 1e-9
        if has_intent or moving:
            self._phase = "MOVING"
            self._hold_elapsed = 0.0
            return
        if self._phase == "MOVING":
            self._phase = "HOLDING"
            self._held_tilt = self._tilt
            self._hold_elapsed = 0.0
            return
        if self._phase == "HOLDING":
            self._hold_elapsed += dt_sec
            if self._hold_elapsed >= self.config.hold_time_sec:
                self._phase = "RETURNING"
            return
        if self._phase == "RETURNING":
            if (
                abs(self._tilt) < 1e-6
                and abs(self._tilt_rate) < 1e-6
                and self._axis_speed < 1e-6
            ):
                self._phase = "IDLE"

    def _update_yaw(
        self,
        confidence: float,
        speed: float,
        dt_sec: float,
    ) -> None:
        target = self._yaw
        if speed > 1e-12 and self._phase == "MOVING":
            raw_yaw = math.atan2(self._velocity[1], self._velocity[0])
            target = blend_angle(self._yaw, raw_yaw, confidence)
        self._yaw, self._yaw_rate = update_limited_angle(
            self._yaw,
            self._yaw_rate,
            target,
            self.config.max_yaw_rate_radps,
            self.config.max_yaw_accel_radps2,
            dt_sec,
        )

    def _update_tilt(self, speed: float, dt_sec: float) -> None:
        if self._phase == "MOVING":
            ratio = smoothstep(
                self.config.tilt_speed_low_mps,
                self.config.tilt_speed_high_mps,
                speed,
            )
            target = self.config.max_tilt_rad * ratio
        elif self._phase == "HOLDING":
            target = self._held_tilt
        else:
            target = 0.0

        delta = target - self._tilt
        rate_limit = (
            self.config.max_tilt_rate_radps
            if delta >= 0.0
            else self.config.max_untilt_rate_radps
        )
        desired_rate = clamp(delta / dt_sec, -rate_limit, rate_limit)
        self._tilt_rate = move_toward(
            self._tilt_rate,
            desired_rate,
            self.config.max_tilt_accel_radps2 * dt_sec,
        )
        step = self._tilt_rate * dt_sec
        if delta != 0.0 and step * delta > 0.0 and abs(step) >= abs(delta):
            self._tilt = target
            self._tilt_rate = 0.0
        else:
            self._tilt = clamp(
                self._tilt + step,
                0.0,
                self.config.max_tilt_rad,
            )

    def _update_orientation(self, dt_sec: float) -> None:
        target_frame = orientation_frame(self._yaw, self._tilt)
        angle = vector_angle(self._frame.z_axis, target_frame.z_axis)
        target_speed = min(
            angle / dt_sec,
            self.config.max_axis_angular_speed_radps,
        )
        self._axis_speed = move_toward(
            self._axis_speed,
            target_speed,
            self.config.max_axis_angular_accel_radps2 * dt_sec,
        )
        next_z = rotate_unit_vector_toward(
            self._frame.z_axis,
            target_frame.z_axis,
            min(angle, self._axis_speed * dt_sec),
        )
        self._frame = minimal_twist_frame(self._frame, next_z)
        orientation = quaternion_from_frame(self._frame)
        if quaternion_dot(self._orientation, orientation) < 0.0:
            orientation = negate_quaternion(orientation)
        self._orientation = orientation

    def _make_state(
        self,
        *,
        direction_confidence: float,
        angular_acceleration: Vector3,
    ) -> VirtualPenKinematicState:
        tip_velocity = (self._velocity[0], self._velocity[1], 0.0)
        tip_acceleration = (
            self._acceleration[0],
            self._acceleration[1],
            0.0,
        )
        offset = rotate_vector(self._orientation, self.config.tool0_to_pen_tip)
        tool_position = subtract_vectors(tuple(self._tip_position), offset)
        tool_velocity = subtract_vectors(
            tip_velocity,
            cross(self._angular_velocity, offset),
        )
        return VirtualPenKinematicState(
            time_sec=self._time,
            tip_position_world=tuple(self._tip_position),
            tip_velocity_world=tip_velocity,
            tip_acceleration_world=tip_acceleration,
            orientation_world=self._orientation,
            angular_velocity_world=self._angular_velocity,
            angular_acceleration_world=angular_acceleration,
            planar_speed_mps=math.hypot(*self._velocity),
            direction_confidence=direction_confidence,
            yaw_rad=self._yaw,
            tilt_rad=self._tilt,
            motion_phase=self._phase,
            axis_angular_speed_radps=self._axis_speed,
            tool0_position_world=tool_position,
            tool0_orientation_world=self._orientation,
            tool0_linear_velocity_world=tool_velocity,
            tool0_angular_velocity_world=self._angular_velocity,
        )


def smoothstep(edge0: float, edge1: float, value: float) -> float:
    _validate_range(edge0, edge1, "smoothstep")
    t = clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def blend_angle(current: float, target: float, weight: float) -> float:
    return wrap_angle(current + shortest_angle(target - current) * clamp(weight, 0.0, 1.0))


def normalize_intent(x: float, y: float) -> tuple[float, float]:
    magnitude = math.hypot(x, y)
    if magnitude == 0.0:
        return 0.0, 0.0
    scale = 1.0 / max(1.0, magnitude)
    return x * scale, y * scale


def update_limited_angle(
    current: float,
    current_rate: float,
    target: float,
    max_rate: float,
    max_acceleration: float,
    dt_sec: float,
) -> tuple[float, float]:
    delta = shortest_angle(target - current)
    if abs(delta) < 1e-12 and abs(current_rate) < 1e-12:
        return current, 0.0
    braking_rate = math.sqrt(2.0 * max_acceleration * abs(delta))
    desired_rate = math.copysign(min(max_rate, braking_rate), delta) if delta else 0.0
    rate = move_toward(current_rate, desired_rate, max_acceleration * dt_sec)
    step = rate * dt_sec
    if delta != 0.0 and step * delta > 0.0 and abs(step) >= abs(delta):
        return wrap_angle(target), 0.0
    return wrap_angle(current + step), rate


def orientation_frame(yaw: float, tilt: float) -> OrientationFrame:
    tip_to_tail = (
        math.sin(tilt) * math.cos(yaw),
        math.sin(tilt) * math.sin(yaw),
        math.cos(tilt),
    )
    z_axis = scale_vector(tip_to_tail, -1.0)
    heading = (-math.cos(yaw), -math.sin(yaw), 0.0)
    y_axis = normalize_vector(cross(z_axis, heading))
    x_axis = normalize_vector(cross(y_axis, z_axis))
    return OrientationFrame(x_axis=x_axis, y_axis=y_axis, z_axis=z_axis)


def minimal_twist_frame(
    previous: OrientationFrame,
    next_z_axis: Vector3,
) -> OrientationFrame:
    z_axis = normalize_vector(next_z_axis)
    projected_x = project_onto_plane(previous.x_axis, z_axis)
    if vector_norm(projected_x) < 1e-9:
        projected_x = project_onto_plane(previous.y_axis, z_axis)
    if vector_norm(projected_x) < 1e-9:
        projected_x = project_onto_plane(least_aligned_axis(z_axis), z_axis)
    x_axis = normalize_vector(projected_x)
    y_axis = normalize_vector(cross(z_axis, x_axis))
    return OrientationFrame(
        x_axis=normalize_vector(cross(y_axis, z_axis)),
        y_axis=y_axis,
        z_axis=z_axis,
    )


def quaternion_from_frame(frame: OrientationFrame) -> Quaternion:
    m00, m10, m20 = frame.x_axis
    m01, m11, m21 = frame.y_axis
    m02, m12, m22 = frame.z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quaternion = Quaternion(
            (m21 - m12) / s,
            (m02 - m20) / s,
            (m10 - m01) / s,
            0.25 * s,
        )
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = Quaternion(
            0.25 * s,
            (m01 + m10) / s,
            (m02 + m20) / s,
            (m21 - m12) / s,
        )
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = Quaternion(
            (m01 + m10) / s,
            0.25 * s,
            (m12 + m21) / s,
            (m02 - m20) / s,
        )
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = Quaternion(
            (m02 + m20) / s,
            (m12 + m21) / s,
            0.25 * s,
            (m10 - m01) / s,
        )
    return normalize_quaternion(quaternion)


def angular_velocity_from_delta(
    previous: Quaternion,
    current: Quaternion,
    dt_sec: float,
) -> Vector3:
    current = current if quaternion_dot(previous, current) >= 0.0 else negate_quaternion(current)
    delta = quaternion_multiply(current, quaternion_inverse(previous))
    if delta.w < 0.0:
        delta = negate_quaternion(delta)
    xyz_norm = math.sqrt(delta.x**2 + delta.y**2 + delta.z**2)
    if xyz_norm < 1e-12:
        return 0.0, 0.0, 0.0
    angle = 2.0 * math.atan2(xyz_norm, clamp(delta.w, -1.0, 1.0))
    return (
        delta.x / xyz_norm * angle / dt_sec,
        delta.y / xyz_norm * angle / dt_sec,
        delta.z / xyz_norm * angle / dt_sec,
    )


def rotate_unit_vector_toward(
    current: Vector3,
    target: Vector3,
    max_angle: float,
) -> Vector3:
    current = normalize_vector(current)
    target = normalize_vector(target)
    angle = vector_angle(current, target)
    if angle <= max_angle or angle < 1e-12:
        return target
    if max_angle == 0.0:
        return current
    axis = cross(current, target)
    if vector_norm(axis) < 1e-9:
        axis = cross(current, least_aligned_axis(current))
    axis = normalize_vector(axis)
    cosine = math.cos(max_angle)
    sine = math.sin(max_angle)
    axis_cross = cross(axis, current)
    axis_dot = dot(axis, current)
    return normalize_vector(
        (
            current[0] * cosine + axis_cross[0] * sine + axis[0] * axis_dot * (1.0 - cosine),
            current[1] * cosine + axis_cross[1] * sine + axis[1] * axis_dot * (1.0 - cosine),
            current[2] * cosine + axis_cross[2] * sine + axis[2] * axis_dot * (1.0 - cosine),
        )
    )


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    q = (quaternion.x, quaternion.y, quaternion.z)
    uv = cross(q, vector)
    uuv = cross(q, uv)
    return add_vectors(
        vector,
        add_vectors(scale_vector(uv, 2.0 * quaternion.w), scale_vector(uuv, 2.0)),
    )


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def quaternion_inverse(quaternion: Quaternion) -> Quaternion:
    return Quaternion(-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w)


def normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(
        quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2
    )
    if norm == 0.0:
        raise ValueError("quaternion norm must be non-zero")
    return Quaternion(
        quaternion.x / norm,
        quaternion.y / norm,
        quaternion.z / norm,
        quaternion.w / norm,
    )


def negate_quaternion(quaternion: Quaternion) -> Quaternion:
    return Quaternion(
        -quaternion.x,
        -quaternion.y,
        -quaternion.z,
        -quaternion.w,
    )


def limit_vector_norm(vector: tuple[float, float], limit: float) -> tuple[float, float]:
    magnitude = math.hypot(*vector)
    if magnitude <= limit or magnitude == 0.0:
        return vector
    scale = limit / magnitude
    return vector[0] * scale, vector[1] * scale


def move_vector_toward(
    current: tuple[float, float],
    target: tuple[float, float],
    max_step: float,
) -> tuple[float, float]:
    delta = (target[0] - current[0], target[1] - current[1])
    magnitude = math.hypot(*delta)
    if magnitude <= max_step or magnitude == 0.0:
        return target
    scale = max_step / magnitude
    return current[0] + delta[0] * scale, current[1] + delta[1] * scale


def move_toward(current: float, target: float, max_step: float) -> float:
    if current < target:
        return min(current + max_step, target)
    if current > target:
        return max(current - max_step, target)
    return current


def project_onto_plane(vector: Vector3, normal: Vector3) -> Vector3:
    normal = normalize_vector(normal)
    return subtract_vectors(vector, scale_vector(normal, dot(vector, normal)))


def least_aligned_axis(vector: Vector3) -> Vector3:
    return min(
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        key=lambda axis: abs(dot(vector, axis)),
    )


def vector_angle(a: Vector3, b: Vector3) -> float:
    return math.acos(clamp(dot(normalize_vector(a), normalize_vector(b)), -1.0, 1.0))


def normalize_vector(vector: Vector3) -> Vector3:
    norm = vector_norm(vector)
    if norm == 0.0:
        raise ValueError("vector norm must be non-zero")
    return scale_vector(vector, 1.0 / norm)


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(dot(vector, vector))


def dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Vector3, b: Vector3) -> Vector3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def add_vectors(a: Vector3, b: Vector3) -> Vector3:
    return a[0] + b[0], a[1] + b[1], a[2] + b[2]


def subtract_vectors(a: Vector3, b: Vector3) -> Vector3:
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def scale_vector(vector: Vector3, scale: float) -> Vector3:
    return vector[0] * scale, vector[1] * scale, vector[2] * scale


def quaternion_dot(a: Quaternion, b: Quaternion) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w


def shortest_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def wrap_angle(value: float) -> float:
    wrapped = shortest_angle(value)
    return math.pi if math.isclose(wrapped, -math.pi) else wrapped


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _validate_range(low: float, high: float, name: str) -> None:
    if low < 0.0 or high <= low:
        raise ValueError(f"{name} range must satisfy 0 <= low < high")
