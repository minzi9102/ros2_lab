import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PlanarVelocity:
    x: float = 0.0
    y: float = 0.0


@dataclass(frozen=True)
class PaperBounds:
    width: float
    height: float

    def clamp_xy(self, x: float, y: float) -> tuple[float, float]:
        half_width = self.width / 2.0
        half_height = self.height / 2.0
        return (
            clamp(x, -half_width, half_width),
            clamp(y, -half_height, half_height),
        )


@dataclass(frozen=True)
class PenPose2D:
    tip_x: float
    tip_y: float
    yaw: float
    tilt_rad: float = 0.0


class SmoothPlanarVelocity:
    def __init__(
        self,
        *,
        max_speed_mps: float,
        acceleration_mps2: float,
        deceleration_mps2: float,
    ) -> None:
        if max_speed_mps <= 0.0:
            raise ValueError("max_planar_speed_mps must be greater than zero")
        if acceleration_mps2 <= 0.0:
            raise ValueError("acceleration_mps2 must be greater than zero")
        if deceleration_mps2 <= 0.0:
            raise ValueError("deceleration_mps2 must be greater than zero")

        self.max_speed_mps = float(max_speed_mps)
        self.acceleration_mps2 = float(acceleration_mps2)
        self.deceleration_mps2 = float(deceleration_mps2)
        self._vx = 0.0
        self._vy = 0.0

    def update(self, target_x: float, target_y: float, dt_sec: float) -> PlanarVelocity:
        if dt_sec <= 0.0:
            return self.current()

        target_vx, target_vy = self._target_velocity(target_x, target_y)
        target_is_zero = target_vx == 0.0 and target_vy == 0.0
        rate = self.deceleration_mps2 if target_is_zero else self.acceleration_mps2
        self._vx, self._vy = step_vector(
            self._vx,
            self._vy,
            target_vx,
            target_vy,
            rate * dt_sec,
        )
        self._clamp_current_speed()
        return self.current()

    def stop_immediately(self) -> PlanarVelocity:
        self._vx = 0.0
        self._vy = 0.0
        return self.current()

    def current(self) -> PlanarVelocity:
        return PlanarVelocity(x=self._vx, y=self._vy)

    def _target_velocity(self, target_x: float, target_y: float) -> tuple[float, float]:
        magnitude = math.hypot(target_x, target_y)
        if magnitude == 0.0:
            return 0.0, 0.0
        scale = self.max_speed_mps / magnitude
        return target_x * scale, target_y * scale

    def _clamp_current_speed(self) -> None:
        magnitude = math.hypot(self._vx, self._vy)
        if magnitude <= self.max_speed_mps:
            return
        scale = self.max_speed_mps / magnitude
        self._vx *= scale
        self._vy *= scale


class VirtualPenState:
    def __init__(
        self,
        *,
        initial_tip_x: float,
        initial_tip_y: float,
        initial_yaw: float,
        paper_bounds: PaperBounds,
        yaw_hold_speed_mps: float,
        target_tilt_rad: float = 0.0,
        tilt_activate_speed_mps: float = 0.0,
        tilt_rate_radps: float = 1.0,
        untilt_rate_radps: float = 1.0,
    ) -> None:
        if yaw_hold_speed_mps < 0.0:
            raise ValueError("yaw_hold_speed_mps must be non-negative")
        if target_tilt_rad < 0.0 or target_tilt_rad >= math.pi / 2.0:
            raise ValueError("target_tilt_rad must be in [0, pi/2)")
        if tilt_activate_speed_mps < 0.0:
            raise ValueError("tilt_activate_speed_mps must be non-negative")
        if tilt_rate_radps <= 0.0:
            raise ValueError("tilt_rate_radps must be greater than zero")
        if untilt_rate_radps <= 0.0:
            raise ValueError("untilt_rate_radps must be greater than zero")
        self._bounds = paper_bounds
        self._yaw_hold_speed_mps = float(yaw_hold_speed_mps)
        self._target_tilt_rad = float(target_tilt_rad)
        self._tilt_activate_speed_mps = float(tilt_activate_speed_mps)
        self._tilt_rate_radps = float(tilt_rate_radps)
        self._untilt_rate_radps = float(untilt_rate_radps)
        tip_x, tip_y = self._bounds.clamp_xy(initial_tip_x, initial_tip_y)
        self._pose = PenPose2D(tip_x=tip_x, tip_y=tip_y, yaw=initial_yaw)

    @property
    def pose(self) -> PenPose2D:
        return self._pose

    def update(self, velocity: PlanarVelocity, dt_sec: float) -> PenPose2D:
        if dt_sec < 0.0:
            raise ValueError("dt_sec must be non-negative")

        tip_x = self._pose.tip_x + velocity.x * dt_sec
        tip_y = self._pose.tip_y + velocity.y * dt_sec
        tip_x, tip_y = self._bounds.clamp_xy(tip_x, tip_y)

        speed = math.hypot(velocity.x, velocity.y)
        yaw = self._pose.yaw
        if speed >= self._yaw_hold_speed_mps and speed > 0.0:
            yaw = desired_pen_tail_yaw(velocity.x, velocity.y)

        target_tilt = (
            self._target_tilt_rad
            if speed >= self._tilt_activate_speed_mps and speed > 0.0
            else 0.0
        )
        rate = (
            self._tilt_rate_radps
            if target_tilt > self._pose.tilt_rad
            else self._untilt_rate_radps
        )
        tilt_rad = move_toward(
            self._pose.tilt_rad,
            target_tilt,
            rate * dt_sec,
        )

        self._pose = PenPose2D(
            tip_x=tip_x,
            tip_y=tip_y,
            yaw=yaw,
            tilt_rad=tilt_rad,
        )
        return self._pose


def desired_pen_tail_yaw(linear_x: float, linear_y: float) -> float:
    yaw = math.atan2(linear_y, linear_x)
    if math.isclose(yaw, -math.pi):
        return math.pi
    return yaw


def pen_axis_vector(
    *,
    tail_yaw: float,
    tilt_rad: float,
    pen_length: float,
) -> tuple[float, float, float]:
    if pen_length <= 0.0:
        raise ValueError("pen_length must be greater than zero")
    if tilt_rad < 0.0 or tilt_rad >= math.pi / 2.0:
        raise ValueError("tilt_rad must be in [0, pi/2)")

    horizontal = math.sin(tilt_rad) * pen_length
    vertical = math.cos(tilt_rad) * pen_length
    return (
        horizontal * math.cos(tail_yaw),
        horizontal * math.sin(tail_yaw),
        vertical,
    )


def step_vector(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
    max_step: float,
) -> tuple[float, float]:
    if max_step < 0.0:
        raise ValueError("max_step must be non-negative")
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    delta_magnitude = math.hypot(delta_x, delta_y)
    if delta_magnitude <= max_step + 1e-12:
        return target_x, target_y
    if delta_magnitude == 0.0:
        return current_x, current_y
    scale = max_step / delta_magnitude
    return (
        current_x + delta_x * scale,
        current_y + delta_y * scale,
    )


def move_toward(current: float, target: float, max_step: float) -> float:
    if max_step < 0.0:
        raise ValueError("max_step must be non-negative")
    if current < target:
        return min(current + max_step, target)
    if current > target:
        return max(current - max_step, target)
    return current


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
