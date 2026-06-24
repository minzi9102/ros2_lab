import math

from .safety_limiter import TwistCommand


class SmoothVelocityController:
    def __init__(
        self,
        *,
        linear_speed_mps: float,
        acceleration_mps2: float,
        deceleration_mps2: float,
    ) -> None:
        if linear_speed_mps <= 0.0:
            raise ValueError('linear_speed_mps must be greater than zero')
        if acceleration_mps2 <= 0.0:
            raise ValueError('acceleration_mps2 must be greater than zero')
        if deceleration_mps2 <= 0.0:
            raise ValueError('deceleration_mps2 must be greater than zero')

        self.linear_speed_mps = float(linear_speed_mps)
        self.acceleration_mps2 = float(acceleration_mps2)
        self.deceleration_mps2 = float(deceleration_mps2)
        self._linear_x = 0.0
        self._linear_y = 0.0

    def update(self, target_x: float, target_y: float, dt_sec: float) -> TwistCommand:
        if dt_sec <= 0.0:
            return self.current_command()

        target_x, target_y = self._target_velocity(target_x, target_y)
        if self._is_opposite_direction(target_x, target_y):
            target_x, target_y = 0.0, 0.0

        target_is_zero = target_x == 0.0 and target_y == 0.0
        rate = self.deceleration_mps2 if target_is_zero else self.acceleration_mps2
        self._linear_x, self._linear_y = self._step_vector(
            target_x,
            target_y,
            rate * dt_sec,
        )
        self._clamp_current_speed()
        return self.current_command()

    def stop_immediately(self) -> TwistCommand:
        self._linear_x = 0.0
        self._linear_y = 0.0
        return self.current_command()

    def current_command(self) -> TwistCommand:
        return TwistCommand(linear_x=self._linear_x, linear_y=self._linear_y)

    def _target_velocity(
        self,
        target_x: float,
        target_y: float,
    ) -> tuple[float, float]:
        magnitude = math.hypot(target_x, target_y)
        if magnitude == 0.0:
            return 0.0, 0.0
        scale = self.linear_speed_mps / magnitude
        return target_x * scale, target_y * scale

    def _is_opposite_direction(self, target_x: float, target_y: float) -> bool:
        current_magnitude = math.hypot(self._linear_x, self._linear_y)
        target_magnitude = math.hypot(target_x, target_y)
        if current_magnitude == 0.0 or target_magnitude == 0.0:
            return False

        dot = self._linear_x * target_x + self._linear_y * target_y
        cross = self._linear_x * target_y - self._linear_y * target_x
        return dot < 0.0 and abs(cross) <= 1e-9

    def _step_vector(
        self,
        target_x: float,
        target_y: float,
        max_step: float,
    ) -> tuple[float, float]:
        delta_x = target_x - self._linear_x
        delta_y = target_y - self._linear_y
        delta_magnitude = math.hypot(delta_x, delta_y)
        if delta_magnitude <= max_step + 1e-12:
            return target_x, target_y

        scale = max_step / delta_magnitude
        return (
            self._linear_x + delta_x * scale,
            self._linear_y + delta_y * scale,
        )

    def _clamp_current_speed(self) -> None:
        magnitude = math.hypot(self._linear_x, self._linear_y)
        if magnitude <= self.linear_speed_mps:
            return
        scale = self.linear_speed_mps / magnitude
        self._linear_x *= scale
        self._linear_y *= scale
