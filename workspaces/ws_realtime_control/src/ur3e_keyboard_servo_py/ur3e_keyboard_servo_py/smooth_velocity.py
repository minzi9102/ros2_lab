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

        target_x = self._clamp_unit(target_x) * self.linear_speed_mps
        target_y = self._clamp_unit(target_y) * self.linear_speed_mps

        if self._must_stop_before_changing_direction(target_x, target_y):
            target_x = 0.0
            target_y = 0.0

        self._linear_x = self._step_axis(self._linear_x, target_x, dt_sec)
        self._linear_y = self._step_axis(self._linear_y, target_y, dt_sec)
        return self.current_command()

    def stop_immediately(self) -> TwistCommand:
        self._linear_x = 0.0
        self._linear_y = 0.0
        return self.current_command()

    def current_command(self) -> TwistCommand:
        return TwistCommand(linear_x=self._linear_x, linear_y=self._linear_y)

    def _must_stop_before_changing_direction(
        self,
        target_x: float,
        target_y: float,
    ) -> bool:
        current_is_zero = self._linear_x == 0.0 and self._linear_y == 0.0
        target_is_zero = target_x == 0.0 and target_y == 0.0
        if current_is_zero or target_is_zero:
            return False

        current_axis = 'x' if self._linear_x != 0.0 else 'y'
        target_axis = 'x' if target_x != 0.0 else 'y'
        if current_axis != target_axis:
            return True

        current_value = self._linear_x if current_axis == 'x' else self._linear_y
        target_value = target_x if target_axis == 'x' else target_y
        return current_value * target_value < 0.0

    def _step_axis(self, current: float, target: float, dt_sec: float) -> float:
        if current == target:
            return current

        speeding_up = (
            current == 0.0
            or current * target > 0.0
            and abs(target) > abs(current)
        )
        rate = self.acceleration_mps2 if speeding_up else self.deceleration_mps2
        max_step = rate * dt_sec
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + max_step if delta > 0.0 else current - max_step

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))
