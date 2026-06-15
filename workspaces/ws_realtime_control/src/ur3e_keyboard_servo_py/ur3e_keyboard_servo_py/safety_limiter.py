from dataclasses import dataclass
from typing import Optional

from .key_mapping import KeyAction, KeyCommand


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0

    @property
    def is_zero(self) -> bool:
        return (
            self.linear_x == 0.0
            and self.linear_y == 0.0
            and self.linear_z == 0.0
            and self.angular_x == 0.0
            and self.angular_y == 0.0
            and self.angular_z == 0.0
        )


class SafetyLimiter:
    def __init__(
        self,
        *,
        linear_speed_mps: float,
        key_timeout_sec: float,
        enable_z: bool = False,
        enable_rotation: bool = False,
    ) -> None:
        if linear_speed_mps < 0.0:
            raise ValueError('linear_speed_mps must be non-negative')
        if key_timeout_sec < 0.0:
            raise ValueError('key_timeout_sec must be non-negative')

        self.linear_speed_mps = float(linear_speed_mps)
        self.key_timeout_sec = float(key_timeout_sec)
        self.enable_z = bool(enable_z)
        self.enable_rotation = bool(enable_rotation)
        self._active_command = TwistCommand()
        self._last_motion_time: Optional[float] = None

    def apply_key_command(self, command: KeyCommand, now_sec: float) -> TwistCommand:
        if command.action == KeyAction.MOVE:
            self._active_command = TwistCommand(
                linear_x=self._clamp_unit(command.x) * self.linear_speed_mps,
                linear_y=self._clamp_unit(command.y) * self.linear_speed_mps,
                linear_z=0.0,
                angular_x=0.0,
                angular_y=0.0,
                angular_z=0.0,
            )
            self._last_motion_time = now_sec
            return self._active_command

        if command.action in (KeyAction.STOP, KeyAction.QUIT):
            return self.stop()

        return self.current_command(now_sec)

    def current_command(self, now_sec: float) -> TwistCommand:
        if self._last_motion_time is None:
            return TwistCommand()

        if now_sec - self._last_motion_time > self.key_timeout_sec:
            return self.stop()

        return self._active_command

    def stop(self) -> TwistCommand:
        self._active_command = TwistCommand()
        self._last_motion_time = None
        return self._active_command

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))
