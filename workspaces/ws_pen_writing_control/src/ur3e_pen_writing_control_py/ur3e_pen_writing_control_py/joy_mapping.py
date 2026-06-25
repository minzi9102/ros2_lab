import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class JoyControl:
    target_x: float = 0.0
    target_y: float = 0.0
    emergency_stop: bool = False
    quit_requested: bool = False


class JoyMapper:
    def __init__(self, *, deadzone: float) -> None:
        if deadzone < 0.0 or deadzone >= 1.0:
            raise ValueError("joy deadzone must be in [0.0, 1.0)")
        self.deadzone = float(deadzone)

    def map(self, axes: Sequence[float], buttons: Sequence[int]) -> JoyControl:
        a_pressed = self._button_pressed(buttons, 0)
        b_pressed = self._button_pressed(buttons, 1)
        if a_pressed or b_pressed:
            return JoyControl(
                emergency_stop=True,
                quit_requested=b_pressed,
            )

        if len(axes) < 2:
            return JoyControl()

        target_x = -self._axis_value(axes[1])
        target_y = -self._axis_value(axes[0])
        magnitude = math.hypot(target_x, target_y)
        if magnitude <= self.deadzone:
            return JoyControl()
        if magnitude > 1.0:
            target_x /= magnitude
            target_y /= magnitude
        return JoyControl(target_x=target_x, target_y=target_y)

    @staticmethod
    def _button_pressed(buttons: Sequence[int], index: int) -> bool:
        return len(buttons) > index and int(buttons[index]) != 0

    @staticmethod
    def _axis_value(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))
