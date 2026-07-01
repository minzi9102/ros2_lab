import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class JoyIntent:
    x: float = 0.0
    y: float = 0.0
    emergency_stop: bool = False
    quit_requested: bool = False


class JoyMapper:
    def __init__(self, deadzone: float) -> None:
        if not 0.0 <= deadzone < 1.0:
            raise ValueError("deadzone must be in [0, 1)")
        self.deadzone = float(deadzone)

    def map(self, axes: Sequence[float], buttons: Sequence[int]) -> JoyIntent:
        a_pressed = len(buttons) > 0 and bool(buttons[0])
        b_pressed = len(buttons) > 1 and bool(buttons[1])
        if a_pressed or b_pressed:
            return JoyIntent(
                emergency_stop=True,
                quit_requested=b_pressed,
            )
        if len(axes) < 2:
            return JoyIntent()

        x = -max(-1.0, min(1.0, float(axes[1])))
        y = -max(-1.0, min(1.0, float(axes[0])))
        magnitude = math.hypot(x, y)
        if magnitude <= self.deadzone:
            return JoyIntent()
        if magnitude > 1.0:
            x /= magnitude
            y /= magnitude
        return JoyIntent(x=x, y=y)
