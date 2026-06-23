from dataclasses import dataclass
from enum import Enum
from typing import Dict, Set, Tuple

from .evdev_key_reader import EvdevKeyEvent, KeyEventValue


class Direction(Enum):
    X_POSITIVE = 'x_positive'
    X_NEGATIVE = 'x_negative'
    Y_POSITIVE = 'y_positive'
    Y_NEGATIVE = 'y_negative'


KEY_DIRECTIONS: Dict[str, Direction] = {
    'KEY_W': Direction.X_POSITIVE,
    'KEY_UP': Direction.X_POSITIVE,
    'KEY_S': Direction.X_NEGATIVE,
    'KEY_DOWN': Direction.X_NEGATIVE,
    'KEY_A': Direction.Y_POSITIVE,
    'KEY_LEFT': Direction.Y_POSITIVE,
    'KEY_D': Direction.Y_NEGATIVE,
    'KEY_RIGHT': Direction.Y_NEGATIVE,
}


@dataclass(frozen=True)
class InputDecision:
    emergency_stop: bool = False
    quit_requested: bool = False


class PressedKeyState:
    def __init__(self) -> None:
        self._pressed_keys: Set[str] = set()

    def apply(self, event: EvdevKeyEvent) -> InputDecision:
        if event.value == KeyEventValue.REPEAT:
            return InputDecision()

        if event.key_name == 'KEY_SPACE' and event.value == KeyEventValue.DOWN:
            self.clear()
            return InputDecision(emergency_stop=True)

        if event.key_name == 'KEY_Q' and event.value == KeyEventValue.DOWN:
            self.clear()
            return InputDecision(emergency_stop=True, quit_requested=True)

        if event.key_name not in KEY_DIRECTIONS:
            return InputDecision()

        if event.value == KeyEventValue.DOWN:
            self._pressed_keys.add(event.key_name)
        elif event.value == KeyEventValue.UP:
            self._pressed_keys.discard(event.key_name)
        return InputDecision()

    def clear(self) -> None:
        self._pressed_keys.clear()

    def target_axes(self) -> Tuple[float, float]:
        directions = {KEY_DIRECTIONS[key] for key in self._pressed_keys}
        if len(directions) != 1:
            return 0.0, 0.0

        direction = next(iter(directions))
        if direction == Direction.X_POSITIVE:
            return 1.0, 0.0
        if direction == Direction.X_NEGATIVE:
            return -1.0, 0.0
        if direction == Direction.Y_POSITIVE:
            return 0.0, 1.0
        return 0.0, -1.0
