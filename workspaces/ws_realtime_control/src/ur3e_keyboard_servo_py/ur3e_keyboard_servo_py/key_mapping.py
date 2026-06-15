from dataclasses import dataclass
from enum import Enum
from typing import Optional


class KeyAction(Enum):
    MOVE = 'move'
    STOP = 'stop'
    QUIT = 'quit'
    IGNORE = 'ignore'


@dataclass(frozen=True)
class KeyCommand:
    action: KeyAction
    x: float = 0.0
    y: float = 0.0

    @property
    def is_motion(self) -> bool:
        return self.action == KeyAction.MOVE


KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'
KEY_RIGHT = '\x1b[C'
KEY_LEFT = '\x1b[D'


def map_key(raw_key: Optional[str]) -> KeyCommand:
    if raw_key is None or raw_key == '':
        return KeyCommand(KeyAction.IGNORE)

    if raw_key == KEY_UP:
        return KeyCommand(KeyAction.MOVE, x=1.0)
    if raw_key == KEY_DOWN:
        return KeyCommand(KeyAction.MOVE, x=-1.0)
    if raw_key == KEY_LEFT:
        return KeyCommand(KeyAction.MOVE, y=1.0)
    if raw_key == KEY_RIGHT:
        return KeyCommand(KeyAction.MOVE, y=-1.0)
    if raw_key == ' ':
        return KeyCommand(KeyAction.STOP)

    normalized = raw_key.lower()

    if normalized == 'w':
        return KeyCommand(KeyAction.MOVE, x=1.0)
    if normalized == 's':
        return KeyCommand(KeyAction.MOVE, x=-1.0)
    if normalized == 'a':
        return KeyCommand(KeyAction.MOVE, y=1.0)
    if normalized == 'd':
        return KeyCommand(KeyAction.MOVE, y=-1.0)
    if normalized == 'q':
        return KeyCommand(KeyAction.QUIT)

    return KeyCommand(KeyAction.IGNORE)
