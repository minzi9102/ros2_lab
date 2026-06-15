import atexit
import select
import sys
import termios
import tty
from typing import Optional, TextIO


class TerminalKeyReader:
    def __init__(self, stream: TextIO = sys.stdin) -> None:
        self._stream = stream
        self._fd = stream.fileno()
        self._original_settings = None
        self._active = False

    def __enter__(self) -> 'TerminalKeyReader':
        self.start()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def start(self) -> None:
        if self._active:
            return

        self._original_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._active = True
        atexit.register(self.close)

    def close(self) -> None:
        if not self._active:
            return

        if self._original_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._original_settings)
        self._active = False

    def read_key(self) -> Optional[str]:
        readable, _, _ = select.select([self._stream], [], [], 0.0)
        if not readable:
            return None

        first = self._stream.read(1)
        if first != '\x1b':
            return first

        sequence = first
        while True:
            readable, _, _ = select.select([self._stream], [], [], 0.0)
            if not readable:
                break
            sequence += self._stream.read(1)
            if len(sequence) >= 3:
                break
        return sequence
