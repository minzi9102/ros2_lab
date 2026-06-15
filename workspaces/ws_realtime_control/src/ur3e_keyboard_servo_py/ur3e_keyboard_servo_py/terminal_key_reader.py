import atexit
import select
import sys
import termios
import tty
from typing import Optional, TextIO


class TerminalKeyReader:
    def __init__(
        self,
        stream: TextIO = sys.stdin,
        *,
        fallback_device: Optional[str] = '/dev/tty',
    ) -> None:
        self._stream = self._select_stream(stream, fallback_device)
        self._owns_stream = self._stream is not stream
        self._fd = self._stream.fileno()
        self._original_settings = None
        self._active = False
        self._interactive = self._stream.isatty()

    @property
    def is_interactive(self) -> bool:
        return self._interactive

    @property
    def source_name(self) -> str:
        if self._stream is sys.stdin:
            return 'stdin'
        return getattr(self._stream, 'name', 'custom stream')

    def __enter__(self) -> 'TerminalKeyReader':
        self.start()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    def start(self) -> None:
        if self._active:
            return

        if not self._interactive:
            return

        self._original_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._active = True
        atexit.register(self.close)

    def close(self) -> None:
        if not self._active:
            if self._owns_stream:
                self._stream.close()
                self._owns_stream = False
            return

        if self._original_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._original_settings)
        self._active = False

        if self._owns_stream:
            self._stream.close()
            self._owns_stream = False

    def read_key(self) -> Optional[str]:
        if not self._interactive:
            return None

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

    @staticmethod
    def _select_stream(stream: TextIO, fallback_device: Optional[str]) -> TextIO:
        if stream.isatty() or fallback_device is None:
            return stream

        try:
            fallback_stream = open(fallback_device, 'r', buffering=1)
        except OSError:
            return stream

        if fallback_stream.isatty():
            return fallback_stream

        fallback_stream.close()
        return stream
