import os
import select
from dataclasses import dataclass
from enum import Enum
from typing import List


class KeyEventValue(Enum):
    UP = 0
    DOWN = 1
    REPEAT = 2


@dataclass(frozen=True)
class EvdevKeyEvent:
    key_name: str
    value: KeyEventValue


class EvdevKeyReader:
    def __init__(self, device_path: str, *, evdev_module=None) -> None:
        self._device_path = device_path
        self._evdev = evdev_module
        self._device = None

    @property
    def source_name(self) -> str:
        return self._device_path

    def start(self) -> None:
        if self._device is not None:
            return

        self._validate_device_path()
        if self._evdev is None:
            try:
                import evdev
            except ImportError as exc:
                raise RuntimeError(
                    'evdev input requires python3-evdev; install it before starting'
                ) from exc
            self._evdev = evdev

        try:
            device = self._evdev.InputDevice(self._device_path)
        except OSError as exc:
            raise RuntimeError(
                f'failed to open evdev input device {self._device_path}: {exc}'
            ) from exc

        capabilities = device.capabilities()
        key_codes = capabilities.get(self._evdev.ecodes.EV_KEY, [])
        required_codes = {
            self._evdev.ecodes.KEY_W,
            self._evdev.ecodes.KEY_S,
            self._evdev.ecodes.KEY_A,
            self._evdev.ecodes.KEY_D,
        }
        if not required_codes.intersection(key_codes):
            device.close()
            raise RuntimeError(
                f'evdev input device is not a supported keyboard: {self._device_path}'
            )

        self._device = device

    def close(self) -> None:
        if self._device is not None:
            self._device.close()
            self._device = None

    def read_events(self) -> List[EvdevKeyEvent]:
        if self._device is None:
            return []

        readable, _, _ = select.select([self._device.fd], [], [], 0.0)
        if not readable:
            return []

        events = []
        for event in self._device.read():
            if event.type != self._evdev.ecodes.EV_KEY:
                continue
            key_name = self._evdev.ecodes.KEY.get(event.code)
            if isinstance(key_name, (list, tuple)):
                key_name = key_name[0] if key_name else None
            if key_name is None:
                continue
            try:
                value = KeyEventValue(event.value)
            except ValueError:
                continue
            events.append(EvdevKeyEvent(str(key_name), value))
        return events

    def _validate_device_path(self) -> None:
        if not self._device_path:
            raise RuntimeError('input_device is required when input_backend=evdev')
        if not os.path.exists(self._device_path):
            raise RuntimeError(f'evdev input device does not exist: {self._device_path}')
        if not os.access(self._device_path, os.R_OK):
            raise RuntimeError(
                f'evdev input device is not readable: {self._device_path}; '
                'check input group membership'
            )
