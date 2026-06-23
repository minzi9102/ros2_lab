from types import SimpleNamespace

import pytest

from ur3e_keyboard_servo_py.evdev_key_reader import (
    EvdevKeyReader,
    KeyEventValue,
)


class FakeDevice:
    def __init__(self, _path, key_codes, events=None):
        self._key_codes = key_codes
        self._events = events or []
        self.closed = False
        self.fd = 42

    def capabilities(self):
        return {1: self._key_codes}

    def read(self):
        return self._events

    def close(self):
        self.closed = True


def fake_evdev(key_codes, events=None):
    ecodes = SimpleNamespace(
        EV_KEY=1,
        KEY_W=17,
        KEY_S=31,
        KEY_A=30,
        KEY_D=32,
        KEY={17: 'KEY_W'},
    )
    return SimpleNamespace(
        ecodes=ecodes,
        InputDevice=lambda path: FakeDevice(path, key_codes, events),
    )


def test_missing_device_is_rejected(tmp_path):
    reader = EvdevKeyReader(
        str(tmp_path / 'missing-event'),
        evdev_module=fake_evdev([17]),
    )

    with pytest.raises(RuntimeError, match='does not exist'):
        reader.start()


def test_unreadable_device_is_rejected(tmp_path, monkeypatch):
    device_path = tmp_path / 'event-kbd'
    device_path.touch()
    monkeypatch.setattr('os.access', lambda *_args: False)
    reader = EvdevKeyReader(str(device_path), evdev_module=fake_evdev([17]))

    with pytest.raises(RuntimeError, match='not readable'):
        reader.start()


def test_non_keyboard_device_is_rejected(tmp_path):
    device_path = tmp_path / 'event-mouse'
    device_path.touch()
    reader = EvdevKeyReader(str(device_path), evdev_module=fake_evdev([]))

    with pytest.raises(RuntimeError, match='not a supported keyboard'):
        reader.start()


def test_supported_keyboard_device_opens_and_closes(tmp_path):
    device_path = tmp_path / 'event-kbd'
    device_path.touch()
    reader = EvdevKeyReader(str(device_path), evdev_module=fake_evdev([17, 31]))

    reader.start()
    assert reader.source_name == str(device_path)
    reader.close()


def test_linux_key_events_are_converted_to_internal_values(tmp_path, monkeypatch):
    device_path = tmp_path / 'event-kbd'
    device_path.touch()
    events = [
        SimpleNamespace(type=1, code=17, value=1),
        SimpleNamespace(type=1, code=17, value=2),
        SimpleNamespace(type=1, code=17, value=0),
    ]
    reader = EvdevKeyReader(
        str(device_path),
        evdev_module=fake_evdev([17], events),
    )
    monkeypatch.setattr(
        'ur3e_keyboard_servo_py.evdev_key_reader.select.select',
        lambda *_args: ([42], [], []),
    )
    reader.start()

    converted = reader.read_events()

    assert [event.value for event in converted] == [
        KeyEventValue.DOWN,
        KeyEventValue.REPEAT,
        KeyEventValue.UP,
    ]
