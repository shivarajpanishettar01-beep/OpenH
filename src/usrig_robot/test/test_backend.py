"""RtdeBackend against a fake rtde_control module: no robot, no network."""
import sys
import types

import numpy as np
import pytest

from usrig_robot.backends import RtdeBackend, TcpMismatchError


class FakeControl:
    FLAG_UPLOAD_SCRIPT = 1
    calls = []

    def __init__(self, ip, freq, *args):
        self.tcp = [0.0, 0.0, 0.2, 0.0, 0.0, 0.0]
        FakeControl.calls.append(('init', ip, freq, args))

    def __getattr__(self, name):
        def f(*a):
            FakeControl.calls.append((name, a))
            return [0.0, 0.0, 0.2, 0.0, 0.0, 0.0] if name == 'getTCPOffset' else True
        return f


@pytest.fixture(autouse=True)
def fake_rtde(monkeypatch):
    FakeControl.calls = []
    monkeypatch.setitem(sys.modules, 'rtde_control', types.SimpleNamespace(RTDEControlInterface=FakeControl))


def names():
    return [c[0] for c in FakeControl.calls]


def test_watchdog_set_from_deadman_and_kicked():
    b = RtdeBackend('1.2.3.4', 125, [0.1] * 6, watchdog_min_hz=1 / 0.2, stop_decel=1.5,
                    expected_tcp=[0, 0, 0.2, 0, 0, 0], rt_priority=80, log=lambda m: None)
    b.connect()
    assert ('setWatchdog', (5.0,)) in FakeControl.calls
    assert FakeControl.calls[0][3][-1] == 80          # rt_priority passed to RTDEControlInterface
    b.kick()
    assert names().count('kickWatchdog') == 1
    b.stop()
    assert ('speedStop', (1.5,)) in FakeControl.calls


def test_tcp_mismatch_refuses():
    b = RtdeBackend('1.2.3.4', 125, [0.1] * 6, expected_tcp=[0, 0, 0.15, 0, 0, 0], log=lambda m: None)
    with pytest.raises(TcpMismatchError):
        b.connect()
    assert 'disconnect' in names()


def test_force_mode_then_stop():
    b = RtdeBackend('1.2.3.4', 125, [0.1] * 6, log=lambda m: None)
    b.connect()
    b.command(np.eye(4), [0.01, 0, 0], [0, 0, 0], 0.5, True, 4.0, 0.008)
    b.command(np.eye(4), [0, 0, 0], [0, 0, 0], 0.5, False, 0.0, 0.008)
    n = names()
    assert n.index('forceMode') < n.index('forceModeStop') and n.count('speedL') == 2
