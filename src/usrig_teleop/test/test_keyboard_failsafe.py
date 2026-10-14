"""keyboard_teleop with a fake evdev device: stuck keys and device loss must never keep the robot moving."""
import os
import shutil
import sys
import threading
import time
import types
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]


class FakeEvent:
    def __init__(self, type_, code, value):
        self.type, self.code, self.value = type_, code, value
        self.sec, self.usec = int(time.time()), 0


class FakeDevice:
    def __init__(self, hub):
        self.hub = hub
        self.path, self.name = 'fake-kbd', 'fake keyboard'
        self.fd = hub.r

    def capabilities(self):
        return {1: [17, 57]}

    def read(self):
        if self.hub.fail:
            raise OSError(19, 'No such device')
        os.read(self.hub.r, 1)
        evs, self.hub.queue = self.hub.queue, []
        return evs

    def active_keys(self):
        return list(self.hub.active)


class Hub:
    def __init__(self):
        self.r, self.w = os.pipe()
        self.queue, self.active, self.fail, self.available = [], set(), False, True

    def push(self, *evs):
        self.queue += list(evs)
        os.write(self.w, b'x')

    def break_device(self):
        self.fail = True
        self.available = False
        os.write(self.w, b'x')


@pytest.fixture
def hub(monkeypatch, tmp_path):
    h = Hub()
    ec = types.SimpleNamespace(EV_SYN=0, EV_KEY=1, SYN_DROPPED=3, KEY_W=17, KEY_SPACE=57,
                               KEY={17: 'KEY_W', 57: 'KEY_SPACE', 42: 'KEY_LEFTSHIFT', 29: 'KEY_LEFTCTRL'})

    def input_device(path):
        if not h.available:
            raise OSError(2, 'gone')
        h.fail = False
        return FakeDevice(h)
    monkeypatch.setitem(sys.modules, 'evdev', types.SimpleNamespace(
        ecodes=ec, InputDevice=input_device, list_devices=lambda: ['fake-kbd']))
    cfg = tmp_path / 'config'
    shutil.copytree(REPO / 'src/usrig_bringup/config', cfg)
    km = yaml.safe_load((cfg / 'keymap.yaml').read_text())   # test-only values, not the rig's
    km['speeds'] = {'slide_along_mm_s': 10, 'slide_across_mm_s': 10, 'rock_deg_s': 5, 'tilt_deg_s': 5, 'rotate_deg_s': 5}
    km['device'] = 'fake-kbd'
    (cfg / 'keymap.yaml').write_text(yaml.safe_dump(km))
    rig = yaml.safe_load((cfg / 'rig.yaml').read_text())
    rig['limits']['max_linear_speed_mm_s'] = 20
    rig['limits']['max_angular_speed_deg_s'] = 10
    (cfg / 'rig.yaml').write_text(yaml.safe_dump(rig))
    monkeypatch.setenv('USRIG_CONFIG_DIR', str(cfg))
    return h


def test_device_loss_stops_twists_and_reconnects(hub):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.executors import MultiThreadedExecutor
    from usrig_teleop.keyboard_teleop import KeyboardTeleop
    rclpy.init()
    try:
        node = KeyboardTeleop()
        listen = rclpy.create_node('listen')
        got = []
        listen.create_subscription(TwistStamped, '/teleop/twist', lambda m: got.append((time.monotonic(), m.twist.linear.x)), 100)
        ex = MultiThreadedExecutor()
        ex.add_node(node)
        ex.add_node(listen)
        threading.Thread(target=ex.spin, daemon=True).start()
        time.sleep(0.5)
        hub.push(FakeEvent(1, 17, 1))                     # W alone: no enable held -> no motion (D2)
        time.sleep(0.4)
        assert max(abs(v) for _, v in got) == 0.0
        hub.push(FakeEvent(1, 29, 1))                     # Ctrl held (enable) + W: slide along +x
        time.sleep(0.6)
        assert max(v for _, v in got) == pytest.approx(0.010, rel=0.05)   # ramped to 10 mm/s
        hub.break_device()                                # unplugged while W is held
        time.sleep(0.4)
        t_lost = time.monotonic()
        n_at_loss = len(got)
        time.sleep(0.8)
        after = [v for t, v in got if t > t_lost]
        assert after == [], f'twists kept coming after the device was lost: {after[:5]}'
        assert got[n_at_loss - 1][1] == 0.0               # the last twist before silence is zero
        hub.available = True                               # plugged back in, no key held
        time.sleep(1.8)
        resumed = [v for t, v in got if t > t_lost]
        assert resumed and all(v == 0.0 for v in resumed)  # publishing again, robot not moving
    finally:
        rclpy.shutdown()


def test_keystate_resync_and_loss():
    from usrig_teleop.ramp import KeyState
    k = KeyState()
    k.key('W', True)
    k.key('A', True)
    k.resync(['A'])                                       # W's key-up was dropped by the kernel
    assert k.held == {'A'}
    k.device_lost()
    assert k.held == set() and k.lost
    k.device_back([])
    assert not k.lost


def test_motion_needs_a_dedicated_keyboard_device():
    """device: null + Ctrl hold-to-move: Ctrl+C / Ctrl+Z / Ctrl+S typed in any terminal would move the arm."""
    from usrig_teleop.keyboard_teleop import check_device
    with pytest.raises(ValueError, match='device is null'):
        check_device({'device': None}, publish_twist=True)
    assert check_device({'device': None, 'allow_any_keyboard': True}, publish_twist=True)      # warns
    assert check_device({'device': None}, publish_twist=False)       # commands only: warning, no refusal
    assert check_device({'device': '/dev/input/by-id/usb-op-event-kbd'}, publish_twist=True) is None
