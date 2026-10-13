import shutil
import threading
import time
from pathlib import Path

import numpy as np
import pytest
import yaml

from usrig_teleop.spacemouse_teleop import map_joy

REPO = Path(__file__).resolve().parents[3]
SM = yaml.safe_load((REPO / 'src/usrig_bringup/config/keymap.yaml').read_text())['spacemouse']
SPEEDS = [0.01, 0.01, 0.0, 0.1, 0.1, 0.1]


def test_map_requires_enable_and_respects_deadband():
    axes = [0.5, 0, 0, 0, 0, 0]
    v, f = map_joy(axes, [0, 0], SM, SPEEDS, 0.25)
    assert np.all(v == 0) and f == 0                   # enable not held
    v, f = map_joy(axes, [1, 0], SM, SPEEDS, 0.25)
    assert v[0] == pytest.approx(0.01 * (0.5 - 0.08) / 0.92) and np.count_nonzero(v) == 1
    v, _ = map_joy([0.05, 0, 0, 0, 0, 0], [1, 0], SM, SPEEDS, 0.25)
    assert np.all(v == 0)                              # inside the deadband
    v, _ = map_joy([1, 0, 0, 0, 0, 0], [1, 1], SM, SPEEDS, 0.25)
    assert v[0] == pytest.approx(0.0025)               # fine button
    _, f = map_joy([0, 0, -1, 0, 0, 0], [1, 0], SM, SPEEDS, 0.25)
    assert f == pytest.approx(1.0)                     # push down = compress


@pytest.fixture
def cfg(tmp_path, monkeypatch):
    c = tmp_path / 'config'
    shutil.copytree(REPO / 'src/usrig_bringup/config', c)
    km = yaml.safe_load((c / 'keymap.yaml').read_text())          # test-only values
    km['speeds'] = {'slide_along_mm_s': 10, 'slide_across_mm_s': 10, 'rock_deg_s': 5, 'tilt_deg_s': 5, 'rotate_deg_s': 5}
    km['spacemouse']['force_rate_n_s'] = 2.0
    (c / 'keymap.yaml').write_text(yaml.safe_dump(km))
    rig = yaml.safe_load((c / 'rig.yaml').read_text())
    rig['limits']['max_linear_speed_mm_s'] = 20
    rig['limits']['max_angular_speed_deg_s'] = 10
    (c / 'rig.yaml').write_text(yaml.safe_dump(rig))
    monkeypatch.setenv('USRIG_CONFIG_DIR', str(c))


def test_node_enable_release_and_driver_loss(cfg):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
    from usrig_teleop.spacemouse_teleop import SpaceMouseTeleop
    rclpy.init()
    try:
        node = SpaceMouseTeleop()
        aux = rclpy.create_node('aux')
        joy_pub = aux.create_publisher(Joy, '/spacenav/joy', 10)
        tw, cmds = [], []
        aux.create_subscription(TwistStamped, '/teleop/twist', lambda m: tw.append((time.monotonic(), m.twist.linear.x)), 100)
        aux.create_subscription(String, '/operator/command', lambda m: cmds.append(m.data), 100)
        ex = MultiThreadedExecutor()
        ex.add_node(node)
        ex.add_node(aux)
        threading.Thread(target=ex.spin, daemon=True).start()
        time.sleep(0.5)

        def pump(axes, buttons, seconds):
            t_end = time.monotonic() + seconds
            while time.monotonic() < t_end:
                joy_pub.publish(Joy(axes=[float(a) for a in axes], buttons=buttons))
                time.sleep(0.01)
        pump([1, 0, 0, 0, 0, 0], [1, 0], 0.4)               # enable held, full deflection along x
        assert max(v for _, v in tw) == pytest.approx(0.010, rel=0.02)
        t_rel = time.monotonic()
        pump([1, 0, 0, 0, 0, 0], [0, 0], 0.3)               # enable released
        assert all(v == 0.0 for t, v in tw if t > t_rel + 0.05)
        pump([0, 0, -1, 0, 0, 0], [1, 0], 0.3)              # push down: force nudges
        assert any('"force_step"' in c for c in cmds)
        t_loss = time.monotonic()                           # driver stops sending
        time.sleep(0.6)
        late = [v for t, v in tw if t > t_loss + 0.2]
        assert late == [], f'twists after the driver stopped: {late[:3]}'
        assert any('teleop_input_lost' in c for c in cmds)
    finally:
        rclpy.shutdown()
