"""freedrive_check against a scripted /robot/state (the operator's pedal and hand are simulated). MOCKED."""
import threading
import time

import numpy as np
import pytest
import rclpy

from usrig_common import geometry
from usrig_common.timeutil import now_ns, stamp_from_ns
from usrig_interfaces.msg import RobotState

DOWN = np.diag([1.0, -1.0, -1.0])


class FakeArm:
    """Publishes /robot/state at 200 Hz; the test sets labels and moves the pose."""

    def __init__(self):
        self.node = rclpy.create_node('fake_arm')
        self.pub = self.node.create_publisher(RobotState, '/robot/state', 50)
        self.T = np.eye(4)
        self.T[:3, :3] = DOWN
        self.T[:3, 3] = [0.4, 0.0, 0.2]
        self.fd = False
        self.lock = threading.Lock()
        self.node.create_timer(0.005, self.publish)
        self.ex = rclpy.executors.SingleThreadedExecutor()
        self.ex.add_node(self.node)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def publish(self):
        m = RobotState()
        m.header.stamp = stamp_from_ns(now_ns())
        with self.lock:
            geometry.fill_pose_msg(m.tcp_base, self.T)
            fd = self.fd
        m.tcp_phantom = m.tcp_base
        m.force_setpoint_n = float('nan')
        m.mode = RobotState.MODE_FREEDRIVE if fd else RobotState.MODE_IDLE
        m.action_source = RobotState.SOURCE_KINESTHETIC if fd else RobotState.SOURCE_NONE
        m.action_is_expert = fd
        self.pub.publish(m)

    def guide(self, dx_m, seconds=1.0, labelled=True):
        """Hand motion: out and back along x."""
        with self.lock:
            self.fd = labelled
        n = int(seconds / 0.01)
        x0 = self.T[0, 3]
        for k in range(n + 1):
            with self.lock:
                self.T[0, 3] = x0 + dx_m * np.sin(np.pi * k / n)
            time.sleep(0.01)
        with self.lock:
            self.fd = False


@pytest.fixture
def setup():
    rclpy.init()
    from usrig_common.tool_node import ToolNode
    arm = FakeArm()
    tool = ToolNode('freedrive_check_test')
    tool.rig.setdefault('freedrive', {})['input'] = 2
    tool.rig['limits']['max_linear_speed_mm_s'] = 200.0
    tool.state(5.0)
    yield arm, tool
    tool.destroy_node()
    arm.ex.shutdown()
    arm.node.destroy_node()
    rclpy.shutdown()


def script(arm, labelled_motion=True, drift_mm=0.0):
    """Operator answers for the 4 steps; the pedal / hand actions happen between the prompts."""
    def ask(prompt):
        if prompt.startswith('\n2/4'):
            return ''
        if 'released the pedal' in prompt:        # step 2 running: guide the arm, then answer
            arm.guide(0.05, 1.0, labelled_motion)
            return ''
        if prompt.startswith('\n4/4'):
            with arm.lock:
                arm.fd = True
            if drift_mm:
                threading.Thread(target=lambda: arm.guide(drift_mm / 1000.0 / 2, 2.0, True), daemon=True).start()
            return ''
        with arm.lock:
            arm.fd = False
        return ''
    return ask


def test_freedrive_check_passes(setup, capsys):
    from usrig_freedrive import freedrive_check
    arm, tool = setup
    assert freedrive_check.main(['--still-s', '0.5', '--drift-s', '0.5'], ask=script(arm), node=tool)
    out = capsys.readouterr().out
    assert '2 guided: labelled kinesthetic' in out and 'RESULT: PASS' in out


def test_freedrive_check_catches_wrong_pedal_input(setup, capsys):
    from usrig_freedrive import freedrive_check
    arm, tool = setup
    assert not freedrive_check.main(['--still-s', '0.5', '--drift-s', '0.5'], ask=script(arm, labelled_motion=False),
                                    node=tool)
    line = next(x for x in capsys.readouterr().out.splitlines() if 'no unlabelled motion' in x)
    assert ' FAIL ' in line


def test_freedrive_check_catches_drift(setup, capsys):
    from usrig_freedrive import freedrive_check
    arm, tool = setup
    assert not freedrive_check.main(['--still-s', '0.5', '--drift-s', '1.0'], ask=script(arm, drift_mm=10.0), node=tool)
    assert 'payload' in capsys.readouterr().out


def test_freedrive_check_refuses_without_pedal_input(setup):
    from usrig_freedrive import freedrive_check
    arm, tool = setup
    tool.rig['freedrive']['input'] = None
    with pytest.raises(RuntimeError, match='freedrive.input is null'):
        freedrive_check.main([], ask=lambda _: '', node=tool)
