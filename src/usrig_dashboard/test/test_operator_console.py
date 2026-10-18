"""Operator console GUI, offscreen, with a scripted ROS side: buttons send the right commands and are
disabled when the rig would refuse; nothing takes keyboard focus. Skipped without PySide6."""
import os
import threading
import time
import types

import pytest

QtWidgets = pytest.importorskip('PySide6.QtWidgets')
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


class FakeRos:
    def __init__(self, status, mode=0, control=True):
        from usrig_common import config
        from usrig_interfaces.msg import ControlStatus
        self.scfg = config.load_config('session', None)
        self.rig = config.load_config('rig', None)
        self.sim, self.session_param, self.busy = False, '', None
        self.lock = threading.Lock()
        self.cli, self.vc, self.sent = {}, types.SimpleNamespace(), []
        self.status, self.control = status, control
        self.ctl = ControlStatus()
        self.ctl.mode = mode

    def snapshot(self):
        L = {'status': self.status, 'ctl': self.ctl if self.control else None}
        live = {'status': self.status is not None, 'ctl': self.control, 'state': False, 'qc': False}
        return L, live, [], [], set(), None, {}

    def session_dir(self):
        return None

    def send(self, p):
        self.sent.append(p)

    def note(self, t):
        pass

    def trigger(self, key, label):
        self.sent.append(('trigger', key))

    def run_async(self, name, fn):
        self.sent.append(('async', name))
        return True


@pytest.fixture
def app(monkeypatch, tmp_path):
    import shutil
    from pathlib import Path
    cfg = tmp_path / 'config'
    shutil.copytree(Path(__file__).resolve().parents[2] / 'usrig_bringup' / 'config', cfg)
    monkeypatch.setenv('USRIG_CONFIG_DIR', str(cfg))
    a = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(QtWidgets.QMessageBox, 'question', lambda *a, **k: QtWidgets.QMessageBox.StandardButton.Yes)
    return a


def make(app, status, **kw):
    from usrig_dashboard.operator_console import build_window
    ros = FakeRos(status, **kw)
    win, refresh = build_window(ros)
    win.show()
    refresh()
    app.processEvents()
    return ros, win, refresh


def btn(win, text):
    for b in win.findChildren(QtWidgets.QPushButton):
        if b.text() == text:
            return b
    raise AssertionError(f'no button {text!r}')


IDLE = {'state': 'idle', 'waiting_for': ['start_stop'], 'episode': None,
        'session': {'session_id': 's1', 'stage': 0, 'robot_control': True}}
REC = {'state': 'recording', 'waiting_for': ['discard', 'outcome', 'start_stop'],
       'episode': {'number': 3, 'episode_id': 's1_ep_0003', 'elapsed_s': 12.0, 'paused': False},
       'session': {'session_id': 's1', 'stage': 0, 'robot_control': True}}


def test_start_pause_outcome_send_operator_commands(app):
    ros, win, refresh = make(app, IDLE)
    assert btn(win, 'Start episode').isEnabled() and not btn(win, 'Pause').isEnabled()
    btn(win, 'Start episode').click()
    assert ros.sent[-1] == {'cmd': 'start_stop'}
    ros.status = REC
    refresh()
    assert btn(win, 'Stop episode').isEnabled() and btn(win, 'Pause').isEnabled()
    btn(win, 'Pause').click()
    btn(win, 'Success').click()
    assert ros.sent[-2:] == [{'cmd': 'pause'}, {'cmd': 'outcome', 'value': 'success'}]
    win.close()


def test_disabled_buttons_say_why_and_send_nothing(app):
    ros, win, refresh = make(app, REC, mode=6)                       # FAULT
    b = btn(win, 'Teleop ON')
    assert not b.isEnabled() and 'FAULT' in b.toolTip()
    b.click()
    assert ('async', 'teleop on') not in ros.sent
    assert btn(win, 'Acknowledge fault').isEnabled()
    btn(win, 'Acknowledge fault').click()
    assert ros.sent[-1] == {'cmd': 'ack_fault'}
    assert not btn(win, 'Go to approach').isEnabled()               # episode in progress
    win.close()


def test_nothing_takes_keyboard_focus(app):
    """evdev reads the operator keyboard whatever has focus: a focused button would be pressed twice."""
    ros, win, refresh = make(app, IDLE)
    from PySide6.QtCore import Qt
    assert all(b.focusPolicy() == Qt.NoFocus for b in win.findChildren(QtWidgets.QPushButton))
    for w in win.findChildren(QtWidgets.QLineEdit) + win.findChildren(QtWidgets.QPlainTextEdit):
        assert w.focusPolicy() in (Qt.ClickFocus, Qt.NoFocus)
    assert not isinstance(app.focusWidget(), (QtWidgets.QLineEdit, QtWidgets.QPlainTextEdit, QtWidgets.QPushButton))
    win.close()


def test_without_robot_control_motion_is_disabled(app):
    ros, win, refresh = make(app, IDLE, control=False)               # freedrive collection
    for t in ('PROTECTIVE STOP', 'Teleop ON', 'Go to approach', 'Verify selected'):
        assert not btn(win, t).isEnabled(), t
    assert btn(win, 'Start episode').isEnabled()
    win.close()


def test_stop_without_outcome_asks_for_one(app):
    """QC fails an episode without an outcome event: Stop asks first (or stops anyway / cancels)."""
    ros, win, refresh = make(app, REC)
    win.ask_outcome = lambda: 'recovered'
    btn(win, 'Stop episode').click()
    assert ros.sent[-2:] == [{'cmd': 'outcome', 'value': 'recovered'}, {'cmd': 'start_stop'}]
    ros.status = dict(REC, episode=dict(REC['episode'], episode_id='s1_ep_0004', number=4))   # next episode
    refresh()
    win.ask_outcome = lambda: None                                   # cancel
    n = len(ros.sent)
    btn(win, 'Stop episode').click()
    assert len(ros.sent) == n
    ros.status = dict(REC, episode=dict(REC['episode'], outcome='success'))
    refresh()
    win.ask_outcome = lambda: (_ for _ in ()).throw(AssertionError('must not ask when an outcome is set'))
    btn(win, 'Stop episode').click()
    assert ros.sent[-1] == {'cmd': 'start_stop'}
    win.close()


def test_disabled_protective_stop_does_not_look_active(app):
    """Without robot_control the console cannot stop the arm: the button must not look like a live red stop."""
    ros, win, refresh = make(app, IDLE, control=False)
    b = btn(win, 'PROTECTIVE STOP')
    assert not b.isEnabled() and 'pendant' in b.toolTip()
    img = b.grab().toImage()
    c = img.pixelColor(img.width() // 2, img.height() // 4)
    assert not (c.red() > 150 and c.green() < 90 and c.blue() < 90), (c.red(), c.green(), c.blue())
    win.close()


def test_protective_stop_is_never_refused_while_another_action_runs(app):
    """The console runs one blocking action at a time; the protective stop must bypass that lock."""
    ros, win, refresh = make(app, REC)
    ros.busy = 'run path'                                            # a 600 s path from the console
    ros.pstops = []
    ros.protective_stop = lambda: ros.pstops.append(time.monotonic()) or True
    refresh()
    b = btn(win, 'PROTECTIVE STOP')
    assert b.isEnabled()
    b.click()
    assert len(ros.pstops) == 1
    win.close()


def test_console_ros_protective_stop_ignores_busy():
    """ConsoleRos.protective_stop calls /robot/protective_stop even while a worker action holds `busy`."""
    from usrig_dashboard.operator_console import ConsoleRos
    called = threading.Event()

    class Cli:
        srv_name = '/robot/protective_stop'

        def wait_for_service(self, timeout_sec=None):
            return True
    fake = types.SimpleNamespace(busy='run path', cli={'pstop': Cli()}, notes=[])
    fake.note = fake.notes.append
    fake.call = lambda client, req, timeout=None: called.set() or types.SimpleNamespace(success=True, message='ok')
    assert ConsoleRos.protective_stop(fake) is True
    assert called.wait(2.0), fake.notes
    assert fake.busy == 'run path'                                   # the other action keeps its slot


def test_stop_right_after_an_outcome_click_does_not_ask_again(app):
    """Status arrives at 2 Hz: Success then Stop within half a second asked 'No outcome is set' (seen live)."""
    ros, win, refresh = make(app, REC)
    win.ask_outcome = lambda: (_ for _ in ()).throw(AssertionError('asked although Success was just clicked'))
    btn(win, 'Success').click()
    btn(win, 'Stop episode').click()                                  # status not refreshed in between
    assert ros.sent[-2:] == [{'cmd': 'outcome', 'value': 'success'}, {'cmd': 'start_stop'}]
    win.close()


def test_set_task_sends_target_and_instruction(app):
    ros, win, refresh = make(app, IDLE)
    win.findChild(QtWidgets.QLineEdit, 'task_target').setText('ijv_long')
    win.findChild(QtWidgets.QLineEdit, 'task_instruction').setText('Centre the right IJV in long axis')
    btn(win, 'Set task').click()
    assert ros.sent[-1] == {'cmd': 'set_task', 'target': 'ijv_long', 'instruction': 'Centre the right IJV in long axis'}
    win.close()
