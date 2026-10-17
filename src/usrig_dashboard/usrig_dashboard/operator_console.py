"""Operator console for data collection: episodes, recording, safety, presets, freedrive, session.

  ros2 run usrig_dashboard operator_console [--ros-args -p session_id:=<id> -p sim:=true]
  (or collect.launch.py console:=true)

The demo dashboard (images, plots) stays for demos. This console has NO images and reads the 500 Hz robot
stream decimated (1 message in 25 decoded), so it can run during collection. Design rules:

* It never moves the robot by itself and is never the motion input: probe motion stays on the dedicated
  keyboard / SpaceMouse (hold-to-move). Buttons only send the same commands as the keyboard
  (/operator/command) or call robot_control services, and every button that can move the arm asks first.
* Mouse only: buttons never take keyboard focus and there are no shortcuts. evdev reads the operator
  keyboard regardless of focus, so a focused button would be pressed twice (Space = view reached).
* It is not the parent of capture processes: collections are launched in their own terminal, detached;
  closing the console stops nothing.
* Pause: the robot holds (with robot_control) and the span is marked in the bag and episode.json; recording continues (see
  usrig_episode/episode_status.py for why).
"""
import json
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from usrig_common import config, geometry
from usrig_common.session import data_root, read_json, session_dir
from usrig_common.timeutil import ns_from_stamp
from usrig_interfaces.msg import ControlStatus, EpisodeEvent, RobotState

from . import console_model as cm

DECIMATE = 25                 # 500 Hz robot state -> 20 Hz decoded
STALE_S = {'status': 2.0, 'ctl': 0.5, 'state': 0.5, 'qc': 2.0}
EXPECTED_NODES = ['micrus', 'robot_state', 'robot_control', 'd415', 'qc_monitor', 'episode_manager', 'keyboard_teleop']


class ConsoleRos(Node):
    """ROS side: latest values only, never blocks the GUI. Service calls run in worker threads."""

    def __init__(self):
        super().__init__('operator_console')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('session_id', '')
        self.declare_parameter('sim', False)
        cfg_dir = self.get_parameter('config_dir').value or None
        self.scfg = config.load_config('session', cfg_dir)
        self.rig = config.load_config('rig', cfg_dir)
        self.session_param = self.get_parameter('session_id').value
        self.sim = bool(self.get_parameter('sim').value)
        self.lock = threading.Lock()
        self.latest, self.t_last = {}, {}
        self.events = deque(maxlen=300)
        self.log = deque(maxlen=300)              # console's own messages (worker results)
        self.keys_held, self.input_lost = set(), None
        self.fd = {'samples': 0, 'freedrive': 0, 'episode': None}
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/episode/status', lambda m: self._set('status', json.loads(m.data)), latched)
        self.create_subscription(ControlStatus, '/robot/control_status', lambda m: self._set('ctl', m), be)
        self.create_subscription(String, '/qc/status', lambda m: self._set('qc', json.loads(m.data)), be)
        self.create_subscription(String, '/micrus/frame_info', lambda m: self._set('frame_info', m.data), be)
        self.create_subscription(EpisodeEvent, '/episode/events', self._on_event,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, '/operator/command', self._on_cmd, QoSProfile(depth=50))
        self._n_state = 0
        self.create_subscription(RobotState, '/robot/state', self._on_state_raw, be, raw=True)
        self.pub_cmd = self.create_publisher(String, '/operator/command', 10)
        self.cli = {'pstop': self.create_client(Trigger, '/robot/protective_stop'),
                    'zero_ft': self.create_client(Trigger, '/robot/zero_ft'),
                    'teleop': self.create_client(SetBool, '/robot/set_teleop'),
                    'retract': self.create_client(Trigger, '/robot/retract')}
        from usrig_presets.waypoints import VerifyClients
        self.vc = VerifyClients(self)
        from usrig_interfaces.srv import RunPath
        self.cli['run_path'] = self.create_client(RunPath, '/robot/run_path')
        self.busy = None                          # name of the running worker action

    # ---------------------------------------------------------------- inputs
    def _set(self, k, v):
        with self.lock:
            self.latest[k] = v
            self.t_last[k] = time.monotonic()

    def _on_state_raw(self, b):
        self._n_state += 1
        if self._n_state % DECIMATE:
            return
        from rclpy.serialization import deserialize_message
        m = deserialize_message(b, RobotState)
        self._set('state', m)
        with self.lock:
            st = self.latest.get('status') or {}
            ep = (st.get('episode') or {}).get('episode_id')
            if ep != self.fd['episode']:
                self.fd = {'samples': 0, 'freedrive': 0, 'episode': ep}
            if ep and st.get('state') == 'recording':
                self.fd['samples'] += 1
                self.fd['freedrive'] += int(m.mode == RobotState.MODE_FREEDRIVE)

    def _on_event(self, m):
        try:
            v = json.loads(m.value or '{}')
        except ValueError:
            v = {}
        with self.lock:
            self.events.append((ns_from_stamp(m.header.stamp), m.episode_id, m.kind, v))

    def _on_cmd(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            if c.get('cmd') == 'key':
                (self.keys_held.add if c.get('down') else self.keys_held.discard)(c.get('key'))
            elif c.get('cmd') == 'teleop_input_lost':
                self.input_lost = c.get('device') or 'keyboard'
                self.keys_held.clear()
            elif c.get('cmd') == 'teleop_input_back':
                self.input_lost = None

    # ---------------------------------------------------------------- helpers for the waypoint tools
    def state(self, timeout=2.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                s = self.latest.get('state')
                age = time.monotonic() - self.t_last.get('state', 0)
            if s is not None and age < STALE_S['state']:
                return s
            time.sleep(0.02)
        raise RuntimeError('no live /robot/state (is robot_state running?)')

    def tip_pose_base(self, tool='probe'):
        from usrig_common.tool_node import ToolNode
        return ToolNode.tip_pose_base(self, tool)

    def call(self, client, req, timeout=30.0):
        from usrig_common.tool_node import ToolNode
        return ToolNode.call(self, client, req, timeout)

    # ---------------------------------------------------------------- outputs
    def send(self, payload):
        self.pub_cmd.publish(String(data=json.dumps(payload)))
        self.note(f'sent {payload.get("cmd")}' + (f' {payload["value"]}' if 'value' in payload else ''))

    def note(self, text):
        with self.lock:
            self.log.append((time.time(), text))

    def run_async(self, name, fn):
        """Run a blocking action (service calls) off the GUI thread; one at a time."""
        if self.busy:
            self.note(f'{name}: refused, {self.busy} still running')
            return False
        self.busy = name

        def work():
            try:
                r = fn()
                self.note(f'{name}: {r if r is not None else "done"}')
            except Exception as e:  # noqa: BLE001
                self.note(f'{name} FAILED: {e}')
            finally:
                self.busy = None
        threading.Thread(target=work, daemon=True).start()
        return True

    def protective_stop(self):
        """Never refused: its own thread, outside the one-at-a-time worker slot (`busy`), so it also works
        while a console path / move / verify is running."""
        def work():
            try:
                if not self.cli['pstop'].wait_for_service(timeout_sec=2.0):
                    raise RuntimeError(f"{self.cli['pstop'].srv_name} not available (use the pendant)")
                r = self.call(self.cli['pstop'], Trigger.Request(), 10)
                self.note(f'protective stop: {r.message}' if r.success else f'protective stop REFUSED: {r.message}')
            except Exception as e:  # noqa: BLE001
                self.note(f'protective stop FAILED: {e} (use the pendant)')
        threading.Thread(target=work, daemon=True).start()
        return True

    def trigger(self, key, label):
        def fn():
            if not self.cli[key].wait_for_service(timeout_sec=2.0):
                raise RuntimeError(f'{self.cli[key].srv_name} not available')
            r = self.call(self.cli[key], Trigger.Request(), 60)
            return r.message if r.success else f'refused: {r.message}'
        return self.run_async(label, fn)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            live = {k: (k in self.t_last and now - self.t_last[k] <= t) for k, t in STALE_S.items()}
            return dict(self.latest), live, list(self.events), list(self.log), set(self.keys_held), \
                self.input_lost, dict(self.fd)

    def session_dir(self):
        st = self.latest.get('status') or {}
        sd = (st.get('session') or {}).get('session_dir')
        if sd:
            return Path(sd)
        if self.session_param:
            return session_dir(data_root(self.scfg, self.sim), self.session_param)
        return None


# ==================================================================================================
# GUI
# ==================================================================================================

STYLE = """
QWidget { font-size: 13px; }
QGroupBox { font-weight: 600; border: 1px solid #b0b7c3; border-radius: 6px; margin-top: 10px; padding: 6px; }
QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
QPushButton { padding: 6px 10px; border-radius: 4px; border: 1px solid #8c96a6; background: #f4f6f9; }
QPushButton:hover:enabled { background: #e3e9f2; }
QPushButton:disabled { color: #9aa1ab; background: #eceef1; border-color: #cfd4db; }
QPushButton#primary { background: #1f5fae; color: white; border-color: #1f5fae; font-weight: 600; }
QPushButton#good { background: #2e7d32; color: white; border-color: #2e7d32; }
QPushButton#warn { background: #b26a00; color: white; border-color: #b26a00; }
QPushButton#bad { background: #a83232; color: white; border-color: #a83232; }
QPushButton#primary:disabled, QPushButton#good:disabled, QPushButton#warn:disabled, QPushButton#bad:disabled { background: #cfd4db; color: #8a919b; border-color: #cfd4db; }
QPushButton#pstop { background: #c62828; color: white; font-weight: 700; font-size: 15px; padding: 10px 18px; border: 2px solid #8e1b1b; }
QPushButton#pstop:disabled { background: #cfd4db; color: #8a919b; border-color: #b0b6bf; }
QLabel#rec { font-size: 18px; font-weight: 700; padding: 6px 14px; border-radius: 6px; }
QLabel#prompt { font-size: 15px; font-weight: 600; padding: 8px 10px; border-radius: 4px; }
QLabel#big { font-size: 26px; font-weight: 700; }
QLabel#mode { font-size: 20px; font-weight: 700; }
QLabel.k { color: #5a6270; }
"""

REC_STYLE = {'recording': 'background:#c62828; color:white;', 'paused': 'background:#e0a100; color:black;',
             'busy': 'background:#1f5fae; color:white;', 'idle': 'background:#d5d9df; color:#333;',
             'none': 'background:#eceef1; color:#777;'}


def build_window(ros, scfg=None, rig=None):
    """Build the console window around ``ros`` (ConsoleRos, or a fake with the same methods for tests).
    Returns (window, refresh_fn)."""
    from PySide6 import QtCore, QtGui, QtWidgets
    from usrig_presets.waypoints import capture_waypoint, executable_problem, list_waypoints, verify_waypoint
    Qt = QtCore.Qt
    scfg = scfg or ros.scfg
    rig = rig or ros.rig

    win = QtWidgets.QMainWindow()
    win.setWindowTitle('usrig operator console')
    win.setStyleSheet(STYLE)
    central = QtWidgets.QWidget()
    root = QtWidgets.QVBoxLayout(central)
    root.setSpacing(6)
    win.setCentralWidget(central)
    buttons = {}            # action -> [QPushButton]

    def button(text, action=None, on_click=None, obj=None, tip=None):
        b = QtWidgets.QPushButton(text)
        b.setFocusPolicy(Qt.NoFocus)                # see module docstring: never takes keyboard focus
        if obj:
            b.setObjectName(obj)
        if tip:
            b.setToolTip(tip)
        if on_click:
            b.clicked.connect(lambda _=False: on_click())
        if action:
            buttons.setdefault(action, []).append(b)
            b.setProperty('base_tip', tip or '')
        return b

    def confirm(title, text):
        r = QtWidgets.QMessageBox.question(win, title, text)
        return r == QtWidgets.QMessageBox.StandardButton.Yes

    def kv_form(rows):
        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(12)
        labels = {}
        for i, (key, title) in enumerate(rows):
            k = QtWidgets.QLabel(title)
            k.setProperty('class', 'k')
            k.setStyleSheet('color:#5a6270')
            v = QtWidgets.QLabel('-')
            v.setTextInteractionFlags(Qt.TextSelectableByMouse)
            v.setWordWrap(True)
            g.addWidget(k, i, 0, Qt.AlignTop)
            g.addWidget(v, i, 1)
            labels[key] = v
        g.setColumnStretch(1, 1)
        return g, labels

    # ------------------------------------------------------------------ header
    head = QtWidgets.QHBoxLayout()
    lbl_session = QtWidgets.QLabel()
    lbl_session.setTextFormat(Qt.RichText)
    lbl_clock = QtWidgets.QLabel()
    lbl_clock.setStyleSheet('font-size:15px; font-weight:600; color:#333')
    lbl_rec = QtWidgets.QLabel('NO EPISODE MANAGER')
    lbl_rec.setObjectName('rec')
    lbl_rec.setAlignment(Qt.AlignCenter)
    lbl_rec.setMinimumWidth(260)

    def do_ack():
        ros.send({'cmd': 'ack_fault'})

    def do_pstop():
        ros.protective_stop()

    head.addWidget(lbl_session, 1)
    head.addWidget(lbl_clock)
    head.addSpacing(12)
    head.addWidget(lbl_rec)
    head.addSpacing(12)
    head.addWidget(button('Acknowledge fault', 'ack_fault', do_ack, 'warn'))
    head.addWidget(button('PROTECTIVE STOP', 'protective_stop', do_pstop, 'pstop',
                          'robot_control triggers a protective stop on the UR7e (the pendant e-stop is always there)'))
    root.addLayout(head)
    lbl_prompt = QtWidgets.QLabel()
    lbl_prompt.setObjectName('prompt')
    root.addWidget(lbl_prompt)

    split = QtWidgets.QSplitter(Qt.Horizontal)
    root.addWidget(split, 1)

    # ------------------------------------------------------------------ left: episode
    left = QtWidgets.QWidget()
    ll = QtWidgets.QVBoxLayout(left)
    ll.setContentsMargins(0, 0, 0, 0)
    g_ep = QtWidgets.QGroupBox('Episode')
    gl = QtWidgets.QVBoxLayout(g_ep)
    top_ep = QtWidgets.QHBoxLayout()
    lbl_epnum = QtWidgets.QLabel('Episode -')
    lbl_epnum.setObjectName('big')
    lbl_elapsed = QtWidgets.QLabel('')
    lbl_elapsed.setObjectName('big')
    top_ep.addWidget(lbl_epnum, 1)
    top_ep.addWidget(lbl_elapsed)
    gl.addLayout(top_ep)
    form, ep_lbl = kv_form([('id', 'Episode ID'), ('date', 'Date'), ('start', 'Start time'), ('paused', 'Paused'),
                            ('state', 'State'), ('type', 'Type / target'), ('instruction', 'Instruction'),
                            ('row', 'Plan row'), ('marks', 'Marks'), ('outcome', 'Outcome')])
    gl.addLayout(form)
    lbl_last = QtWidgets.QLabel()
    lbl_last.setStyleSheet('color:#444; padding-top:4px')
    lbl_last.setWordWrap(True)
    gl.addWidget(lbl_last)
    ll.addWidget(g_ep)

    g_rec = QtWidgets.QGroupBox('Recording')
    rl = QtWidgets.QGridLayout(g_rec)

    sent_outcome = {}           # episode_id -> outcome sent from this console (status lags by up to 0.5 s)

    def send_outcome(v):
        sent_outcome[(ctx_now().episode or {}).get('episode_id')] = v
        ros.send({'cmd': 'outcome', 'value': v})

    def do_start_stop():
        c = ctx_now()
        starting = cm.start_stop_label(c) == 'Start episode'
        if starting and c.session.get('robot_control') and not confirm(
                'Robot may move', 'Starting lets episode_manager move the robot (approach, descend) when a waypoint '
                'or stage needs it. Area clear and e-stop at hand?'):
            return
        ep_now = c.episode or {}
        if (not starting and 'outcome' in c.waiting and not ep_now.get('outcome')
                and ep_now.get('episode_id') not in sent_outcome):
            # QC requires an outcome event: an episode stopped without one fails the 'events' check
            pick = win.ask_outcome()
            if pick is None:
                return
            if pick != 'none':
                send_outcome(pick)
        ros.send({'cmd': 'start_stop'})

    def ask_outcome():
        """'success' | 'recovered' | 'fail' | 'none' (stop without one) | None (cancel)."""
        box = QtWidgets.QMessageBox(win)
        box.setWindowTitle('Outcome before stopping')
        box.setText('No outcome is set for this episode. QC fails an episode without one.')
        picks = {box.addButton(t, QtWidgets.QMessageBox.ButtonRole.AcceptRole): v
                 for t, v in (('Success', 'success'), ('Recovered', 'recovered'), ('Fail', 'fail'))}
        anyway = box.addButton('Stop without outcome', QtWidgets.QMessageBox.ButtonRole.DestructiveRole)
        box.addButton(QtWidgets.QMessageBox.StandardButton.Cancel)
        box.exec()
        b = box.clickedButton()
        return picks.get(b) or ('none' if b is anyway else None)

    def do_discard():
        if confirm('Discard episode', 'Discard this episode? It is stopped and kept on disk as outcome "discard".'):
            ros.send({'cmd': 'discard'})

    def do_next():
        c = ctx_now()
        if c.session.get('robot_control') and not confirm(
                'Robot will move', 'Next row: episode_manager moves to the next approach pose and descends. '
                'Area clear and e-stop at hand?'):
            return
        ros.send({'cmd': 'next'})

    b_ss = button('Start episode', 'start_stop', do_start_stop, 'primary', 'starts / stops the recording of an episode (B)')
    rl.addWidget(b_ss, 0, 0, 1, 2)
    rl.addWidget(button('Pause', 'pause', lambda: ros.send({'cmd': 'pause'}), 'warn',
                        'robot holds (with robot_control); the span is marked, recording continues'), 0, 2)
    rl.addWidget(button('Resume', 'resume', lambda: ros.send({'cmd': 'resume'}), 'good'), 0, 3)
    rl.addWidget(button('Success', 'outcome', lambda: send_outcome('success'), 'good', '1'), 1, 0)
    rl.addWidget(button('Recovered', 'outcome', lambda: send_outcome('recovered'), 'good', '2'), 1, 1)
    rl.addWidget(button('Fail', 'outcome', lambda: send_outcome('fail'), 'bad', '3'), 1, 2)
    rl.addWidget(button('Discard', 'discard', do_discard, 'bad', 'Esc'), 1, 3)
    rl.addWidget(button('View reached', 'view_reached', lambda: ros.send({'cmd': 'view_reached'}), tip='Space'), 2, 0)
    rl.addWidget(button('Failure onset', 'onset', lambda: ros.send({'cmd': 'onset'}), tip='O'), 2, 1)
    rl.addWidget(button('Recovery start', 'recovery_start', lambda: ros.send({'cmd': 'recovery_start'}), tip='U'), 2, 2)
    rl.addWidget(button('Recovery end', 'recovery_end', lambda: ros.send({'cmd': 'recovery_end'}), tip='I'), 2, 3)
    rl.addWidget(button('Next row', 'next', do_next, 'primary', 'N (Stage 2)'), 3, 0, 1, 4)
    note = QtWidgets.QLineEdit()
    note.setPlaceholderText('Note for this episode (type on the PC keyboard, not the operator keyboard)')
    gel = QtWidgets.QComboBox()
    gel.addItems(['gel: -', 'applied', 'reapplied', 'insufficient'])
    gel.setFocusPolicy(Qt.NoFocus)

    def add_note():
        if note.text().strip():
            ros.send({'cmd': 'note', 'value': note.text().strip()})
            note.clear()
        note.clearFocus()                          # give the keyboard back: no field keeps focus
    note.returnPressed.connect(add_note)
    gel.activated.connect(lambda i: i > 0 and ros.send({'cmd': 'gel', 'value': gel.currentText()}))
    rl.addWidget(note, 4, 0, 1, 3)
    rl.addWidget(button('Add note', 'note', add_note), 4, 3)
    rl.addWidget(gel, 5, 0, 1, 2)
    buttons.setdefault('gel_combo', []).append(gel)
    ll.addWidget(g_rec)

    g_prog = QtWidgets.QGroupBox('Session progress')
    pl = QtWidgets.QVBoxLayout(g_prog)
    lbl_prog = QtWidgets.QLabel('-')
    lbl_prog.setWordWrap(True)
    bar = QtWidgets.QProgressBar()
    bar.setVisible(False)
    pl.addWidget(lbl_prog)
    pl.addWidget(bar)
    ll.addWidget(g_prog)
    ll.addStretch(1)
    split.addWidget(left)

    # ------------------------------------------------------------------ middle: robot
    mid = QtWidgets.QWidget()
    ml = QtWidgets.QVBoxLayout(mid)
    ml.setContentsMargins(0, 0, 0, 0)
    g_rob = QtWidgets.QGroupBox('Robot')
    rbl = QtWidgets.QVBoxLayout(g_rob)
    lbl_mode = QtWidgets.QLabel('-')
    lbl_mode.setObjectName('mode')
    rbl.addWidget(lbl_mode)
    form, rob_lbl = kv_form([('source', 'Action source'), ('expert', 'Expert label'), ('flags', 'Safety flags'),
                             ('fsp', 'Force setpoint'), ('force', 'Probe-axis force'), ('pose', 'TCP (phantom)'),
                             ('input', 'Teleop input'), ('streams', 'Streams')])
    rbl.addLayout(form)
    ml.addWidget(g_rob)

    g_ctl = QtWidgets.QGroupBox('Teleop and safety')
    cl = QtWidgets.QGridLayout(g_ctl)

    def set_teleop(on):
        def fn():
            if not ros.cli['teleop'].wait_for_service(timeout_sec=2.0):
                raise RuntimeError('/robot/set_teleop not available')
            r = ros.call(ros.cli['teleop'], SetBool.Request(data=on), 10)
            return r.message if r.success else f'refused: {r.message}'
        ros.run_async('teleop on' if on else 'teleop off', fn)

    step = rig_force_step()

    def rig_force_step_text():
        return f'{step:g} N' if step else 'step from keymap'

    def force(sign):
        if step:
            ros.send({'cmd': 'force_step', 'delta': sign * step})

    def do_recontact():
        if confirm('Robot will move', 'Recontact: slow bounded descent along the probe axis until contact. Continue?'):
            ros.send({'cmd': 'recontact'})

    cl.addWidget(button('Teleop ON', 'teleop_on', lambda: set_teleop(True), 'good',
                        'enable keyboard / SpaceMouse motion (still needs Ctrl / the enable button held)'), 0, 0)
    cl.addWidget(button('Teleop OFF (hold)', 'teleop_off', lambda: set_teleop(False), 'warn'), 0, 1)
    cl.addWidget(button('Operator hold (P)', 'hold', lambda: ros.send({'cmd': 'hold'}),
                        tip='toggle: robot holds still, twists ignored'), 0, 2)
    cl.addWidget(button(f'Force + ({rig_force_step_text()})', 'force_up', lambda: force(+1), tip='T'), 1, 0)
    cl.addWidget(button(f'Force - ({rig_force_step_text()})', 'force_down', lambda: force(-1), tip='G'), 1, 1)
    cl.addWidget(button('Recontact', 'recontact', do_recontact, tip='J: after a contact loss'), 1, 2)
    cl.addWidget(button('Zero F/T (free air)', 'zero_ft', lambda: ros.trigger('zero_ft', 'zero F/T')), 2, 0)

    def do_retract():
        if confirm('Robot will move', 'Retract the probe along its axis?'):
            ros.trigger('retract', 'retract')
    cl.addWidget(button('Retract', 'retract', do_retract), 2, 1)
    ml.addWidget(g_ctl)

    g_log = QtWidgets.QGroupBox('Console log')
    lgl = QtWidgets.QVBoxLayout(g_log)
    log_view = QtWidgets.QPlainTextEdit()
    log_view.setReadOnly(True)
    log_view.setMaximumBlockCount(300)
    log_view.setFocusPolicy(Qt.NoFocus)
    lgl.addWidget(log_view)
    ml.addWidget(g_log, 1)
    split.addWidget(mid)

    # ------------------------------------------------------------------ right: tabs
    tabs = QtWidgets.QTabWidget()
    tabs.setFocusPolicy(Qt.NoFocus)
    split.addWidget(tabs)
    split.setSizes([520, 420, 560])

    # ---- presets
    pre = QtWidgets.QWidget()
    prl = QtWidgets.QVBoxLayout(pre)
    lbl_reg = QtWidgets.QLabel()
    lbl_reg.setWordWrap(True)
    prl.addWidget(lbl_reg)
    wp_table = QtWidgets.QTableWidget(0, 6)
    wp_table.setHorizontalHeaderLabels(['waypoint', 'organ / view', 'force N', 'verified', 'NCC', 'status'])
    wp_table.horizontalHeader().setStretchLastSection(True)
    wp_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
    wp_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
    wp_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
    wp_table.setFocusPolicy(Qt.NoFocus)
    prl.addWidget(wp_table, 1)
    wps_cache = {'list': []}

    def selected_wp():
        r = wp_table.currentRow()
        return wps_cache['list'][r] if 0 <= r < len(wps_cache['list']) else None

    def load_waypoints():
        sd = ros.session_dir()
        wps = list_waypoints(scfg, sd, ros.sim)
        wps_cache['list'] = wps
        rows = cm.waypoint_rows(wps, executable_problem, rig)
        wp_table.setRowCount(len(rows))
        for i, r in enumerate(rows):
            vals = [r['name'], r['view'], r['force_n'], 'yes' if r['verified'] else 'no', r['ncc'], r['status']]
            for j, v in enumerate(vals):
                it = QtWidgets.QTableWidgetItem('-' if v is None else str(v))
                if j == 5:
                    it.setForeground(QtGui.QColor('#2e7d32' if v == 'ready' else '#a83232'))
                wp_table.setItem(i, j, it)
        reg = read_json(sd / 'session.json').get('phantom_registration') if sd else None
        lbl_reg.setText(f'Session: {sd or "none"}<br>Phantom registration: '
                        + ('<b style="color:#2e7d32">loaded</b> (RMS %.2f mm)' % reg.get('rms_mm', float('nan'))
                           if reg else '<b style="color:#a83232">missing</b>: run register_phantom first'))
        lbl_reg.setTextFormat(Qt.RichText)

    def phantom_move(T, label):
        from geometry_msgs.msg import PoseStamped
        from usrig_interfaces.srv import MoveToPose

        def fn():
            req = MoveToPose.Request()
            req.target = PoseStamped()
            req.target.header.frame_id = 'phantom'
            geometry.fill_pose_msg(req.target.pose, T)
            r = ros.call(ros.vc.move, req, 180)
            return r.message if r.success else f'refused: {r.message}'
        ros.run_async(label, fn)

    def do_goto():
        w = selected_wp()
        if w is None:
            return ros.note('select a waypoint first')
        if confirm('Robot will move', f'Move to the approach pose of "{w["name"]}" (via the safe height)?'):
            phantom_move(geometry.pose_from_pq(w['approach_position_m'], w['approach_quaternion_xyzw']),
                         f'go to {w["name"]}')

    def do_descend():
        from usrig_interfaces.srv import DescendToContact
        w = selected_wp()
        f = abs(float((w or {}).get('force_n') or 0.0))
        if not confirm('Robot will move', 'Descend along the probe axis until contact'
                       + (f' at {f:.1f} N (taught force)' if f else ' (default setpoint)') + '?'):
            return

        def fn():
            req = DescendToContact.Request()
            req.force_setpoint_n = f
            r = ros.call(ros.vc.descend, req, 120)
            return r.message if r.success else f'refused: {r.message}'
        ros.run_async('descend', fn)

    def do_verify():
        w = selected_wp()
        if w is None:
            return ros.note('select a waypoint first')
        if not confirm('Robot will move', f'Verify "{w["name"]}": retract, approach, zero F/T, descend, snapshot, '
                       'compare, retract. Area clear and e-stop at hand?'):
            return
        sd = ros.session_dir()
        ros.run_async(f'verify {w["name"]}', lambda: verify_waypoint(ros, sd, w['name'], ros.vc))

    act = QtWidgets.QHBoxLayout()
    act.addWidget(button('Reload', None, load_waypoints))
    act.addWidget(button('Go to approach', 'goto', do_goto, 'primary'))
    act.addWidget(button('Descend to contact', 'descend', do_descend))
    act.addWidget(button('Retract', 'retract', do_retract))
    act.addWidget(button('Verify selected', 'verify', do_verify))
    prl.addLayout(act)

    path_row = QtWidgets.QHBoxLayout()
    path_combo = QtWidgets.QComboBox()
    path_combo.setFocusPolicy(Qt.NoFocus)
    speed = QtWidgets.QDoubleSpinBox()
    speed.setRange(1.0, float(config.get_path(rig, 'limits.max_linear_speed_mm_s') or 50.0))
    speed.setValue(float(config.get_path(scfg, 'stage1.speed_mm_s') or 10.0))
    speed.setSuffix(' mm/s')
    speed.setFocusPolicy(Qt.NoFocus)

    def load_paths():
        from usrig_common.session import data_dir
        from usrig_presets.waypoints import session_phantom
        pid = session_phantom(scfg, ros.session_dir())
        f = data_dir(scfg, 'paths', ros.sim) / f'{pid}.yaml' if pid else None
        path_combo.clear()
        if f and f.exists():
            path_combo.addItems(sorted((config.load_yaml(f).get('paths') or {}).keys()))

    def do_run_path():
        from usrig_presets.path_editor import resolve
        from usrig_common.session import data_dir
        from usrig_interfaces.srv import RunPath
        from geometry_msgs.msg import Pose
        name = path_combo.currentText()
        if not name:
            return ros.note('no path selected')
        if not confirm('Robot will move', f'Run path "{name}" at {speed.value():.0f} mm/s? Needs contact '
                       '(descend first); the probe follows the path under force control.'):
            return
        from usrig_presets.waypoints import session_phantom
        pid = session_phantom(scfg, ros.session_dir())

        def fn():
            spec = config.load_yaml(data_dir(scfg, 'paths', ros.sim) / f'{pid}.yaml')['paths'][name]
            poses = resolve(spec['waypoints'], list_waypoints(scfg, ros.session_dir(), ros.sim))
            req = RunPath.Request()
            req.waypoints = [geometry.fill_pose_msg(Pose(), T) for T in poses]
            req.speed_mm_s = float(speed.value())
            r = ros.call(ros.cli['run_path'], req, 600)
            return (f'{r.message}, speed error mean {r.speed_error_mean_pct:.1f} %' if r.success
                    else f'refused: {r.message}')
        ros.run_async(f'run path {name}', fn)

    path_row.addWidget(QtWidgets.QLabel('Path'))
    path_row.addWidget(path_combo, 1)
    path_row.addWidget(speed)
    path_row.addWidget(button('Run path', 'run_path', do_run_path, 'primary'))
    prl.addLayout(path_row)

    g_teach = QtWidgets.QGroupBox('Teach a waypoint at the current probe pose')
    tl = QtWidgets.QFormLayout(g_teach)
    t_name, t_organ, t_view = QtWidgets.QLineEdit(), QtWidgets.QLineEdit(), QtWidgets.QLineEdit()
    t_instr = QtWidgets.QPlainTextEdit()
    t_instr.setPlaceholderText('3-5 paraphrased instructions, one per line')
    t_instr.setMaximumHeight(90)
    tl.addRow('Name', t_name)
    tl.addRow('Organ', t_organ)
    tl.addRow('View', t_view)
    tl.addRow('Instructions', t_instr)

    def do_teach():
        sd = ros.session_dir()
        if sd is None:
            return ros.note('no session: start a collection or pass session_id')
        instr = t_instr.toPlainText().splitlines()
        fi = (ros.snapshot()[0].get('frame_info'))
        name = t_name.text()

        def fn():
            wp, f = capture_waypoint(ros, sd, name, t_organ.text().strip(), t_view.text().strip(), instr, fi)
            return f'saved {wp["name"]} (force {wp["force_n"]:.2f} N) to {f}; verify it before use'
        ros.run_async(f'teach {name}', fn)
    tl.addRow(button('Capture waypoint (zero F/T in free air first)', 'teach', do_teach, 'primary'))
    prl.addWidget(g_teach)
    tabs.addTab(pre, 'Presets')

    # ---- freedrive
    fdw = QtWidgets.QWidget()
    fdl = QtWidgets.QVBoxLayout(fdw)
    form, fd_lbl = kv_form([('pedal', 'Pedal (freedrive)'), ('labels', 'Current labels'), ('share', 'Kinesthetic samples'),
                            ('control', 'robot_control'), ('input', 'Pedal input'), ('grip', 'Grip')])
    fdl.addLayout(form)
    fd_help = QtWidgets.QLabel('Freedrive collection: Start episode with the probe in free air (free-air F/T window), '
                               'hold the pedal to guide the arm by the wrist handle, release it to lock, Stop episode. '
                               'Our software never commands the arm in this mode: the UR safety configuration is the '
                               'only limit.')
    fd_help.setWordWrap(True)
    fd_help.setStyleSheet('color:#444')
    fdl.addWidget(fd_help)
    fdl.addStretch(1)
    tabs.addTab(fdw, 'Freedrive')

    # ---- session
    sw = QtWidgets.QWidget()
    sl = QtWidgets.QVBoxLayout(sw)
    sf = QtWidgets.QFormLayout()
    s_mode = QtWidgets.QComboBox()
    for k, m in cm.COLLECTION_MODES.items():
        s_mode.addItem(m['label'], k)
    s_input = QtWidgets.QComboBox()
    s_input.addItems(['keyboard', 'spacemouse'])
    s_oper = QtWidgets.QComboBox()
    s_oper.addItems([str(o) for o in (scfg['session'].get('operators') or [])])
    s_phantom = QtWidgets.QComboBox()
    s_phantom.addItems(sorted((scfg.get('phantoms') or {}).keys()))
    if scfg['session'].get('phantom_id'):
        s_phantom.setCurrentText(scfg['session']['phantom_id'])
    s_id = QtWidgets.QLineEdit(ros.session_param or '')
    s_sim = QtWidgets.QCheckBox('simulation (sim_root, never exported)')
    s_sim.setChecked(bool(ros.sim))
    s_cam = QtWidgets.QCheckBox('wrist camera')
    s_cam.setChecked(True)
    s_hours = QtWidgets.QDoubleSpinBox()
    s_hours.setRange(0.0, 12.0)
    s_hours.setSingleStep(0.5)
    s_hours.setValue(0.0)
    s_hours.setSpecialValueText('set the planned hours (disk check)')
    s_hours.setFocusPolicy(Qt.ClickFocus)
    for w in (s_mode, s_input, s_oper, s_phantom, s_sim, s_cam):
        w.setFocusPolicy(Qt.NoFocus)
    id_row = QtWidgets.QHBoxLayout()
    id_row.addWidget(s_id, 1)

    def new_id():
        r = data_root(scfg, s_sim.isChecked()) / 'sessions'
        existing = {p.name for p in r.glob('*')} if r.exists() else set()
        s_id.setText(cm.make_session_id(scfg['session']['id_pattern'], s_phantom.currentText(),
                                        s_oper.currentText(), existing))
    id_row.addWidget(button('New id', None, new_id))
    sf.addRow('Collection mode', s_mode)
    sf.addRow('Teleop input', s_input)
    sf.addRow('Operator', s_oper)
    sf.addRow('Phantom', s_phantom)
    sf.addRow('Session id', id_row)
    sf.addRow('', s_sim)
    sf.addRow('', s_cam)
    sf.addRow('Planned hours', s_hours)
    sl.addLayout(sf)
    lbl_cmd = QtWidgets.QLabel()
    lbl_cmd.setWordWrap(True)
    lbl_cmd.setTextInteractionFlags(Qt.TextSelectableByMouse)
    lbl_cmd.setStyleSheet('font-family: monospace; background:#f4f6f9; padding:6px; border-radius:4px')
    sl.addWidget(lbl_cmd)
    out = QtWidgets.QPlainTextEdit()
    out.setReadOnly(True)
    out.setFocusPolicy(Qt.NoFocus)
    out.setStyleSheet('font-family: monospace; font-size: 12px')
    proc = {'p': None}

    def sess_dir_for_form():
        return session_dir(data_root(scfg, s_sim.isChecked()), s_id.text().strip())

    def collect_cmd():
        if not s_id.text().strip():
            return None
        return cm.launch_command(s_mode.currentData(), s_id.text().strip(),
                                 session_json=str(sess_dir_for_form() / 'session.json'), sim=s_sim.isChecked(),
                                 camera=s_cam.isChecked(), input_device=s_input.currentText(),
                                 operator_id=s_oper.currentText(), phantom_id=s_phantom.currentText())

    def update_cmd():
        c = collect_cmd()
        lbl_cmd.setText(' '.join(c) if c else 'set a session id (New id) to build the launch command')

    for w in (s_mode, s_input, s_oper, s_phantom):
        w.currentIndexChanged.connect(lambda *_: update_cmd())
    s_id.textChanged.connect(lambda *_: update_cmd())
    s_sim.toggled.connect(lambda *_: update_cmd())
    s_cam.toggled.connect(lambda *_: update_cmd())

    def run_check(argv, title):
        if proc['p'] is not None and proc['p'].poll() is None:
            return ros.note('a check is still running')
        out.setPlainText(f'$ {" ".join(argv)}\n')
        try:
            proc['p'] = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                         start_new_session=True)
        except OSError as e:
            out.appendPlainText(f'could not start: {e}')
            return

        def reader(p=proc['p']):
            for line in p.stdout:
                with ros.lock:
                    pending.append(line.rstrip('\n'))
            with ros.lock:
                pending.append(f'[{title} exited {p.wait()}]')
        threading.Thread(target=reader, daemon=True).start()

    pending = deque()

    def do_presession():
        if not s_id.text().strip():
            return ros.note('set a session id first')
        if not s_hours.value() and not s_sim.isChecked():
            return ros.note('set the planned hours first (the disk check needs them)')
        m = cm.COLLECTION_MODES[s_mode.currentData()]
        run_check(cm.presession_command(m['stage'] or 0, sess_dir_for_form(), control=m['control'],
                                        sim=s_sim.isChecked(), camera=s_cam.isChecked(),
                                        planned_hours=s_hours.value() or None), 'presession_check')

    def do_launch():
        c = collect_cmd()
        if c is None:
            return ros.note('set a session id first')
        m = cm.COLLECTION_MODES[s_mode.currentData()]
        if m['control'] and not confirm('Start the robot nodes', 'This starts robot_control (the arm can be commanded). '
                                        'Area clear, e-stop at hand? (Then run the Pre-session check; episodes start only after it passes.)'):
            return
        term = cm.terminal_command(c, cm.env_script_path(), title=f'usrig {s_id.text().strip()}')
        if term is None:
            QtWidgets.QApplication.clipboard().setText(' '.join(c))
            return ros.note('no terminal emulator found: launch command copied to the clipboard')
        subprocess.Popen(term, start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ros.note(f'launched {m["label"]} in a new terminal')

    brow = QtWidgets.QHBoxLayout()
    brow.addWidget(button('Pre-session check', None, do_presession))
    brow.addWidget(button('Rig status', None, lambda: run_check(['ros2', 'run', 'usrig_qc', 'rig_status'], 'rig_status')))
    brow.addWidget(button('Launch collection (new terminal)', None, do_launch, 'primary'))
    brow.addWidget(button('Copy command', None, lambda: (QtWidgets.QApplication.clipboard().setText(lbl_cmd.text()),
                                                          ros.note('command copied'))))
    sl.addLayout(brow)
    sl.addWidget(out, 1)
    tabs.addTab(sw, 'Session')
    update_cmd()

    # ---- events
    ev_list = QtWidgets.QListWidget()
    ev_list.setFocusPolicy(Qt.NoFocus)
    tabs.addTab(ev_list, 'Events')

    # ---- health
    hw = QtWidgets.QWidget()
    hl = QtWidgets.QVBoxLayout(hw)
    health = QtWidgets.QTableWidget(0, 4)
    health.setHorizontalHeaderLabels(['topic', 'Hz', 'stamp age ms', 'max gap ms'])
    health.horizontalHeader().setStretchLastSection(True)
    health.setFocusPolicy(Qt.NoFocus)
    lbl_qc = QtWidgets.QLabel('-')
    lbl_qc.setWordWrap(True)
    nodes = QtWidgets.QLabel('-')
    nodes.setWordWrap(True)
    nodes.setTextFormat(Qt.RichText)
    hl.addWidget(lbl_qc)
    hl.addWidget(health, 1)
    hl.addWidget(nodes)
    tabs.addTab(hw, 'Health')

    # ------------------------------------------------------------------ refresh
    cache = {'ctx': cm.Ctx(), 'n_events': -1, 'n_log': -1, 'nodes_t': 0.0, 'wp_t': 0.0}

    def ctx_now():
        return cache['ctx']

    def refresh():
        L, live, events, log, keys, lost, fd = ros.snapshot()
        st = L.get('status') if live['status'] else None
        ctl = L.get('ctl') if live['ctl'] else None
        rs = L.get('state') if live['state'] else None
        mode = ctl.mode if ctl is not None else (rs.mode if rs is not None else None)
        c = cm.Ctx(st, mode, control_running=ctl is not None, state_live=rs is not None)
        cache['ctx'] = c
        now = time.time()
        lbl_clock.setText(time.strftime('%a %d %b %Y   %H:%M:%S'))
        ses = c.session
        sid = ses.get('session_id') or ros.session_param or 'no session'
        sim = ses.get('sim', ros.sim)
        lbl_session.setText(
            f'<span style="font-size:17px; font-weight:700">{sid}</span>'
            + (' <span style="background:#e0a100; padding:2px 6px; border-radius:3px"><b>SIM</b></span>' if sim else '')
            + f'<br><span style="color:#555">stage {ses.get("stage", "-")} · {ses.get("collection_mode", "-")}'
            f' · operator {ses.get("operator_id", "-")} · phantom {ses.get("phantom_id", "-")}'
            f' · robot_control {"on" if ses.get("robot_control") else "off"}</span>')
        ep = c.episode or {}
        state = c.ep_state
        if not c.manager:
            rec, rs_key = 'NO EPISODE MANAGER', 'none'
        elif c.paused:
            rec, rs_key = f'❚❚ PAUSED · Ep {ep.get("number")}', 'paused'
        elif state == 'recording':
            rec, rs_key = f'● REC · Ep {ep.get("number")} · {cm.fmt_s(ep.get("elapsed_s"))}', 'recording'
        elif state in ('starting', 'preparing', 'finishing'):
            rec, rs_key = cm.STATE_TEXT[state].split(' (')[0].upper(), 'busy'
        else:
            rec, rs_key = 'IDLE', 'idle'
        lbl_rec.setText(rec)
        lbl_rec.setStyleSheet(REC_STYLE[rs_key])
        p = cm.prompt(c)
        lbl_prompt.setText(p)
        lbl_prompt.setStyleSheet('background:%s; color:%s' % (
            ('#fde2e1', '#8e1b1b') if c.fault or not c.manager else
            ('#fff3cd', '#6b4e00') if c.paused or state in ('starting', 'preparing', 'finishing') else
            ('#e3f1e4', '#1e5a22') if state == 'recording' else ('#e8eef7', '#1f3d66')))

        # episode card
        lbl_epnum.setText(f'Episode {ep["number"]}' if ep.get('number') else 'Episode -')
        lbl_elapsed.setText(cm.fmt_s(ep.get('elapsed_s')) if ep.get('elapsed_s') is not None else '')
        marks = []
        for k, n in (('onset_t', 'onset'), ('recovery_start_t', 'recovery start'), ('recovery_end_t', 'recovery end')):
            if ep.get(k) is not None:
                marks.append(f'{n} {ep[k]:.1f} s')
        if ep.get('view_reached'):
            marks.append(f'view reached x{ep["view_reached"]}')
        vals = {'id': ep.get('episode_id'), 'date': ep.get('date'), 'start': ep.get('start_time'),
                'paused': cm.fmt_s(ep.get('paused_s')) + ('  (paused now)' if ep.get('paused') else '') if ep else None,
                'state': cm.STATE_TEXT.get(state, state) if c.manager else None,
                'type': ' / '.join(str(x) for x in (ep.get('type'), ep.get('target')) if x),
                'instruction': ep.get('instruction'), 'row': ep.get('plan_row'),
                'marks': ', '.join(marks), 'outcome': ep.get('outcome')}
        for k, v in vals.items():
            ep_lbl[k].setText('-' if v in (None, '') else str(v))
        last = (st or {}).get('last_episode')
        if last:
            qc = {True: 'QC pass', False: 'QC FAIL ' + ', '.join(last.get('qc_failed') or [])}.get(last.get('qc_pass'), 'QC -')
            lbl_last.setText(f'Last: episode {last.get("number")} · {last.get("outcome")} · '
                             f'{cm.fmt_s(last.get("duration_s"))} · {qc}')
        else:
            lbl_last.setText('Last: -')
        b_ss.setText(cm.start_stop_label(c))

        counts = (st or {}).get('counts') or {}
        plan = (st or {}).get('plan')
        oc = ', '.join(f'{k} {v}' for k, v in sorted((counts.get('by_outcome') or {}).items())) or '-'
        lbl_prog.setText(f'Episodes {counts.get("episodes", "-")}  ·  {oc}<br>'
                         f'QC failed {counts.get("qc_failed", "-")}  ·  kept recording {counts.get("kept_minutes", "-")} min')
        lbl_prog.setTextFormat(Qt.RichText)
        if plan and plan.get('rows'):
            bar.setVisible(True)
            bar.setMaximum(int(plan['rows']))
            bar.setValue(int(plan.get('done') or 0))
            bar.setFormat(f'plan rows %v / %m done · row {plan.get("current_row")} · redo {plan.get("redo", 0)}')
        else:
            bar.setVisible(False)

        # robot
        if ctl is None and rs is None:
            lbl_mode.setText('robot: no data')
            lbl_mode.setStyleSheet('color:#777')
        else:
            mn = cm.mode_name(mode)
            lbl_mode.setText(mn)
            lbl_mode.setStyleSheet('color:%s' % ('#c62828' if mn == 'FAULT' else '#1f5fae' if mn in ('TELEOP', 'FREEDRIVE')
                                                 else '#b26a00' if mn in ('MOVING', 'DESCENT', 'RETRACT', 'PERTURB') else '#333'))
        src = ctl if ctl is not None else rs
        rob_lbl['source'].setText(cm.source_name(src.action_source) if src is not None else '-')
        rob_lbl['expert'].setText(str(bool(src.action_is_expert)) if src is not None else '-')
        fl = cm.flag_names(src.safety_flags) if src is not None else []
        rob_lbl['flags'].setText(', '.join(fl) or 'none')
        rob_lbl['flags'].setStyleSheet('color:#a83232; font-weight:600' if fl else '')
        fsp = src.force_setpoint_n if src is not None else None
        rob_lbl['fsp'].setText('-' if fsp is None or fsp != fsp else f'{fsp:.1f} N')
        if rs is not None:
            T = geometry.pose_from_msg(rs.tcp_base)
            fz = float(T[:3, 2] @ np.asarray(rs.wrench[:3]))
            band = config.get_path(rig, 'limits.force_band_n')
            inb = '' if not band else ('  (in band)' if band[0] <= abs(fz) <= band[1] else '  (outside band)')
            rob_lbl['force'].setText(f'{fz:+.2f} N{inb}')
            pp = rs.tcp_phantom.position
            rob_lbl['pose'].setText(f'[{pp.x * 1000:.1f}, {pp.y * 1000:.1f}, {pp.z * 1000:.1f}] mm')
        else:
            rob_lbl['force'].setText('-')
            rob_lbl['pose'].setText('-')
        rob_lbl['input'].setText(f'LOST ({lost})' if lost else ('keys ' + ' '.join(sorted(keys)) if keys else 'ok, no keys held'))
        rob_lbl['input'].setStyleSheet('color:#a83232; font-weight:600' if lost else '')
        rob_lbl['streams'].setText('  '.join(f'{k} {"ok" if v else "—"}' for k, v in
                                             (('status', live['status']), ('control', live['ctl']),
                                              ('state', live['state']), ('qc', live['qc']))))

        # freedrive tab
        pedal = rs is not None and rs.mode == RobotState.MODE_FREEDRIVE
        fd_lbl['pedal'].setText('HELD: arm free' if pedal else ('released: arm locked' if rs is not None else '-'))
        fd_lbl['pedal'].setStyleSheet('color:#1f5fae; font-weight:700' if pedal else '')
        fd_lbl['labels'].setText(f'{cm.mode_name(rs.mode)} / {cm.source_name(rs.action_source)} / expert '
                                 f'{rs.action_is_expert}' if rs is not None else '-')
        fd_lbl['share'].setText(f'{fd["freedrive"]} of {fd["samples"]} decoded samples this episode'
                                if fd.get('episode') else '-')
        fd_lbl['control'].setText('RUNNING: stop it for freedrive collection' if ctl is not None else 'not running (correct for freedrive)')
        fd_lbl['control'].setStyleSheet('color:#a83232; font-weight:600' if ctl is not None else '')
        fd_lbl['input'].setText(str(config.get_path(rig, 'freedrive.input')))
        fd_lbl['grip'].setText(str(config.get_path(scfg, 'freedrive.grip')))

        # events and log
        if len(events) != cache['n_events']:
            cache['n_events'] = len(events)
            ev_list.clear()
            for t, eid, kind, v in reversed(events[-200:]):
                lab = v.get('phase') or v.get('state') or v.get('outcome') or v.get('motion') or ''
                if kind == 'motion':
                    lab = f'{v.get("motion")} {"ok" if v.get("ok") else v.get("message")}'
                ev_list.addItem(f'{time.strftime("%H:%M:%S", time.localtime(t * 1e-9))}  {kind:12s} {lab}  {eid}')
        if len(log) != cache['n_log']:
            cache['n_log'] = len(log)
            log_view.setPlainText('\n'.join(f'{time.strftime("%H:%M:%S", time.localtime(t))}  {x}' for t, x in log[-200:]))
            log_view.verticalScrollBar().setValue(log_view.verticalScrollBar().maximum())
        with ros.lock:
            lines = [pending.popleft() for _ in range(len(pending))]
        for line in lines:
            out.appendPlainText(line)

        # health
        q = L.get('qc') if live['qc'] else None
        if q:
            topics = q.get('topics') or {}
            health.setRowCount(len(topics))
            for i, (tn, s) in enumerate(sorted(topics.items())):
                for j, v in enumerate([tn, s.get('hz'), s.get('stamp_age_ms'), s.get('max_gap_ms')]):
                    health.setItem(i, j, QtWidgets.QTableWidgetItem(str(v)))
            lbl_qc.setText(f'micrus frame gaps {q.get("micrus_frame_gaps")} · coupling {q.get("coupling_ok")} · '
                           f'force in band {q.get("force_in_band")} · free disk {q.get("free_disk_gb")} GB · '
                           f'wrist USB {q.get("wrist_usb")}')
        else:
            lbl_qc.setText('qc_monitor: no status')
        if hasattr(ros, 'get_node_names_and_namespaces') and now - cache['nodes_t'] > 2:
            cache['nodes_t'] = now
            running = {n for n, _ in ros.get_node_names_and_namespaces()}
            nodes.setText('Nodes: ' + ' · '.join(
                f'<span style="color:{"#2e7d32" if n in running else "#a83232"}">{n}</span>' for n in EXPECTED_NODES)
                + ' · bag: ' + ('<b style="color:#c62828">recording</b>' if 'rosbag2_recorder' in running else 'idle'))
        if now - cache['wp_t'] > 10:
            cache['wp_t'] = now
            load_waypoints()
            if path_combo.count() == 0:
                load_paths()

        # enable / disable every action with its reason
        for action, ws in buttons.items():
            if action == 'gel_combo':
                ok, why = cm.allowed('gel', c)
            else:
                ok, why = cm.allowed(action, c)
            if ros.busy and action in ('goto', 'descend', 'retract', 'run_path', 'verify', 'teach', 'zero_ft',
                                       'teleop_on', 'teleop_off'):
                ok, why = False, f'{ros.busy} is running'
            for w in ws:
                w.setEnabled(ok)
                base = w.property('base_tip') or ''
                w.setToolTip(base if ok else (f'{base}\n' if base else '') + f'disabled: {why}')

    # Text fields take focus only when clicked, and nothing has focus at start: keys from the operator
    # keyboard (read by evdev whatever has focus) must not also type into a field.
    for w in win.findChildren(QtWidgets.QLineEdit) + win.findChildren(QtWidgets.QPlainTextEdit):
        if w.focusPolicy() != Qt.NoFocus:
            w.setFocusPolicy(Qt.ClickFocus)
    for w in win.findChildren(QtWidgets.QAbstractSpinBox):
        w.setFocusPolicy(Qt.ClickFocus)
    central.setFocusPolicy(Qt.StrongFocus)
    central.setFocus()

    win.ask_outcome = ask_outcome                  # replaceable in tests (modal dialog)
    load_paths()
    return win, refresh


def rig_force_step():
    """keymap.yaml force_step_n (the console's Force +/- use the keyboard's step), or None."""
    try:
        km = config.load_config('keymap', None)
        return float(km['force_step_n']) if km.get('force_step_n') is not None else None
    except Exception:  # noqa: BLE001
        return None


def main(args=None):
    from PySide6 import QtCore, QtWidgets
    rclpy.init(args=args)
    ros = ConsoleRos()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(ros)
    threading.Thread(target=ex.spin, daemon=True).start()
    import os
    os.nice(10)                                    # recording and control keep priority over the console
    app = QtWidgets.QApplication(sys.argv)
    win, refresh = build_window(ros)
    import signal
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    alive = QtCore.QTimer()
    alive.timeout.connect(lambda: None if rclpy.ok() else app.quit())
    alive.start(200)
    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(250)                               # 4 Hz: status, not video
    win.resize(1600, 950)
    win.show()
    rc = app.exec()
    ex.shutdown()
    ros.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
