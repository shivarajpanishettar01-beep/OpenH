"""Operator dashboard: PySide6 + pyqtgraph, rclpy in a worker thread.

All display subscriptions are best effort and keep only the latest message; the GUI redraws on
its own timer. Killing or restarting the dashboard cannot affect recording or robot control:
it only reads topics, sends /operator/command, and calls /robot/protective_stop and
/robot/zero_ft when the operator presses those buttons.

  ros2 run usrig_dashboard dashboard --ros-args -p session_id:=<id>
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
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from usrig_common import config, gates
from usrig_common.session import read_json, session_dir
from usrig_common.timeutil import ns_from_stamp
from usrig_interfaces.msg import EpisodeEvent, RobotState

MODES = ['IDLE', 'MOVING', 'DESCENT', 'TELEOP', 'PERTURB', 'RETRACT', 'FAULT', 'FREEDRIVE', 'POLICY']
SOURCES = ['teleop', 'scripted_task', 'transit', 'perturbation', 'none', 'kinesthetic', 'policy']
FLAGS = {1: 'envelope', 2: 'tilt', 4: 'force_max', 8: 'deadman', 16: 'speed'}


class RosSide(Node):
    """Collects the latest data; never blocks, never publishes anything but operator input."""

    def __init__(self):
        super().__init__('dashboard')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('session_id', '')
        self.declare_parameter('sim', False)
        cfg_dir = self.get_parameter('config_dir').value or None
        self.scfg = config.load_config('session', cfg_dir)
        self.rig = config.load_config('rig', cfg_dir)
        self.session_id = self.get_parameter('session_id').value
        self.sim = bool(self.get_parameter('sim').value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.lock = threading.Lock()
        self.latest = {}
        self.t_last = {}          # monotonic arrival time per stream: stale / absent streams are shown as such
        self.fps = {k: deque(maxlen=60) for k in ('bmode', 'color', 'depth')}
        self.force = deque(maxlen=5000)
        self.events = deque(maxlen=200)
        self.keys_held = set()
        ct = self.scfg['wrist_camera']['topics']
        self.create_subscription(Image, '/micrus/bmode', lambda m: self._img('bmode', m), qos)
        self.create_subscription(String, '/micrus/frame_info', lambda m: self._set('frame_info', m.data), qos)
        self.create_subscription(Image, ct['color'], lambda m: self._img('color', m), qos)
        self.create_subscription(Image, ct['depth'], lambda m: self._img('depth', m), qos)
        self.create_subscription(RobotState, '/robot/state', self._robot, qos)
        self.create_subscription(String, '/qc/status', lambda m: self._set('qc', m.data), qos)
        self.create_subscription(EpisodeEvent, '/episode/events', self._event,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(String, '/operator/command', self._cmd, QoSProfile(depth=50))
        self.pub_cmd = self.create_publisher(String, '/operator/command', 10)
        self.cli_pstop = self.create_client(Trigger, '/robot/protective_stop')
        self.cli_zero = self.create_client(Trigger, '/robot/zero_ft')

    def _set(self, k, v):
        with self.lock:
            self.latest[k] = v
            self.t_last[k] = time.monotonic()

    def _img(self, k, m):
        with self.lock:
            self.latest[k] = m
            self.t_last[k] = time.monotonic()
            self.fps[k].append(self.t_last[k])

    def _robot(self, m):
        with self.lock:
            self.latest['robot'] = m
            self.t_last['robot'] = time.monotonic()
            o = m.tcp_base.orientation
            x, y, z, w = o.x, o.y, o.z, o.w
            zaxis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
            self.force.append((ns_from_stamp(m.header.stamp) * 1e-9, float(zaxis @ np.asarray(m.wrench[:3]))))

    def _event(self, m):
        with self.lock:
            self.events.append((ns_from_stamp(m.header.stamp), m.episode_id, m.kind, m.value))

    def _cmd(self, m):
        try:
            c = json.loads(m.data)
        except ValueError:
            return
        if c.get('cmd') == 'key':
            with self.lock:
                (self.keys_held.add if c.get('down') else self.keys_held.discard)(c.get('key'))

    def send(self, payload):
        m = String()
        m.data = json.dumps(payload)
        self.pub_cmd.publish(m)

    def trigger(self, cli):
        if cli.service_is_ready():
            cli.call_async(Trigger.Request())
            return True
        return False


def to_array(m):
    if m.encoding in ('mono8', '8UC1'):
        return np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width]
    if m.encoding in ('16UC1', 'mono16'):
        return np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
    if m.encoding in ('rgb8', 'bgr8'):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
        a = a.reshape(m.height, m.width, 3)
        return a[:, :, ::-1] if m.encoding == 'bgr8' else a
    return None


STALE_S = {'robot': 0.5, 'bmode': 1.0, 'frame_info': 1.0, 'color': 1.0, 'depth': 1.0, 'qc': 2.0}
# Buttons that make episode_manager move the robot (approach, descend): asked to confirm first.
MOTION_BUTTONS = {'start_stop', 'next'}


def stream_state(t_last, now, timeout_s):
    """'live', 'STALE n.n s' or 'NO DATA': a display never shows an old value as current."""
    if t_last is None:
        return 'NO DATA'
    age = now - t_last
    return 'live' if age <= timeout_s else f'STALE {age:.1f} s'


def rate(dq, now=None, timeout_s=1.0):
    if len(dq) < 2:
        return 0.0
    if now is not None and now - dq[-1] > timeout_s:
        return 0.0                     # stream stopped: do not keep showing its last rate
    return (len(dq) - 1) / max(dq[-1] - dq[0], 1e-6)


def main(args=None):
    from PySide6 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg

    rclpy.init(args=args)
    ros = RosSide()
    ex = SingleThreadedExecutor()
    ex.add_node(ros)
    threading.Thread(target=ex.spin, daemon=True).start()

    try:
        from ament_index_python.packages import get_package_share_directory
        lay_file = Path(get_package_share_directory('usrig_dashboard')) / 'config' / 'dashboard_layout.yaml'
    except Exception:
        lay_file = Path(__file__).resolve().parents[1] / 'config' / 'dashboard_layout.yaml'
    layout = yaml.safe_load(lay_file.read_text())
    if layout.get('gui_nice'):
        import os
        os.nice(int(layout['gui_nice']))      # recording and control keep priority over the GUI
    from usrig_common.session import data_root
    root = data_root(ros.scfg, ros.sim)
    sdir = session_dir(root, ros.session_id) if ros.session_id else None
    band = config.get_path(ros.rig, 'limits.force_band_n')
    cap_s = float(ros.scfg['stage2']['cap_s'])

    pg.setConfigOptions(imageAxisOrder='row-major', antialias=False)
    app = QtWidgets.QApplication(sys.argv)
    win = QtWidgets.QMainWindow()
    win.setWindowTitle(f'usrig dashboard — {ros.session_id or "no session"}{"  [SIM]" if ros.sim else ""}')
    central = QtWidgets.QWidget()
    vbox = QtWidgets.QVBoxLayout(central)
    win.setCentralWidget(central)

    # ---- top bar
    top = QtWidgets.QHBoxLayout()
    lbl_top = QtWidgets.QLabel()
    lbl_top.setStyleSheet('font-weight: bold; font-size: 14px')
    btn_estop = QtWidgets.QPushButton('PROTECTIVE STOP')
    btn_estop.setStyleSheet('background: #c62828; color: white; font-weight: bold; padding: 8px 20px')
    btn_estop.clicked.connect(lambda: ros.trigger(ros.cli_pstop))
    btn_ack = QtWidgets.QPushButton('Acknowledge fault')
    btn_ack.clicked.connect(lambda: ros.send({'cmd': 'ack_fault'}))
    top.addWidget(lbl_top, 1)
    top.addWidget(btn_ack)
    top.addWidget(btn_estop)
    vbox.addLayout(top)

    tabs = QtWidgets.QTabWidget()
    vbox.addWidget(tabs, 1)

    # ---- collect view
    collect = QtWidgets.QWidget()
    grid = QtWidgets.QGridLayout(collect)
    panels = {}

    def box(title):
        g = QtWidgets.QGroupBox(title)
        lay = QtWidgets.QVBoxLayout(g)
        return g, lay

    g, lay = box('Ultrasound (B-mode)')
    bmode_plot = pg.PlotWidget()
    bmode_plot.invertY(True)
    bmode_plot.setAspectLocked(True)
    bmode_plot.setLabel('bottom', 'lateral', units='mm')
    bmode_plot.setLabel('left', 'depth', units='mm')
    bmode_img = pg.ImageItem()
    bmode_plot.addItem(bmode_img)
    lbl_bmode = QtWidgets.QLabel()
    lay.addWidget(bmode_plot, 1)
    lay.addWidget(lbl_bmode)
    panels['bmode'] = g

    g, lay = box('Wrist camera')
    h = QtWidgets.QHBoxLayout()
    color_view = pg.ImageView()
    depth_view = pg.ImageView()
    for v in (color_view, depth_view):
        v.ui.histogram.hide()
        v.ui.roiBtn.hide()
        v.ui.menuBtn.hide()
        h.addWidget(v)
    depth_view.setColorMap(pg.colormap.get('viridis'))
    lay.addLayout(h, 1)
    lbl_wrist = QtWidgets.QLabel()
    lay.addWidget(lbl_wrist)
    panels['wrist'] = g

    def send_checked(p):
        if p['cmd'] in MOTION_BUTTONS:
            r = QtWidgets.QMessageBox.question(
                win, 'Robot may move', f'"{p["cmd"]}" lets episode_manager move the robot (approach, descend) '
                'when robot_control is running. Area clear and e-stop at hand?')
            if r != QtWidgets.QMessageBox.StandardButton.Yes:
                return
        ros.send(p)

    g, lay = box('Episode')
    lbl_ep = QtWidgets.QLabel()
    lbl_ep.setWordWrap(True)
    lay.addWidget(lbl_ep, 1)
    row = QtWidgets.QHBoxLayout()
    for text, payload in (('Start/stop (S0)', {'cmd': 'start_stop'}), ('Success', {'cmd': 'outcome', 'value': 'success'}),
                          ('Recovered', {'cmd': 'outcome', 'value': 'recovered'}),
                          ('Fail', {'cmd': 'outcome', 'value': 'fail'}), ('Discard', {'cmd': 'discard'}),
                          ('Next', {'cmd': 'next'}), ('View reached', {'cmd': 'view_reached'}),
                          ('Onset', {'cmd': 'onset'}), ('Recovery start', {'cmd': 'recovery_start'}),
                          ('Recovery end', {'cmd': 'recovery_end'})):
        b = QtWidgets.QPushButton(text)
        b.clicked.connect(lambda _=False, p=payload: send_checked(p))
        row.addWidget(b)
    lay.addLayout(row)
    note = QtWidgets.QLineEdit()
    note.setPlaceholderText('note (Enter to add)')
    note.returnPressed.connect(lambda: (ros.send({'cmd': 'note', 'value': note.text()}), note.clear()))
    lay.addWidget(note)
    panels['episode'] = g

    g, lay = box('Force')
    force_plot = pg.PlotWidget()
    force_plot.setLabel('left', 'F probe axis', units='N')
    force_plot.setLabel('bottom', 't', units='s')
    force_curve = force_plot.plot(pen='y')
    if band:
        for b in band:
            force_plot.addItem(pg.InfiniteLine(pos=b, angle=0, pen=pg.mkPen('g', style=QtCore.Qt.DashLine)))
    lbl_wrench = QtWidgets.QLabel()
    lay.addWidget(force_plot, 1)
    lay.addWidget(lbl_wrench)
    panels['force'] = g

    g, lay = box('Pose and teleop')
    lbl_teleop = QtWidgets.QLabel()
    lbl_teleop.setWordWrap(True)
    lay.addWidget(lbl_teleop)
    panels['teleop'] = g

    g, lay = box('Data health')
    health = QtWidgets.QTableWidget(0, 4)
    health.setHorizontalHeaderLabels(['topic', 'Hz', 'stamp age ms', 'max gap ms'])
    health.horizontalHeader().setStretchLastSection(True)
    lbl_qc = QtWidgets.QLabel()
    lay.addWidget(health, 1)
    lay.addWidget(lbl_qc)
    panels['health'] = g

    g, lay = box('Nodes')
    node_table = QtWidgets.QTableWidget(0, 2)
    node_table.setHorizontalHeaderLabels(['node', 'state'])
    node_table.horizontalHeader().setStretchLastSection(True)
    lay.addWidget(node_table)
    panels['nodes'] = g

    g, lay = box('Session progress')
    lbl_prog = QtWidgets.QLabel()
    lbl_prog.setWordWrap(True)
    lay.addWidget(lbl_prog)
    panels['progress'] = g

    for name, pos in layout['collect'].items():
        if name in panels:
            grid.addWidget(panels[name], *pos)
    tabs.addTab(collect, 'Collect')

    # ---- session tab (wizards are the CLI tools; the dashboard shows status and launches them)
    sess = QtWidgets.QWidget()
    sl = QtWidgets.QVBoxLayout(sess)
    btn_zero = QtWidgets.QPushButton('Zero F/T sensor (free air only)')
    btn_zero.clicked.connect(lambda: ros.trigger(ros.cli_zero))
    sl.addWidget(btn_zero)
    lbl_sess = QtWidgets.QLabel()
    lbl_sess.setWordWrap(True)
    lbl_sess.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    sl.addWidget(lbl_sess, 1)
    tabs.addTab(sess, 'Session')

    for title, text in (('Teach', 'Teach waypoints:  ros2 run usrig_presets waypoint_teach ~/data/sessions/<session>\n'
                                  'Verify:            ros2 run usrig_presets waypoint_verify ~/data/sessions/<session>\n'
                                  'Paths (Stage 1):   ros2 run usrig_presets path_editor --help'),
                        ('Plan', 'Generate:  ros2 run usrig_presets plan_generator generate --help   (-> ~/data/plans/)\n'
                                 'Lock:      ros2 run usrig_presets plan_generator lock --plan ~/data/plans/<plan>.csv '
                                 '--session ~/data/sessions/<session>')):
        w = QtWidgets.QLabel(text)
        w.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        w.setAlignment(QtCore.Qt.AlignTop)
        tabs.addTab(w, title)

    # ---- review tab
    review = QtWidgets.QWidget()
    rl = QtWidgets.QVBoxLayout(review)
    ep_table = QtWidgets.QTableWidget(0, 6)
    ep_table.setHorizontalHeaderLabels(['episode', 'stage', 'type', 'outcome', 'QC', 'failed checks'])
    ep_table.horizontalHeader().setStretchLastSection(True)
    rl.addWidget(ep_table, 1)
    rb = QtWidgets.QHBoxLayout()
    btn_reload = QtWidgets.QPushButton('Reload')
    btn_redo = QtWidgets.QPushButton('Mark redo')
    btn_pj = QtWidgets.QPushButton('Open in PlotJuggler')
    for b in (btn_reload, btn_redo, btn_pj):
        rb.addWidget(b)
    rl.addLayout(rb)
    tabs.addTab(review, 'Review')

    def load_review():
        if not sdir:
            return
        eps = sorted(sdir.glob('ep_*/episode.json'))
        ep_table.setRowCount(len(eps))
        for i, f in enumerate(eps):
            e = json.loads(f.read_text())
            qc = e.get('qc') or {}
            vals = [f.parent.name, e.get('stage'), e.get('type'), e.get('outcome'),
                    'pass' if qc.get('pass') else 'FAIL', ', '.join(qc.get('failed', []))]
            for j, v in enumerate(vals):
                ep_table.setItem(i, j, QtWidgets.QTableWidgetItem(str(v)))

    def selected_ep():
        r = ep_table.currentRow()
        return sdir / ep_table.item(r, 0).text() if r >= 0 and sdir else None

    def mark_redo():
        d = selected_ep()
        if d is None:
            return
        e = json.loads((d / 'episode.json').read_text())
        if e.get('plan_row') is not None:
            prog = read_json(sdir / 'plan_progress.json')
            prog[str(e['plan_row'])] = {'status': 'redo', 'episode_id': e['episode_id'], 'by': 'review'}
            from usrig_common.session import write_json_atomic
            write_json_atomic(sdir / 'plan_progress.json', prog)

    btn_reload.clicked.connect(load_review)
    btn_redo.clicked.connect(mark_redo)
    btn_pj.clicked.connect(lambda: selected_ep() and subprocess.Popen(
        ['ros2', 'run', 'plotjuggler', 'plotjuggler', '-d', str(selected_ep())]))

    # ---- refresh
    gate_cache = {'t': 0, 'v': None}

    def refresh():
        with ros.lock:
            L = dict(ros.latest)
            now = time.monotonic()
            fps = {k: rate(v, now) for k, v in ros.fps.items()}
            live = {k: stream_state(ros.t_last.get(k), now, t) for k, t in STALE_S.items()}
            force = list(ros.force)
            events = list(ros.events)
            keys = sorted(ros.keys_held)
        robot = L.get('robot')
        rec = 'IDLE'
        ep_info = {}
        for t, eid, kind, val in events:
            if kind == 'start':
                rec, ep_info = 'REC', {'id': eid, 't0': t, **json.loads(val or '{}')}
            elif kind == 'end':
                rec = 'IDLE'
        mode = MODES[robot.mode] if robot is not None and robot.mode < len(MODES) else '—'
        if live['robot'] != 'live':
            mode = f'state {live["robot"]}'
        lbl_top.setText(f'session {ros.session_id}   stage {ros.scfg["session"]["stage"]}   '
                        f'phantom {ros.scfg["session"].get("phantom_id")}   robot {mode}   {rec}')
        lbl_top.setStyleSheet('font-weight: bold; font-size: 14px; color: %s' %
                              ('#c62828' if mode == 'FAULT' or mode.startswith('state ') else
                               ('#2e7d32' if rec == 'REC' else 'black')))

        fi = json.loads(L['frame_info']) if 'frame_info' in L else {}
        if 'bmode' in L:
            a = to_array(L['bmode'])
            if a is not None:
                bmode_img.setImage(a, autoLevels=False, levels=(0, 255))
                im = fi.get('image', {})
                ps = im.get('pixel_spacing_mm')
                org = im.get('origin_mm') or [0, 0]
                if ps:
                    sx, sy = (ps if isinstance(ps, (list, tuple)) else (ps, ps))[:2]
                    bmode_img.setRect(QtCore.QRectF(org[0], org[1], a.shape[1] * sx, a.shape[0] * sy))
        lbl_bmode.setText(f'[{live["bmode"]}]  frame {fi.get("frame_index")}   {fps["bmode"]:.1f} fps   '
                          f'stage {ros.scfg["session"]["stage"]}')
        if 'color' in L:
            a = to_array(L['color'])
            if a is not None:
                color_view.setImage(a, autoLevels=False, levels=(0, 255), autoRange=False)
        vfrac = None
        if 'depth' in L:
            d = to_array(L['depth'])
            if d is not None:
                vfrac = float(np.mean(d > 0))
                depth_view.setImage(d.astype(np.float32), autoLevels=True, autoRange=False)
        lbl_wrist.setText(f'colour [{live["color"]}] {fps["color"]:.1f} fps   depth [{live["depth"]}] {fps["depth"]:.1f} fps   '
                          f'valid depth {vfrac if vfrac is None else f"{vfrac:.0%}"}')

        if force:
            t = np.array([x[0] for x in force])
            f = np.array([x[1] for x in force])
            m = t > t[-1] - layout.get('force_window_s', 10)
            force_curve.setData(t[m] - t[-1], f[m])
        if robot is not None:
            w = list(robot.wrench)
            lbl_wrench.setText('F [%.1f %.1f %.1f] N   T [%.2f %.2f %.2f] Nm   setpoint %.1f N' %
                               (*w, robot.force_setpoint_n))
            flags = [n for b, n in FLAGS.items() if robot.safety_flags & b]
            p = robot.tcp_phantom.position
            lbl_teleop.setText(f'keys held: {" ".join(keys) or "—"}\n'
                               f'source: {SOURCES[robot.action_source] if robot.action_source < len(SOURCES) else "?"}  '
                               f'expert: {robot.action_is_expert}\n'
                               f'TCP (phantom) [{p.x * 1000:.1f}, {p.y * 1000:.1f}, {p.z * 1000:.1f}] mm\n'
                               f'safety flags: {", ".join(flags) or "none"}')

        if ep_info and rec == 'REC':
            el = (time.time_ns() - ep_info['t0']) * 1e-9
            left = cap_s - el
            lbl_ep.setText(f'<b>{ep_info.get("id")}</b>  type {ep_info.get("type")}  target {ep_info.get("target")}<br>'
                           f'<i>{ep_info.get("instruction") or ""}</i><br>'
                           f'elapsed {el:.1f} s   {"cap in %.0f s" % left if ros.scfg["session"]["stage"] == 2 else ""}')
        elif rec == 'IDLE':
            lbl_ep.setText('not recording')

        if 'qc' in L:
            q = json.loads(L['qc'])
            topics = q.get('topics', {})
            health.setRowCount(len(topics))
            for i, (tn, s) in enumerate(sorted(topics.items())):
                for j, v in enumerate([tn, s.get('hz'), s.get('stamp_age_ms'), s.get('max_gap_ms')]):
                    health.setItem(i, j, QtWidgets.QTableWidgetItem(str(v)))
            lbl_qc.setText(f'QC [{live["qc"]}]   micrus gaps {q.get("micrus_frame_gaps")}   coupling {q.get("coupling_ok")}   '
                           f'force in band {q.get("force_in_band")}   disk {q.get("free_disk_gb")} GB   '
                           f'USB {q.get("wrist_usb")}')

        if time.monotonic() - gate_cache.get('nodes_t', 0) > 2:
            gate_cache['nodes_t'] = time.monotonic()
            running = {n for n, _ in ros.get_node_names_and_namespaces()}
            names = list(layout.get('expected_nodes', [])) + [layout.get('recording_node', 'rosbag2_recorder')]
            node_table.setRowCount(len(names))
            for i, n in enumerate(names):
                up = n in running
                if n == layout.get('recording_node'):
                    txt = 'recording' if up else 'not recording'
                else:
                    txt = 'running' if up else 'MISSING'
                node_table.setItem(i, 0, QtWidgets.QTableWidgetItem(n))
                it = QtWidgets.QTableWidgetItem(txt)
                it.setForeground(QtGui.QColor('#2e7d32' if up else ('#757575' if n == layout.get('recording_node') else '#c62828')))
                node_table.setItem(i, 1, it)

        if sdir and time.monotonic() - gate_cache['t'] > 5:
            gate_cache['t'] = time.monotonic()
            st = int(ros.scfg['session']['stage'])
            gate_cache['v'] = gates.check_stage(st, ros.scfg, root)
            prog = read_json(sdir / 'plan_progress.json')
            done = sum(1 for v in prog.values() if v.get('status') == 'done')
            redo = [k for k, v in prog.items() if v.get('status') == 'redo']
            eps = [json.loads(f.read_text()) for f in sorted(sdir.glob('ep_*/episode.json'))]
            strip = ''.join({'success': 'S', 'recovered': 'R', 'fail': 'F', 'discard': 'x'}.get(e.get('outcome'), '?')
                            for e in eps[-40:])
            by_type = {}
            for e in eps:
                if e.get('outcome') != 'discard':
                    by_type[e.get('type')] = by_type.get(e.get('type'), 0) + 1
            lbl_prog.setText(f'stage {st} gate passed: {gate_cache["v"].get("passed")}<br>'
                             f'plan rows done {done}, redo queue {redo[:20]}<br>'
                             f'episodes by type {by_type}<br>outcomes: {strip}')
            lbl_sess.setText(json.dumps(read_json(sdir / 'session.json'), indent=1)[:4000])

    # Ctrl+C / SIGTERM shut rclpy down; the Qt loop must then quit too (otherwise the window lives on).
    import signal
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    alive = QtCore.QTimer()
    alive.timeout.connect(lambda: None if rclpy.ok() else app.quit())   # also lets Python run signal handlers
    alive.start(200)

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(int(1000 / layout.get('refresh_hz', 10)))
    load_review()
    win.resize(1600, 1000)
    win.show()
    rc = app.exec()
    ex.shutdown()
    ros.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
