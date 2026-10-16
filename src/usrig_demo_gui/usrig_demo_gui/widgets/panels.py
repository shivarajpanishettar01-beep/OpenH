"""The dashboard panels. Each one takes the matching part of DashboardState in update_state()."""
from __future__ import annotations

import math

from PySide6 import QtCore, QtWidgets

from .. import theme
from ..models import (PHASE_TEXT, ContactState, DashboardState, Health, RecordingStatus,
                      StreamStatus, TaskPhase, UltrasoundFrame)
from ..frames import ToolFrames
from .common import DASH, U, Bar, ElidedLabel, Swatch, ImageView, KVList, Panel, fmt, mono_font, style, vline


def make_robot_view(phantom, mode='auto', trajectory=None):
    """Mesh view (OpenGL) when available, else the simple QPainter skeleton."""
    if mode in ('auto', 'gl'):
        try:
            from .robot_view_gl import RobotViewGL
            return RobotViewGL(phantom, trajectory)
        except Exception:
            if mode == 'gl':
                raise
    from .robot_view import RobotView
    return RobotView(phantom, trajectory)


# ====================================================================== A: robot
LEGEND_TIP = ('Trajectory map from the last rest position: the run starts when the robot leaves a rest '
              '(stationary, no episode running) and is replaced when it moves off again.\n'
              'Paths come from the controller TCP pose (/robot/tcp_pose) and the rig calibration:\n'
              '  flange = TCP x inv(robot.tcp_setting)\n'
              '  probe tip = TCP if probe.tcp_already_includes_probe, else flange x probe.tip_offset_flange_m\n'
              '  wrist camera = flange x hand-eye (flange -> wrist_camera on /tf_static)\n'
              'Dots mark the rest position and each task phase change (from /episode/events).')


class RobotStatePanel(Panel):
    def __init__(self, phantom=None, view_mode='auto', trajectory=None):
        super().__init__('Robot state')
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(0)
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(U // 2)
        self.view = make_robot_view(phantom, view_mode, trajectory)
        left.addWidget(self.view, 1)
        left.addLayout(self._legend())
        row.addLayout(left, 1)
        row.addSpacing(U + 4)
        row.addWidget(vline())
        row.addSpacing(U + 4)
        self.kv = KVList([('#', 'TCP position (m)'), ('x', 'X'), ('y', 'Y'), ('z', 'Z'),
                          ('#', 'TCP rotation vector (rad)'), ('rx', 'Rx'), ('ry', 'Ry'), ('rz', 'Rz'),
                          ('#', 'Joint angles (deg)')] + [(f'j{i}', f'J{i}') for i in range(1, 7)] +
                         [('#', 'Controller'), ('mode', 'Mode'), ('flags', 'Safety flags')], label_w=56)
        self.kv.setFixedWidth(176)
        row.addWidget(self.kv)
        self.body.addLayout(row, 1)

    NAMES = {'flange': 'Flange', 'tip': 'Probe tip, contact', 'air': 'Probe tip, air', 'camera': 'Wrist camera'}

    def _legend(self):
        """Trajectory key under the 3D view."""
        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(6)
        self.legend = {}
        for key, col in (('flange', theme.TRAJ_FLANGE), ('tip', theme.TRAJ_TIP), ('air', theme.TRAJ_TIP_AIR),
                         ('camera', theme.TRAJ_CAMERA)):
            sw = Swatch(col)
            lab = QtWidgets.QLabel(self.NAMES[key])
            lab.setObjectName('small')
            lab.setToolTip(LEGEND_TIP)
            row.addWidget(sw)
            row.addWidget(lab)
            row.addSpacing(U)
            self.legend[key] = (sw, lab)
        mk = QtWidgets.QLabel('\u25cf phase change')
        mk.setObjectName('small')
        mk.setToolTip(LEGEND_TIP)
        row.addWidget(mk)
        self.legend['marker'] = (mk,)
        self.legend_none = QtWidgets.QLabel('Trajectory map: needs the rig calibration (hover for details)')
        self.legend_none.setObjectName('small')
        self.legend_none.hide()
        row.addWidget(self.legend_none)
        row.addStretch(1)
        return row

    def update_state(self, d: DashboardState):
        r = d.robot
        contact = {ContactState.IN_CONTACT: True, ContactState.NO_CONTACT: False}.get(d.contact)
        frames = ToolFrames(r.T_flange, r.T_tip, r.T_camera) if r.connected else ToolFrames()
        self.view.set_state(r.joints_rad, r.connected, r.mode, frames, contact,
                            d.task.phase.value if d.task.phase else None)
        # legend: say which paths are not available and why (calibration still null in rig.yaml)
        miss = r.frames_missing or {}
        known = contact is not None                    # contact unknown: one colour, plain "Probe tip"
        for key in ('flange', 'tip', 'camera'):
            sw, lab = self.legend[key]
            name = self.NAMES[key] if (key != 'tip' or known) else 'Probe tip'
            if key in miss:
                lab.setText(f'{name.split(",")[0]} (not calibrated)')
                style(lab, f'color: {theme.FAINT}; font-size: {theme.SMALL_PX}px;')
                lab.setToolTip(f'needs {miss[key]} in rig.yaml\n\n' + LEGEND_TIP)
            else:
                lab.setText(name)
                style(lab, f'color: {theme.MUTED}; font-size: {theme.SMALL_PX}px;')
                lab.setToolTip(LEGEND_TIP)
        for w in self.legend['air']:
            w.setVisible(known and 'tip' not in miss)
        none = r.connected and {'flange', 'tip', 'camera'} <= set(miss)   # nothing can be drawn: one line
        self.legend_none.setVisible(none)
        if none:
            self.legend_none.setToolTip('needs in rig.yaml: ' + ', '.join(sorted(set(miss.values()))) +
                                        '\n\n' + LEGEND_TIP)
        for key in ('flange', 'tip', 'camera', 'marker'):
            for w in self.legend[key]:
                w.setVisible(not none)
        for w in self.legend['air']:
            w.setVisible(not none and known and 'tip' not in miss)
        self.set_info('' if r.connected else 'No robot state' + (', showing the last known pose'
                                                                  if getattr(self.view, 'has_state', False) else ''),
                      None if r.connected else theme.RED)
        p = r.tcp_pos_m if r.connected else None
        for i, k in enumerate('xyz'):
            self.kv.set(k, fmt(None if p is None else p[i], '{:+.4f}'))
        rv = r.tcp_rotvec if r.connected else None
        for i, k in enumerate(('rx', 'ry', 'rz')):
            self.kv.set(k, fmt(None if rv is None else rv[i], '{:+.3f}'))
        q = r.joints_rad if r.connected else None
        for i in range(6):
            self.kv.set(f'j{i + 1}', fmt(None if q is None else math.degrees(q[i]), '{:+.1f}'))
        self.kv.set('mode', (r.mode or DASH) if r.connected else DASH)
        flags = ', '.join(r.safety_flags) if r.safety_flags else ('none' if r.connected else DASH)
        self.kv.set('flags', flags, theme.ORANGE if r.safety_flags else None)


# ====================================================================== B: ultrasound
class UltrasoundPanel(Panel):
    def __init__(self):
        super().__init__('B-mode ultrasound')
        self.view = ImageView('gray', depth_ruler=True)
        self.body.addWidget(self.view, 1)
        self.meta = QtWidgets.QLabel()
        self.meta.setTextFormat(QtCore.Qt.RichText)
        self.meta.setFixedHeight(theme.ROW_H)
        self.body.addWidget(self.meta)

    def update_state(self, d: DashboardState):
        u: UltrasoundFrame = d.ultrasound
        # the ruler spans the image actually shown (imaged depth), not the depth setting in the label
        ruler_mm = u.image.shape[0] * u.pixel_mm[1] if u.image is not None else u.depth_mm
        self.view.set_frame(u.image, u.status, pixel_mm=u.pixel_mm, depth_mm=ruler_mm)
        live = u.status == StreamStatus.LIVE
        self.set_info(fmt(u.fps, '{:.1f}', 'fps') if live else u.status.value.capitalize(),
                      None if live else (theme.RED if u.status == StreamStatus.DISCONNECTED else theme.ORANGE))
        parts = [('Depth', fmt(u.depth_mm, '{:g}', 'mm')), ('Gain', fmt(u.gain, '{:g}')),
                 ('Frequency', fmt(u.freq_mhz, '{:g}', 'MHz')),
                 ('Frame', fmt(u.frame_index if live else None, '{:d}'))]
        sep = '&nbsp;' * 6
        self.meta.setText(sep.join(f'<span style="color:{theme.MUTED}">{k}</span>&nbsp;&nbsp;{v}' for k, v in parts))
        self.meta.setToolTip('Gain / frequency show — when the scanner does not report them '
                             '(session.yaml leaves them at the scanner default).')


# ====================================================================== C: wrist cameras
class WristCamerasPanel(Panel):
    """RGB and depth side by side, each with a one-line caption (stream, rate or state)."""

    def __init__(self):
        super().__init__('Wrist cameras')
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(U)
        self.views, self.caps = {}, {}
        for key, kind in (('rgb', 'rgb'), ('depth', 'depth')):
            col = QtWidgets.QVBoxLayout()
            col.setSpacing(U // 2)
            v = ImageView(kind)
            v.setMinimumSize(120, 90)
            cap = QtWidgets.QLabel()
            cap.setObjectName('small')
            cap.setFixedHeight(theme.ROW_H - 4)
            col.addWidget(v, 1)
            col.addWidget(cap)
            row.addLayout(col, 1)
            self.views[key], self.caps[key] = v, cap
        self.body.addLayout(row, 1)

    def update_state(self, d: DashboardState):
        for key, c in (('rgb', d.rgb), ('depth', d.depth)):
            self.views[key].set_frame(c.image, c.status, depth_range=c.depth_range_m)
            live = c.status == StreamStatus.LIVE
            name = 'Colour' if key == 'rgb' else f'Depth, {c.depth_range_m[0]:g} to {c.depth_range_m[1]:g} m'
            state = fmt(c.fps, '{:.1f}', 'fps') if live else c.status.value
            self.caps[key].setText(f'{name}, {state}')
            col = theme.MUTED if live else (theme.RED if c.status == StreamStatus.DISCONNECTED else theme.ORANGE)
            style(self.caps[key], f'color: {col}; font-size: {theme.SMALL_PX}px;')


# ====================================================================== E: contact and image quality
class QualityPanel(Panel):
    """Contact and image metrics (display aids, not validated success criteria)."""

    def __init__(self):
        super().__init__('Contact and image')
        self.kv = None
        self.keys = []

    def update_state(self, d: DashboardState):
        m = d.metrics
        names = [it.name for it in m.items] + ['Image quality']
        if names != self.keys:                       # build once (or when the metric set changes)
            if self.kv is not None:
                self.kv.setParent(None)
            self.kv = KVList([(n, n) for n in names], label_w=140)
            self.body.addWidget(self.kv, 1)
            self.keys = names
        for it in m.items:
            if it.value is None:
                self.kv.set(it.name, 'n/a', theme.FAINT, tip=it.note)
            else:
                col = theme.ORANGE if it.level == Health.WARN else None
                tip = it.note + (f'\nsource: {it.source}' if it.source else '')
                self.kv.set(it.name, fmt(it.value, it.fmt, it.unit), col, tip=tip)
        tip = 'Coupling check as in qc_monitor: mean B-mode intensity above the coupling floor.'
        if m.image_quality is None:
            self.kv.set('Image quality', 'unknown', theme.FAINT, tip=tip)
        else:
            col = theme.HEALTH_COLOR[m.image_quality_level] if m.image_quality_level != Health.OK else None
            self.kv.set('Image quality', m.image_quality, col, tip=tip)


# ====================================================================== F: episode
class EpisodeInfoPanel(Panel):
    """Episode identity and state, and the most recent events (episode time)."""

    N_EVENTS = 5

    def __init__(self):
        super().__init__('Episode')
        self.task = KVList([('episode', 'Episode'), ('phase', 'Phase'), ('elapsed', 'Elapsed'),
                            ('outcome', 'Outcome')], label_w=80, left=True)
        self.body.addWidget(self.task)
        ev = QtWidgets.QGridLayout()
        ev.setHorizontalSpacing(U)
        ev.setVerticalSpacing(0)
        ev.setColumnMinimumWidth(0, 80)
        ev.setColumnStretch(1, 1)
        self.ev_rows = []
        for i in range(self.N_EVENTS):
            t = QtWidgets.QLabel()
            t.setObjectName('muted')
            t.setFixedHeight(theme.ROW_H)
            x = ElidedLabel('', QtCore.Qt.ElideRight)
            x.setFixedHeight(theme.ROW_H)
            ev.addWidget(t, i, 0)
            ev.addWidget(x, i, 1)
            self.ev_rows.append((t, x))
        ev.setRowStretch(self.N_EVENTS, 1)
        self.body.addLayout(ev, 1)

    def update_state(self, d: DashboardState):
        t = d.task
        ph = t.phase
        r = d.recording
        self.task.set('episode', r.episode_id or DASH, tip=r.output_path or '')
        if ph is None:
            self.task.set('phase', 'Idle' if t.outcome is None else 'Finished', theme.MUTED)
        else:
            self.task.set('phase', ph.value, theme.ORANGE if ph == TaskPhase.RECOVERY else None,
                          tip=PHASE_TEXT[ph] + (f'\nInstruction: {t.detail}' if t.detail else ''))
        if r.elapsed_s is not None and r.total_s:
            self.task.set('elapsed', f'{r.elapsed_s:.1f} of {r.total_s:.0f} s')
        else:
            self.task.set('elapsed', DASH)
        oc = t.outcome
        oc_col = {'SUCCESS': theme.GREEN, 'FAILURE': theme.RED, 'ABORTED': theme.ORANGE}.get(oc.name if oc else '')
        self.task.set('outcome', oc.value if oc else DASH, oc_col)
        col = {Health.OK: theme.TEXT, Health.WARN: theme.ORANGE, Health.DOWN: theme.RED, Health.UNKNOWN: theme.TEXT}
        evs = list(reversed(t.events))[:self.N_EVENTS]
        for i, (tl, xl) in enumerate(self.ev_rows):
            if i < len(evs):
                tl.setText(f'{evs[i].t_s:.1f} s')
                xl.setText(evs[i].text)
                style(xl, f'color: {col[evs[i].level]};')
            else:
                tl.setText('Events' if i == 0 and not evs else '')
                xl.setText('None yet' if i == 0 and not evs else '')
                style(xl, f'color: {theme.FAINT};')


# ====================================================================== G: recording
class RecordingStatusPanel(Panel):
    def __init__(self):
        super().__init__('Recording and dataset')
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(U)
        self.state = QtWidgets.QLabel()
        self.clock = QtWidgets.QLabel()
        self.clock.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        top.addWidget(self.state)
        top.addStretch(1)
        top.addWidget(self.clock)
        w = QtWidgets.QWidget()
        w.setFixedHeight(theme.ROW_H)
        w.setLayout(top)
        top.setContentsMargins(0, 0, 0, 0)
        self.body.addWidget(w)
        self.bar = Bar(4)
        self.body.addWidget(self.bar)
        self.kv = KVList([('session', 'Session'), ('episode', 'Episode'), ('path', 'Output')], elide=True)
        self.body.addWidget(self.kv)
        self.warn = QtWidgets.QLabel()
        self.warn.setWordWrap(True)
        self.warn.setTextFormat(QtCore.Qt.RichText)
        self.warn.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.body.addWidget(self.warn, 1)
        for v in self.kv.vals.values():
            v.setFont(mono_font())

    def update_state(self, d: DashboardState):
        r = d.recording
        color = {RecordingStatus.RECORDING: theme.RED, RecordingStatus.FINALIZING: theme.ORANGE,
                 RecordingStatus.IDLE: theme.GREY, RecordingStatus.UNKNOWN: theme.GREY}[r.status]
        self.state.setText(r.status.value.upper())
        style(self.state, f'font-weight: 600; color: {color if r.status == RecordingStatus.RECORDING else theme.MUTED};')

        def mmss(s):
            return '--:--' if s is None else f'{int(s // 60):02d}:{s % 60:04.1f}'
        self.clock.setText(f'{mmss(r.elapsed_s)} / {mmss(r.total_s)}' if r.total_s else mmss(r.elapsed_s))
        self.kv.set('session', r.session_id or DASH)
        self.kv.set('episode', r.episode_id or DASH)
        self.kv.set('path', r.output_path or DASH)
        frac = (r.elapsed_s / r.total_s) if (r.elapsed_s is not None and r.total_s) else 0
        self.bar.set_value(frac, theme.RED if r.status == RecordingStatus.RECORDING else theme.GREY)
        if r.warnings:
            self.warn.setText('<br>'.join(f'<span style="color:{theme.ORANGE}">{w}</span>' for w in r.warnings))
        else:
            self.warn.setText(f'<span style="color:{theme.FAINT}">'
                              f'{"recorder state unknown" if r.status == RecordingStatus.UNKNOWN else "no data warnings"}'
                              '</span>')


# ====================================================================== H: ROS 2 nodes
STATUS_TEXT = {Health.OK: ('Running', theme.GREEN), Health.WARN: ('Rate off', theme.ORANGE),
               Health.DOWN: ('Down', theme.RED), Health.UNKNOWN: ('\u2014', theme.FAINT)}


class ROSNodeHealthPanel(Panel):
    """Table: node | topic | rate | status. Status is text, coloured only by state."""

    def __init__(self):
        super().__init__('ROS 2 nodes')
        self.grid = QtWidgets.QGridLayout()
        self.grid.setHorizontalSpacing(U + 4)
        self.grid.setVerticalSpacing(0)
        self.grid.setColumnMinimumWidth(0, 108)
        self.grid.setColumnStretch(1, 1)
        self.grid.setColumnMinimumWidth(2, 58)
        self.grid.setColumnMinimumWidth(3, 58)
        for j, h in enumerate(('Node', 'Topic', 'Rate', 'Status')):
            lab = QtWidgets.QLabel(h)
            lab.setObjectName('section')
            lab.setFixedHeight(theme.ROW_H)
            if j == 2:
                lab.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.grid.addWidget(lab, 0, j)
        self.body.addLayout(self.grid)
        self.body.addStretch(1)
        self.rows = []

    def update_state(self, d: DashboardState):
        nodes = d.nodes
        while len(self.rows) < len(nodes):
            i = len(self.rows) + 1
            name = QtWidgets.QLabel()
            topic = ElidedLabel('', QtCore.Qt.ElideLeft)
            topic.setFont(mono_font())
            style(topic, f'color: {theme.MUTED};')
            rate = QtWidgets.QLabel()
            rate.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            status = QtWidgets.QLabel()
            for j, w in enumerate((name, topic, rate, status)):
                w.setFixedHeight(theme.ROW_H)
                self.grid.addWidget(w, i, j)
            self.rows.append((name, topic, rate, status))
        ok = sum(n.health == Health.OK for n in nodes)
        self.set_info(f'{ok} of {len(nodes)} running', theme.ORANGE if ok < len(nodes) else None)
        for (name, topic, rate, status), n in zip(self.rows, nodes):
            name.setText(n.name)
            topic.setText(n.topic)
            if n.rate_hz is not None:
                rate.setText(f'{n.rate_hz:.1f} Hz' if n.rate_hz < 100 else f'{n.rate_hz:.0f} Hz')
            elif n.expected in ('recording', 'idle'):
                rate.setText(DASH)
            else:
                rate.setText(DASH)
            text, col = STATUS_TEXT[n.health]
            if n.expected == 'recording':
                text = 'Recording'
            status.setText(text)
            style(status, f'color: {col}; font-size: {theme.SMALL_PX}px; font-weight: 500;')
            style(rate, f'color: {theme.ORANGE if n.health == Health.WARN else theme.TEXT};')
            tip = f'{n.name}\n{n.topic}\nexpected: {n.expected or "-"}\nstate: {n.health.value}'
            if name.toolTip() != tip:
                for w in (name, rate, status):
                    w.setToolTip(tip)
