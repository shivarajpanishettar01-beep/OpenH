"""Main window: header, panes, refresh timer.

Wide layout (lab monitor):
    +------------------------+------------------------+-----------------+
    | Robot (3D + map)       | B-mode ultrasound      | Wrist cameras   |
    |                        |                        | Contact & image |
    +------------------------+------------------------+ Episode         |
    | Scan timeline: phase band, force, torque (shared time axis)      | ROS 2 nodes |
    +-------------------------------------------------+-----------------+
The scene (where the probe is, what it sees) is on top; how the contact developed over time is
one strip underneath; secondary readouts sit in the right rail. Panes are separated by 1 px of
the divider colour showing through the grid spacing.
"""
from __future__ import annotations

import time

from PySide6 import QtCore, QtWidgets

from . import theme
from .sources.base import DataSource
from .sources.mock import PHANTOM
from .widgets.header import DashboardHeader
from .widgets.panels import (EpisodeInfoPanel, QualityPanel, RecordingStatusPanel, RobotStatePanel,
                             ROSNodeHealthPanel, UltrasoundPanel, WristCamerasPanel)
from .widgets.timeline import TimelinePanel

WIDE_MIN_PX = 1400      # below this width the panels rearrange into two columns


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, source: DataSource, cfg: dict, robot_view='auto'):
        super().__init__()
        self.source = source
        self.cfg = cfg
        self.setWindowTitle(f'{cfg.get("title", "UR7e Robotic Ultrasound")} — monitor')
        central = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(1)
        self.header = DashboardHeader(cfg.get('title', ''), cfg.get('subtitle', ''))
        outer.addWidget(self.header)

        self.robot = RobotStatePanel(phantom=PHANTOM if source.is_mock else None, view_mode=robot_view,
                                     trajectory=cfg.get('trajectory') or {})
        self.us = UltrasoundPanel()
        self.timeline = TimelinePanel(float(cfg.get('force_window_s', 30)))
        self.cams = WristCamerasPanel()
        self.quality = QualityPanel()
        self.episode = EpisodeInfoPanel()
        # Recording panel is off by default; show_recording_panel: true adds it to the rail.
        self.rec = RecordingStatusPanel() if cfg.get('show_recording_panel', False) else None
        self.nodes = ROSNodeHealthPanel()

        # right rail: secondary readouts, stacked (wide) or two columns (narrow)
        self.rail = QtWidgets.QWidget()
        self.rail.setObjectName('grid')
        self.rail.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.rail_grid = QtWidgets.QGridLayout(self.rail)
        self.rail_grid.setContentsMargins(0, 0, 0, 0)
        self.rail_grid.setSpacing(1)
        self.rail_items = [(self.cams, 4), (self.quality, 4), (self.episode, 5)]
        if self.rec is not None:
            self.rail_items.append((self.rec, 4))
        self.rail_items.append((self.nodes, 5))

        self.grid_host = QtWidgets.QWidget()
        self.grid_host.setObjectName('grid')
        self.grid_host.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.grid = QtWidgets.QGridLayout(self.grid_host)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(1)
        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scroll.setWidget(self.grid_host)
        # Demo only: while a real collection session runs, the panels are swapped for a paused page.
        self.stack = QtWidgets.QStackedWidget()
        self.stack.addWidget(self.scroll)
        self.stack.addWidget(self._paused_page())
        outer.addWidget(self.stack, 1)
        self.setCentralWidget(central)
        self._mode = None

        self._status_bar()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(int(1000 / float(cfg.get('refresh_hz', 30))))
        self.refresh_ms = 0.0
        self._ticks = []

    # ------------------------------------------------------------ layout
    def _arrange_rail(self, columns):
        for w, _ in self.rail_items:
            self.rail_grid.removeWidget(w)
        for i in range(8):
            self.rail_grid.setRowStretch(i, 0)
            self.rail_grid.setColumnStretch(i, 0)
        for k, (w, stretch) in enumerate(self.rail_items):
            r, c = (k, 0) if columns == 1 else divmod(k, 2)
            self.rail_grid.addWidget(w, r, c)
            if columns == 1:
                self.rail_grid.setRowStretch(r, stretch)
        if columns == 2:
            self.rail_grid.setColumnStretch(0, 1)
            self.rail_grid.setColumnStretch(1, 1)

    def _arrange(self, mode):
        if mode == self._mode:
            return
        self._mode = mode
        for w in (self.robot, self.us, self.timeline, self.rail):
            self.grid.removeWidget(w)
        for i in range(6):
            self.grid.setRowStretch(i, 0)
            self.grid.setColumnStretch(i, 0)
            self.grid.setRowMinimumHeight(i, 0)
        if mode == 'wide':
            place = {self.robot: (0, 0, 1, 1), self.us: (0, 1, 1, 1), self.timeline: (1, 0, 1, 2),
                     self.rail: (0, 2, 2, 1)}
            for c, st in enumerate((5, 5, 3)):
                self.grid.setColumnStretch(c, st)
            for r, st in enumerate((2, 1)):
                self.grid.setRowStretch(r, st)
            self.grid.setRowMinimumHeight(1, 300)
            self.grid_host.setMinimumHeight(0)
            self._arrange_rail(1)
        else:
            place = {self.robot: (0, 0, 1, 1), self.us: (0, 1, 1, 1), self.timeline: (1, 0, 1, 2),
                     self.rail: (2, 0, 1, 2)}
            heights = (440, 320, 3 * 240)
            for c in range(2):
                self.grid.setColumnStretch(c, 1)
            for r, h in enumerate(heights):
                self.grid.setRowMinimumHeight(r, h)
                self.grid.setRowStretch(r, 1)
            self.grid_host.setMinimumHeight(sum(heights) + len(heights) - 1)
            self._arrange_rail(2)
        for w, pos in place.items():
            self.grid.addWidget(w, *pos)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._arrange('wide' if self.width() >= WIDE_MIN_PX else 'narrow')

    def showEvent(self, e):
        super().showEvent(e)
        self._arrange('wide' if self.width() >= WIDE_MIN_PX else 'narrow')

    def _paused_page(self):
        page = QtWidgets.QFrame()
        page.setObjectName('panel')
        page.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        lay = QtWidgets.QVBoxLayout(page)
        lay.addStretch(1)
        head = QtWidgets.QLabel('Demo GUI paused: data collection in progress')
        head.setObjectName('title')
        head.setAlignment(QtCore.Qt.AlignCenter)
        self.paused_lbl = QtWidgets.QLabel()
        self.paused_lbl.setAlignment(QtCore.Qt.AlignCenter)
        self.paused_lbl.setWordWrap(True)
        hint = QtWidgets.QLabel('This screen is for demonstrations only. During collection use the operator console '
                                '(ros2 run usrig_dashboard operator_console). The demo view comes back by itself '
                                'when the session ends; nothing is subscribed meanwhile.')
        hint.setObjectName('small')
        hint.setAlignment(QtCore.Qt.AlignCenter)
        hint.setWordWrap(True)
        for w in (head, self.paused_lbl, hint):
            lay.addWidget(w)
        lay.addStretch(1)
        return page

    # ------------------------------------------------------------ status bar
    def _status_bar(self):
        sb = QtWidgets.QStatusBar()
        sb.setSizeGripEnabled(False)
        self.setStatusBar(sb)
        self.src_lbl = QtWidgets.QLabel()
        self.src_lbl.setContentsMargins(U_PAD, 0, 0, 0)
        sb.addWidget(self.src_lbl)
        if self.source.is_mock and self.source.scenarios:
            # Switches the *mock* data only; nothing is sent anywhere.
            lab = QtWidgets.QLabel('Scenario')
            self.combo = QtWidgets.QComboBox()
            self.combo.setFocusPolicy(QtCore.Qt.NoFocus)   # mouse only: evdev teleop reads keys regardless of focus
            self.combo.addItems(self.source.scenarios)
            self.combo.setCurrentText(getattr(self.source, 'scenario', self.source.scenarios[0]))
            self.combo.currentTextChanged.connect(self.source.set_scenario)
            sb.addPermanentWidget(lab)
            sb.addPermanentWidget(self.combo)

    # ------------------------------------------------------------ refresh
    def refresh(self):
        t0 = time.perf_counter()
        d = self.source.snapshot()
        self.header.update_state(d)
        if d.paused_reason:
            self.stack.setCurrentIndex(1)
            self.paused_lbl.setText(d.paused_reason)
            self.src_lbl.setText('Paused: a real collection session is running. Nothing is subscribed.')
            return
        self.stack.setCurrentIndex(0)
        self.robot.update_state(d)
        self.us.update_state(d)
        self.timeline.update_state(d)
        self.cams.update_state(d)
        self.quality.update_state(d)
        self.episode.update_state(d)
        if self.rec is not None:
            self.rec.update_state(d)
        self.nodes.update_state(d)
        now = time.perf_counter()
        self.refresh_ms = 0.9 * self.refresh_ms + 0.1 * (now - t0) * 1000
        self._ticks = [t for t in self._ticks if now - t < 2.0] + [now]
        fps = (len(self._ticks) - 1) / max(self._ticks[-1] - self._ticks[0], 1e-6) if len(self._ticks) > 1 else 0
        self.src_lbl.setText('Mock data, no hardware connected. Read-only monitor.' if d.is_mock
                             else 'Connected to ROS 2. Read-only monitor: nothing is sent to the robot.')
        self.src_lbl.setToolTip(f'display {fps:.0f} fps, update {self.refresh_ms:.0f} ms')


U_PAD = theme.UNIT + 4


def apply_theme(app):
    theme.load_fonts()
    app.setStyle('Fusion')
    app.setFont(theme.app_font())
    app.setStyleSheet(theme.STYLESHEET)
