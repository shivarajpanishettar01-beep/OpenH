"""TimelinePanel: one time axis for the scan.

  Phase    | Approach | Contact | View acq. | Sweep ...      <- task phases as a band
  Force    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   Fx Fy Fz now
  Torque   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   Tx Ty Tz now
           -30 s                                      0 s

The x axis is seconds relative to now. Phases and events are remembered by the panel with their
arrival time, so they scroll with the force data. Events are drawn as ticks on the band and as
faint vertical lines through the force and torque plots, so a force change can be read against
the event that caused it. The plots only know set_data(); any source can feed them.
"""
from __future__ import annotations

import time
from collections import deque

import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

from .. import theme
from ..models import FT_LABELS, FT_UNITS, DashboardState, Health, TaskPhase
from .common import DASH, U, Panel, Swatch, style

LINE_W = 1.6
AXIS_W = 64            # same left-axis width for all three plots, so their x axes line up
SHORT = {TaskPhase.APPROACH: 'Approach', TaskPhase.CONTACT: 'Contact', TaskPhase.VIEW_ACQUISITION: 'View acq.',
         TaskPhase.SWEEP: 'Sweep', TaskPhase.REACQUISITION: 'Reacq.', TaskPhase.RECOVERY: 'Recovery'}
EVENT_COLOR = {Health.OK: theme.GREEN, Health.WARN: theme.ORANGE, Health.DOWN: theme.RED, Health.UNKNOWN: theme.MUTED}


def _style(pw, ylabel, window_s, show_x):
    pw.setBackground(theme.PANEL)
    pi = pw.getPlotItem()
    pi.setMenuEnabled(False)
    pi.hideButtons()
    pi.setMouseEnabled(x=False, y=False)
    pi.setXRange(-window_s, 0, padding=0)
    pi.getViewBox().setDefaultPadding(0.1)
    font = theme.app_font(theme.SMALL_PX)
    for ax in ('left', 'bottom'):
        a = pi.getAxis(ax)
        a.setPen(pg.mkPen(theme.FAINT))
        a.setTextPen(pg.mkPen(theme.MUTED))
        a.setStyle(tickLength=-4, tickFont=font, tickTextOffset=6)
    left = pi.getAxis('left')
    left.setWidth(AXIS_W)
    left.enableAutoSIPrefix(False)
    left.setLabel(f'<span style="color:{theme.MUTED}; font-size:{theme.FONT_PX}px">{ylabel}</span>')
    bottom = pi.getAxis('bottom')
    if show_x:
        bottom.setLabel(f'<span style="color:{theme.MUTED}; font-size:{theme.FONT_PX}px">time (s)</span>')
    else:
        bottom.setStyle(showValues=False)
        bottom.setHeight(4)
    pw.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Ignored)
    return pi


class TimelinePanel(Panel):
    MAX_EVENTS = 40

    def __init__(self, window_s=30.0):
        super().__init__('Scan timeline')
        self.window_s = float(window_s)
        self.phase_hist = deque()          # (t_mono, phase or None) at each change
        self.events = {}                   # (episode, t_s, text) -> (t_mono, level)
        self._ep = None

        # --- plots
        self.band = pg.PlotWidget()
        self.pf = pg.PlotWidget()
        self.pt = pg.PlotWidget()
        pib = _style(self.band, 'Phase', self.window_s, False)
        pif = _style(self.pf, 'Force (N)', self.window_s, False)
        pit = _style(self.pt, 'Torque (Nm)', self.window_s, True)
        pib.showGrid(x=False, y=False)
        pib.hideAxis('bottom')
        for pi in (pif, pit):
            pi.showGrid(x=True, y=True, alpha=0.18)
            pi.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(theme.FAINT, width=1, style=QtCore.Qt.DashLine)),
                       ignoreBounds=True)
        bl = pib.getAxis('left')
        bl.setLabel('')
        bl.setTicks([[(0.5, 'Phase')]])            # horizontal row label instead of a rotated axis title
        bl.setStyle(tickLength=0)
        bl.setPen(pg.mkPen(None))
        pib.setYRange(0, 1, padding=0)
        pib.getViewBox().setLimits(yMin=0, yMax=1)
        self.band.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.band.setFixedHeight(34)
        pif.setXLink(pib)
        pit.setXLink(pib)
        self.pif, self.pit, self.pib = pif, pit, pib
        self.curves = []
        for i in range(6):
            pi = pif if i < 3 else pit
            c = pi.plot([], [], pen=pg.mkPen(theme.AXIS_COLORS[i % 3], width=LINE_W), antialias=True)
            c.setDownsampling(auto=True, method='peak')
            c.setClipToView(True)
            self.curves.append(c)
        self._rects, self._labels, self._ticks, self._evlines = [], [], [], []
        for pi in (pif, pit):                      # no data yet: a plain range around zero
            pi.setYRange(-1, 1, padding=0)
        self._empty = True

        # --- readings beside the plots
        g = QtWidgets.QGridLayout()
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(2 * U)
        g.setVerticalSpacing(U // 2)
        g.addWidget(self.band, 0, 0)
        self.phase_now = QtWidgets.QLabel(DASH)
        self.phase_now.setFixedWidth(150)
        g.addWidget(self.phase_now, 0, 1)
        g.addWidget(self.pf, 1, 0)
        g.addWidget(self.pt, 2, 0)
        fw, fvals = self._readings(FT_LABELS[:3], 0)
        tw, tvals = self._readings(FT_LABELS[3:], 3)
        g.addWidget(fw, 1, 1)
        g.addWidget(tw, 2, 1)
        g.setColumnStretch(0, 1)
        g.setRowMinimumHeight(0, 34)
        g.setRowStretch(1, 1)
        g.setRowStretch(2, 1)
        self.body.addLayout(g, 1)
        self.vals = fvals + tvals

    @staticmethod
    def _readings(labels, start):
        w = QtWidgets.QWidget()
        w.setFixedWidth(150)
        g = QtWidgets.QGridLayout(w)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(U)
        g.setVerticalSpacing(U // 2)
        g.setRowStretch(0, 1)
        vals = []
        for k, lab in enumerate(labels):
            name = QtWidgets.QLabel(lab)
            name.setObjectName('muted')
            v = QtWidgets.QLabel(DASH)
            v.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            v.setFont(theme.app_font(theme.FONT_PX + 2, QtGui.QFont.Medium))
            g.addWidget(Swatch(theme.AXIS_COLORS[(start + k) % 3]), k + 1, 0)
            g.addWidget(name, k + 1, 1)
            g.addWidget(v, k + 1, 2)
            vals.append(v)
        g.setColumnStretch(2, 1)
        g.setRowStretch(len(labels) + 1, 1)
        return w, vals

    # ---------------------------------------------------------------- history
    def _remember(self, d: DashboardState, now):
        ph = d.task.phase
        if not self.phase_hist or self.phase_hist[-1][1] != ph:
            self.phase_hist.append((now, ph))
        while len(self.phase_hist) > 1 and now - self.phase_hist[1][0] > self.window_s:
            self.phase_hist.popleft()
        ep = d.recording.episode_id
        el = d.recording.elapsed_s
        if el is not None:                       # event times are episode-relative: anchor them now
            for e in d.task.events:
                key = (ep, round(e.t_s, 2), e.text)
                if key not in self.events:
                    self.events[key] = (now - (el - e.t_s), e.level, e.text)
        for k in [k for k, v in self.events.items() if now - v[0] > self.window_s]:
            del self.events[k]

    # ---------------------------------------------------------------- drawing
    def _pool(self, pool, make, n, plot):
        while len(pool) < n:
            it = make()
            plot.addItem(it, ignoreBounds=True)
            pool.append(it)
        for i, it in enumerate(pool):
            it.setVisible(i < n)

    def _draw_band(self, now):
        segs = []
        hist = list(self.phase_hist)
        for i, (t0, ph) in enumerate(hist):
            t1 = hist[i + 1][0] if i + 1 < len(hist) else now
            if ph is not None and t1 - now > -self.window_s:
                segs.append((max(t0 - now, -self.window_s), t1 - now, ph, i == len(hist) - 1))
        self._pool(self._rects, lambda: QtWidgets.QGraphicsRectItem(), len(segs), self.pib)
        self._pool(self._labels, lambda: pg.TextItem(anchor=(0, 0.5)), len(segs), self.pib)
        px_per_s = self.pib.getViewBox().width() / self.window_s if self.window_s else 0
        for (x0, x1, ph, cur), rect, lab in zip(segs, self._rects, self._labels):
            if ph == TaskPhase.RECOVERY:
                fill, ink = (theme.ORANGE, '#ffffff') if cur else ('#f2dcc6', theme.TEXT)
            else:
                fill, ink = (theme.ACCENT, '#ffffff') if cur else (theme.ACCENT_DIM, theme.TEXT)
            rect.setRect(QtCore.QRectF(x0, 0.08, max(x1 - x0 - 0.04, 0.01), 0.84))
            rect.setBrush(QtGui.QColor(fill))
            rect.setPen(QtGui.QPen(QtCore.Qt.NoPen))
            text = SHORT.get(ph, ph.value)
            lab.setText(text, color=ink)
            lab.setFont(theme.app_font(theme.SMALL_PX, QtGui.QFont.Medium if cur else None))
            lab.setPos(x0 + 4 / max(px_per_s, 1e-6), 0.5)
            fits = (x1 - x0) * px_per_s > QtGui.QFontMetrics(lab.textItem.font()).horizontalAdvance(text) + 8
            lab.setVisible(fits)

    def _draw_events(self, now):
        evs = sorted(self.events.values(), key=lambda v: v[0])[-self.MAX_EVENTS:]
        self._pool(self._ticks, lambda: pg.ScatterPlotItem(symbol='t', size=8, pxMode=True), len(evs), self.pib)
        n_lines = len(evs) * 2
        while len(self._evlines) < n_lines:
            for pi in (self.pif, self.pit):
                ln = pg.InfiniteLine(angle=90, movable=False)
                pi.addItem(ln, ignoreBounds=True)
                self._evlines.append(ln)
        for i, ln in enumerate(self._evlines):
            ln.setVisible(i < n_lines)
        for k, (t, level, text) in enumerate(evs):
            col = QtGui.QColor(EVENT_COLOR.get(level, theme.MUTED))
            x = t - now
            self._ticks[k].setData([x], [0.04], brush=col, pen=pg.mkPen(col))
            self._ticks[k].setToolTip(text)
            c2 = QtGui.QColor(col)
            c2.setAlpha(110)
            for ln in self._evlines[2 * k:2 * k + 2]:
                ln.setPos(x)
                ln.setPen(pg.mkPen(c2, width=1, style=QtCore.Qt.DashLine))

    def update_state(self, d: DashboardState):
        now = time.monotonic()
        self._remember(d, now)
        ft = d.ft
        if ft.t is None or ft.values is None or len(ft.t) == 0:
            for c in self.curves:
                c.setData([], [])
            if not self._empty:
                for pi in (self.pif, self.pit):
                    pi.setYRange(-1, 1, padding=0)
                self._empty = True
        else:
            if self._empty:
                for pi in (self.pif, self.pit):
                    pi.enableAutoRange(axis='y')
                self._empty = False
            m = ft.t >= -self.window_s
            for i, c in enumerate(self.curves):
                c.setData(ft.t[m], ft.values[m, i])
        last = ft.latest()
        for i, v in enumerate(self.vals):
            v.setText(DASH if last is None else
                      (f'{last[i]:+.2f} {FT_UNITS[i]}' if i < 3 else f'{last[i]:+.3f} {FT_UNITS[i]}'))
        self._draw_band(now)
        self._draw_events(now)
        ph = d.task.phase
        self.phase_now.setText(ph.value if ph else 'No episode')
        style(self.phase_now, f'color: {theme.ORANGE if ph == TaskPhase.RECOVERY else (theme.ACCENT if ph else theme.MUTED)};'
                              f' font-weight: 600;')
        r = d.recording
        if r.episode_id and r.elapsed_s is not None and r.total_s:
            self.set_info(f'Episode {r.episode_id}, {r.elapsed_s:.1f} of {r.total_s:.0f} s')
        elif r.episode_id:
            self.set_info(f'Episode {r.episode_id}')
        else:
            self.set_info(f'Last {self.window_s:.0f} s')
