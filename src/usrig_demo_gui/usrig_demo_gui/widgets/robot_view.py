"""RobotView: a small QPainter 3D view of the UR7e driven by joint angles.

Replaceable component: anything that offers set_state(joints_rad, connected, mode, contact) and
clear_trajectory() can take its place (e.g. an RViz/URDF view later). The arm is drawn as capsules between the DH frame origins, which
is the kinematic skeleton, not the real link shapes. Drag to orbit, wheel to zoom,
double-click to reset the view. No OpenGL needed.
"""
from __future__ import annotations

import math
import time

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

from .. import kinematics as kin
from .. import theme
from ..frames import ToolFrames
from ..trajectory import TrajectoryMap, map_label

LINK_R = [0.046, 0.036, 0.031, 0.028, 0.027, 0.027]     # radius of the segment ending at frame i+1
JOINT_R = [0.050, 0.044, 0.038, 0.032, 0.032, 0.030]


class RobotView(QtWidgets.QWidget):
    DEFAULT = dict(az=-22.0, el=26.0, dist=1.15, target=(0.30, 0.09, 0.26))   # same view as the 3D mesh view

    def __init__(self, phantom=None, trajectory=None, parent=None):
        super().__init__(parent)
        self.setMinimumSize(260, 220)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.phantom = phantom          # dict(center=(x, y), size=(sx, sy, h)) or None
        self.q = kin.Q_SEED.copy()
        self.connected = False
        self.has_state = False
        self.mode = None
        self.traj = TrajectoryMap(**(trajectory or {}))
        self.camera_T = None
        self._drag = None
        self.reset_view()

    def reset_view(self):
        self.az, self.el, self.dist = self.DEFAULT['az'], self.DEFAULT['el'], self.DEFAULT['dist']
        self.target = np.array(self.DEFAULT['target'])
        self.update()

    def set_state(self, joints_rad, connected, mode=None, frames=None, contact=None, phase=None):
        frames = frames or ToolFrames()
        self.connected = connected
        self.mode = mode
        if joints_rad is not None:
            self.q = np.asarray(joints_rad, float)
            self.has_state = True
        self.camera_T = frames.camera
        if connected:
            self.traj.update(time.monotonic(), frames, contact, phase)
        self.update()

    def clear_trajectory(self):
        self.traj.clear()
        self.update()

    # ------------------------------------------------------------ camera
    def _basis(self):
        a, e = math.radians(self.az), math.radians(self.el)
        eye = self.target + self.dist * np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
        f = self.target - eye
        f /= np.linalg.norm(f)
        r = np.cross(f, [0, 0, 1.0])
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        return eye, f, r, u

    def _project(self, pts, cam):
        eye, f, r, u = cam
        v = np.atleast_2d(pts) - eye
        z = v @ f
        z = np.maximum(z, 0.05)
        foc = 1.25 * min(self.width(), self.height() * 1.25)
        sx = self.width() / 2 + foc * (v @ r) / z
        sy = self.height() / 2 - foc * (v @ u) / z
        return sx, sy, z, foc

    # ------------------------------------------------------------ mouse
    def mousePressEvent(self, e):
        self._drag = e.position()

    def mouseMoveEvent(self, e):
        if self._drag is None:
            return
        d = e.position() - self._drag
        self._drag = e.position()
        self.az -= d.x() * 0.4
        self.el = min(max(self.el + d.y() * 0.3, 3.0), 85.0)
        self.update()

    def mouseReleaseEvent(self, _):
        self._drag = None

    def mouseDoubleClickEvent(self, _):
        self.reset_view()

    def wheelEvent(self, e):
        self.dist = min(max(self.dist * 0.9 ** (e.angleDelta().y() / 120), 0.6), 4.0)
        self.update()

    # ------------------------------------------------------------ drawing
    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor(theme.VIEW_3D))
        cam = self._basis()
        live = self.connected and self.has_state
        self._grid(p, cam)
        if self.phantom:
            self._box(p, cam, live)
        if self.has_state:
            self._arm(p, cam, live)
            self._trajectory(p, cam)
        self._gizmo(p, cam)

    def _line(self, p, cam, a, b, pen):
        sx, sy, z, _ = self._project(np.array([a, b]), cam)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(sx[0], sy[0]), QtCore.QPointF(sx[1], sy[1]))

    def _grid(self, p, cam):
        pen = QtGui.QPen(QtGui.QColor(theme.GRID_2D), 1)
        for x in np.arange(-0.4, 0.91, 0.1):
            self._line(p, cam, (x, -0.5, 0), (x, 0.6, 0), pen)
        for y in np.arange(-0.5, 0.61, 0.1):
            self._line(p, cam, (-0.4, y, 0), (0.9, y, 0), pen)

    def _box(self, p, cam, live):
        cx, cy = self.phantom['center']
        sx_, sy_, h = self.phantom['size']
        x0, x1, y0, y1 = cx - sx_ / 2, cx + sx_ / 2, cy - sy_ / 2, cy + sy_ / 2
        c = np.array([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0],
                      [x0, y0, h], [x1, y0, h], [x1, y1, h], [x0, y1, h]])
        faces = [([4, 5, 6, 7], (0, 0, 1)), ([0, 1, 5, 4], (0, -1, 0)), ([1, 2, 6, 5], (1, 0, 0)),
                 ([2, 3, 7, 6], (0, 1, 0)), ([3, 0, 4, 7], (-1, 0, 0))]
        eye = cam[0]
        sx, sy, z, _ = self._project(c, cam)
        base = QtGui.QColor(201, 161, 132) if live else QtGui.QColor(214, 205, 198)
        light = np.array([0.3, -0.5, 0.8])
        light /= np.linalg.norm(light)
        drawn = []
        for idx, n in faces:
            centre = c[idx].mean(axis=0)
            if np.dot(n, eye - centre) <= 0:
                continue
            k = 0.55 + 0.45 * max(np.dot(n, light), 0)
            drawn.append((z[idx].mean(), idx, base.darker(int(100 / k))))
        for _, idx, col in sorted(drawn, key=lambda d: -d[0]):
            poly = QtGui.QPolygonF([QtCore.QPointF(sx[i], sy[i]) for i in idx])
            p.setPen(QtGui.QPen(col.darker(130), 1))
            p.setBrush(col)
            p.drawPolygon(poly)
        p.setBrush(QtCore.Qt.NoBrush)

    def _polyline(self, p, cam, pts, color, width):
        if len(pts) < 2:
            return
        sx, sy, _, _ = self._project(pts, cam)
        p.setPen(QtGui.QPen(QtGui.QColor(color), width))
        p.drawPolyline(QtGui.QPolygonF([QtCore.QPointF(a, b) for a, b in zip(sx, sy)]))

    def _trajectory(self, p, cam):
        """Flange (purple), wrist camera (teal), probe tip (dark blue in contact, light blue in air),
        markers at the rest position and at phase changes."""
        tr = self.traj
        self._polyline(p, cam, tr.flange(), theme.TRAJ_FLANGE, 2.5)
        self._polyline(p, cam, tr.camera(), theme.TRAJ_CAMERA, 2.5)
        tip = tr.tip()
        if len(tip) > 1:
            sx, sy, _, _ = self._project(tip, cam)
            air = tr.tip_in_air()
            pens = {False: QtGui.QPen(QtGui.QColor(theme.TRAJ_TIP), 3), True: QtGui.QPen(QtGui.QColor(theme.TRAJ_TIP_AIR), 3)}
            for i in range(1, len(sx)):
                p.setPen(pens[bool(air[i])])
                p.drawLine(QtCore.QPointF(sx[i - 1], sy[i - 1]), QtCore.QPointF(sx[i], sy[i]))
        if self.camera_T is not None:                      # camera position + viewing direction
            c = self.camera_T[:3, 3]
            self._line(p, cam, c, c + self.camera_T[:3, 2] * 0.04, QtGui.QPen(QtGui.QColor(theme.TRAJ_CAMERA), 2))
            sx, sy, _, _ = self._project(c[None, :], cam)
            p.setBrush(QtGui.QColor('#4a4e55'))
            p.setPen(QtCore.Qt.NoPen)
            p.drawRect(QtCore.QRectF(sx[0] - 5, sy[0] - 3, 10, 6))
            p.setBrush(QtCore.Qt.NoBrush)
        if tr.markers:
            sx, sy, _, _ = self._project(np.array([m.pos for m in tr.markers]), cam)
            p.setFont(theme.small_font())
            fm = p.fontMetrics()
            mid = float(np.median(sx))                      # outer side of each dot, as in the 3D view
            for i, (x, y, m) in enumerate(zip(sx, sy, tr.markers)):
                p.setPen(QtCore.Qt.NoPen)
                p.setBrush(QtGui.QColor(theme.TEXT))
                p.drawEllipse(QtCore.QPointF(x, y), 3, 3)
                p.setBrush(QtCore.Qt.NoBrush)
                p.setPen(QtGui.QColor(theme.TEXT))
                text = map_label(m.label)
                w = fm.horizontalAdvance(text)
                p.drawText(QtCore.QPointF(x + 7 if x >= mid else x - 7 - w, y + 4), text)

    def _arm(self, p, cam, live):
        frames = kin.fk_frames(self.q)
        pts = np.array([T[:3, 3] for T in frames])          # base, F1..F6, tip
        sx, sy, z, foc = self._project(pts, cam)
        link = QtGui.QColor('#9a9ea4') if live else QtGui.QColor('#c4c7cc')
        joint = QtGui.QColor('#5b8fc2') if live else QtGui.QColor('#c4c7cc')
        probe = QtGui.QColor('#6b6f75') if live else QtGui.QColor('#c4c7cc')
        items = []
        for i in range(6):
            items.append(((z[i] + z[i + 1]) / 2, 'seg', i, i + 1, LINK_R[i], link))
        items.append(((z[6] + z[7]) / 2, 'seg', 6, 7, 0.020, probe))
        for i in range(1, 7):
            items.append((z[i] - 0.02, 'joint', i, None, JOINT_R[i - 1], joint))
        for d, kind, a, b, r, col in sorted(items, key=lambda it: -it[0]):
            if kind == 'seg':
                w = max(2.0, 2 * r * foc / ((z[a] + z[b]) / 2))
                pen = QtGui.QPen(col, w, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
                p.setPen(QtGui.QPen(col.darker(160), w + 2, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
                p.drawLine(QtCore.QPointF(sx[a], sy[a]), QtCore.QPointF(sx[b], sy[b]))
                p.setPen(pen)
                p.drawLine(QtCore.QPointF(sx[a], sy[a]), QtCore.QPointF(sx[b], sy[b]))
            else:
                rr = max(3.0, r * foc / z[a])
                p.setPen(QtGui.QPen(col.darker(150), 1))
                p.setBrush(col)
                p.drawEllipse(QtCore.QPointF(sx[a], sy[a]), rr, rr)
        p.setBrush(QtCore.Qt.NoBrush)
        # TCP triad at the probe tip
        if live:
            T = frames[-1]
            for k, col in enumerate(theme.AXIS_COLORS):
                self._line(p, cam, T[:3, 3], T[:3, 3] + T[:3, k] * 0.05, QtGui.QPen(QtGui.QColor(col), 2))

    def _gizmo(self, p, cam):
        """Base-frame orientation, drawn in the bottom-left corner."""
        eye, f, r, u = cam
        o = QtCore.QPointF(34, self.height() - 34)
        p.setFont(theme.small_font())
        for k, (name, col) in enumerate(zip('XYZ', theme.AXIS_COLORS)):
            d = np.zeros(3)
            d[k] = 1
            end = QtCore.QPointF(o.x() + 22 * d @ r, o.y() - 22 * d @ u)
            p.setPen(QtGui.QPen(QtGui.QColor(col), 2))
            p.drawLine(o, end)
            p.drawText(QtCore.QPointF(end.x() + 3 * (1 if d @ r >= 0 else -3), end.y() - 2), name)
