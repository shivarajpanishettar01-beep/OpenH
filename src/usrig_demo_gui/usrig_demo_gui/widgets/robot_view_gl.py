"""RobotViewGL: the UR7e drawn with the Universal Robots meshes (OpenGL through pyqtgraph).

Same interface as the simple RobotView: set_state(joints_rad, connected, mode, frames, contact, phase),
clear_trajectory(), grab_image(). Meshes come from ur_description (BSD-3-Clause, see
assets/LICENSE.ur_description), converted by tools/build_ur_meshes.py.
Left-drag orbits, wheel zooms, middle-drag pans, double-click resets.

The trajectory map (trajectory.py) is drawn in the same scene as the arm: flange (end effector),
probe tip (dark in contact, light in air) and wrist camera paths from the last rest position,
with markers at phase changes. The wrist camera body is drawn at its calibrated pose.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pyqtgraph.opengl as gl
from OpenGL import GL
from pyqtgraph.opengl import shaders
from pyqtgraph.opengl.GLViewWidget import GLViewMixin
from PySide6 import QtCore, QtGui, QtOpenGL, QtWidgets

from .. import kinematics as kin
from .. import theme
from ..frames import ToolFrames
from ..trajectory import TrajectoryMap, map_label

MESH_FILE = Path(__file__).resolve().parents[1] / 'assets' / 'ur7e_meshes.npz'
LINKS = ['base', 'shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3']


# Two-sided "headlight" shading: light comes from the camera, so whatever faces you is lit.
# (pyqtgraph's built-in 'shaded' lights from behind-left and leaves most of the arm black.)
HEADLIGHT = shaders.ShaderProgram('usrig_headlight', [
    shaders.VertexShader("""
        uniform mat4 u_mvp;
        uniform mat3 u_normal;
        attribute vec4 a_position;
        attribute vec3 a_normal;
        attribute vec4 a_color;
        varying vec4 v_color;
        varying vec3 v_normal;
        void main() {
            v_normal = normalize(u_normal * a_normal);
            v_color = a_color;
            gl_Position = u_mvp * a_position;
        }
    """),
    shaders.FragmentShader("""
        #ifdef GL_ES
        precision mediump float;
        #endif
        varying vec4 v_color;
        varying vec3 v_normal;
        void main() {
            vec3 n = normalize(v_normal);
            float d = abs(dot(n, normalize(vec3(0.25, 0.45, 1.0))));
            float s = pow(d, 24.0) * 0.18;
            vec3 rgb = v_color.rgb * (0.38 + 0.62 * d) + vec3(s);
            gl_FragColor = vec4(rgb, v_color.a);
        }
    """),
])


# Lines drawn over the scene (no depth test, normal alpha blending). pyqtgraph's default for lines
# is additive blending, which washes out on a light background; and the probe tip sits a few mm
# inside the phantom while in contact, so a depth-tested path would be hidden there.
OVERLAY = {GL.GL_DEPTH_TEST: False, GL.GL_BLEND: True, GL.GL_CULL_FACE: False,
           'glBlendFunc': (GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)}
TRANSLUCENT = {GL.GL_DEPTH_TEST: True, GL.GL_BLEND: True, GL.GL_CULL_FACE: False,
               'glBlendFunc': (GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)}


def _qmat(T):
    return QtGui.QMatrix4x4(*np.asarray(T, float).ravel().tolist())


def _box(x0, x1, y0, y1, z0, z1):
    v = np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)], np.float32)
    f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]], np.uint32)
    return v, f


def _merge(parts):
    V, F, C, n = [], [], [], 0
    for (v, f), col in parts:
        V.append(v)
        F.append(f + n)
        C.append(np.tile(np.asarray(col, np.float32), (len(v), 1)))
        n += len(v)
    return np.concatenate(V), np.concatenate(F), np.concatenate(C)


def _rgba(hex_, a=1.0):
    c = QtGui.QColor(hex_)
    return (c.redF(), c.greenF(), c.blueF(), a)


class _View(GLViewMixin, QtOpenGL.QOpenGLWindow):
    """pyqtgraph 3D view as a QOpenGLWindow. It is embedded with createWindowContainer instead of
    using GLViewWidget (a QOpenGLWidget), because a QOpenGLWidget switches the whole main window to
    OpenGL compositing: every label and image repaint then goes through the GPU path, which is very
    slow on machines with software OpenGL."""
    reset = QtCore.Signal()

    def __init__(self):
        super().__init__()
        fmt = QtGui.QSurfaceFormat()
        fmt.setDepthBufferSize(24)      # QOpenGLWindow has no depth buffer by default; without it
        fmt.setSamples(4)               # faces are drawn in painting order instead of by distance
        self.setFormat(fmt)

    def mouseDoubleClickEvent(self, e):
        self.reset.emit()

    def rect(self):
        """QWidget-style rect(); pyqtgraph's GLTextItem projects labels with view().rect()."""
        return QtCore.QRect(0, 0, self.width(), self.height())


class RobotViewGL(QtWidgets.QWidget):
    MAX_HZ = 20
    DEFAULT = dict(distance=0.8, elevation=26, azimuth=-22, center=(0.33, 0.10, 0.22))

    def __init__(self, phantom=None, trajectory=None, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self.view = _View()
        self.view.setBackgroundColor(QtGui.QColor(theme.VIEW_3D))
        self.view.reset.connect(self.reset_view)
        self.container = QtWidgets.QWidget.createWindowContainer(self.view, self)
        self.container.setMinimumSize(200, 160)
        lay.addWidget(self.container, 1)
        self.setMinimumSize(260, 220)

        grid = gl.GLGridItem(size=QtGui.QVector3D(1.4, 1.2, 1), color=theme.GRID_3D)
        grid.setSpacing(0.1, 0.1, 0.1)
        grid.translate(0.25, 0.05, 0)
        self.view.addItem(grid)
        for k, col in enumerate(theme.AXIS_COLORS):          # base frame axes
            p = np.zeros((2, 3))
            p[1, k] = 0.12
            self.view.addItem(gl.GLLinePlotItem(pos=p, color=_rgba(col), width=2, antialias=True,
                                                glOptions=TRANSLUCENT))

        data = np.load(MESH_FILE)
        self.links = {}
        for name in LINKS:
            md = gl.MeshData(vertexes=data[f'{name}_v'], faces=data[f'{name}_f'],
                             vertexColors=data[f'{name}_c'].astype(np.float32) / 255.0)
            md._vertexNormals = data[f'{name}_n']     # precomputed (pyqtgraph's own loop is slow)
            item = gl.GLMeshItem(meshdata=md, smooth=True, shader=HEADLIGHT, glOptions='opaque')
            self.view.addItem(item)
            self.links[name] = item

        # probe and holder on tool0 (z out of the flange); illustrative shape, 0.13 m to the tip
        L = kin.PROBE_LEN_M
        v, f, c = _merge([(_box(-0.035, 0.035, -0.032, 0.032, 0.0, 0.035), _rgba('#3c3f44')),
                          (_box(-0.016, 0.016, -0.026, 0.026, 0.035, L - 0.025), _rgba('#d9d9d9')),
                          (_box(-0.009, 0.009, -0.028, 0.028, L - 0.025, L), _rgba('#bfbfbf'))])
        self.probe = gl.GLMeshItem(meshdata=gl.MeshData(vertexes=v, faces=f, vertexColors=c),
                                   smooth=False, shader=HEADLIGHT, glOptions='opaque')
        self.view.addItem(self.probe)

        if phantom:
            cx, cy = phantom['center']
            sx, sy, h = phantom['size']
            v, f = _box(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, 0, h)
            # flat colour per face (top lightest) so the scanning surface reads clearly at any angle;
            # face order from _box: bottom, top, -y, +y, -x, +x (two triangles each)
            shades = ['#a98263', '#dcb898', '#bf9677', '#bf9677', '#b48b6c', '#b48b6c']
            fc = np.array([_rgba(c) for c in shades for _ in range(2)], np.float32)
            ph = gl.GLMeshItem(meshdata=gl.MeshData(vertexes=v, faces=f, faceColors=fc), smooth=False,
                               shader=None, glOptions='opaque')
            self.view.addItem(ph)

        # wrist camera body, drawn at the calibrated camera pose (D415 approx. 99 x 23 x 20 mm;
        # optical frame: z forward, x right, y down, so the body sits behind the optical centre)
        v, f, c = _merge([(_box(-0.0495, 0.0495, -0.0115, 0.0115, -0.020, 0.0), _rgba('#4a4e55')),
                          (_box(-0.006, 0.006, -0.006, 0.006, 0.0, 0.002), _rgba('#1f2328'))])
        self.camera_body = gl.GLMeshItem(meshdata=gl.MeshData(vertexes=v, faces=f, vertexColors=c),
                                         smooth=False, shader=HEADLIGHT, glOptions='opaque')
        self.camera_body.setVisible(False)
        self.view.addItem(self.camera_body)

        # trajectory map (added after the meshes so it is drawn on top)
        # (antialias off: smoothed wide lines come out dotted on some OpenGL drivers)
        self.traj = TrajectoryMap(**(trajectory or {}))
        self.path_items = {
            'flange': gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=_rgba(theme.TRAJ_FLANGE), width=2.5,
                                        antialias=False, glOptions=OVERLAY),
            'camera': gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=_rgba(theme.TRAJ_CAMERA), width=2.5,
                                        antialias=False, glOptions=OVERLAY),
            'tip': gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=_rgba(theme.TRAJ_TIP), width=3,
                                     antialias=False, glOptions=OVERLAY),
        }
        for it in self.path_items.values():
            it.setVisible(False)
            self.view.addItem(it)
        self.marker_item = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=_rgba(theme.TEXT), size=7, pxMode=True,
                                                glOptions=OVERLAY)
        self.marker_item.setVisible(False)
        self.view.addItem(self.marker_item)
        self.marker_labels = []
        self.tcp_axes = [gl.GLLinePlotItem(pos=np.zeros((2, 3)), color=_rgba(col), width=2, antialias=True,
                                           glOptions=OVERLAY) for col in theme.AXIS_COLORS]
        for it in self.tcp_axes:
            self.view.addItem(it)

        self.has_state = False
        self.reset_view()
        self._pose(kin.Q_SEED)
        for it in list(self.links.values()) + [self.probe] + self.tcp_axes:
            it.setVisible(False)
        # Redraw at most MAX_HZ and only when something changed: GUI refresh and 3D rendering are
        # decoupled, so a slow GPU (or software OpenGL) cannot starve the rest of the window.
        self._pending = None
        self._last_q = None
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._apply)
        self._timer.start(int(1000 / self.MAX_HZ))

    def reset_view(self):
        d = self.DEFAULT
        self.view.setCameraPosition(pos=QtGui.QVector3D(*d['center']), distance=d['distance'],
                                    elevation=d['elevation'], azimuth=d['azimuth'])

    def _pose(self, q):
        frames = kin.urdf_frames(q)
        for name, item in self.links.items():
            item.setTransform(_qmat(frames[name]))
        T = frames['tool0']
        self.probe.setTransform(_qmat(T))
        return T

    def set_state(self, joints_rad, connected, mode=None, frames=None, contact=None, phase=None):
        """joints_rad poses the drawn arm; frames (ToolFrames from the TCP topic + calibration) feed the map."""
        self._pending = (None if joints_rad is None else np.array(joints_rad, float), connected,
                         frames or ToolFrames(), contact, phase)

    def clear_trajectory(self):
        self.traj.clear()
        self._draw_trajectory()

    def _draw_trajectory(self):
        tr = self.traj
        for key, pts in (('flange', tr.flange()), ('camera', tr.camera()), ('tip', tr.tip())):
            it = self.path_items[key]
            it.setVisible(len(pts) > 1)
            if len(pts) > 1:
                if key == 'tip':
                    cols = np.where(tr.tip_in_air()[:, None], _rgba(theme.TRAJ_TIP_AIR), _rgba(theme.TRAJ_TIP))
                    it.setData(pos=pts, color=cols.astype(np.float32))
                else:
                    it.setData(pos=pts)
        mk = tr.markers
        self.marker_item.setVisible(bool(mk))
        if mk:
            self.marker_item.setData(pos=np.array([m.pos for m in mk]))
        while len(self.marker_labels) < len(mk):
            lab = gl.GLTextItem(color=QtGui.QColor(theme.TEXT), font=theme.small_font())
            lab.setGLOptions(OVERLAY)
            self.view.addItem(lab)
            self.marker_labels.append(lab)
        for i, lab in enumerate(self.marker_labels):
            if i < len(mk):
                text = map_label(mk[i].label)
                lab.setData(pos=mk[i].pos, text=f'  {text}  ' if text else '')
                lab.setVisible(True)
            else:
                lab.setVisible(False)
        self._place_labels()

    def _place_labels(self):
        """Put each label on the outer side of its dot (left half of the markers -> left, right half ->
        right, in screen x), so labels of nearby markers do not run into each other. Recomputed every
        tick, so it follows the camera when the view is rotated."""
        mk = self.traj.markers
        if not mk or not self.marker_labels:
            return
        try:
            proj = self.marker_labels[0].compute_projection()
        except Exception:
            return
        xs = [proj.map(QtGui.QVector3D(*map(float, m.pos))).x() for m in mk]
        mid = float(np.median(xs))
        for x, lab in zip(xs, self.marker_labels):
            side = QtCore.Qt.AlignRight if x < mid else QtCore.Qt.AlignLeft
            if lab.alignment != side | QtCore.Qt.AlignVCenter:
                lab.setData(alignment=side | QtCore.Qt.AlignVCenter)

    def _apply(self):
        self._place_labels()
        if self._pending is None:
            return
        joints_rad, connected, frames, contact, phase = self._pending
        self._pending = None
        if joints_rad is not None and (self._last_q is None or np.abs(joints_rad - self._last_q).max() > 1e-4):
            self._last_q = joints_rad
            T = self._pose(joints_rad)
            if frames.tip is None:                    # triad at the drawn probe tip when no tip frame
                T = T.copy()
                T[:3, 3] = T[:3, 3] + T[:3, 2] * kin.PROBE_LEN_M
            else:
                T = frames.tip
            for k, it in enumerate(self.tcp_axes):
                it.setData(pos=np.array([T[:3, 3], T[:3, 3] + T[:3, k] * 0.05]))
            self.has_state = True
        self.camera_body.setVisible(frames.camera is not None)
        if frames.camera is not None:
            self.camera_body.setTransform(_qmat(frames.camera))
        if connected and self.traj.update(time.monotonic(), frames, contact, phase):
            self._draw_trajectory()
        for it in list(self.links.values()) + [self.probe] + self.tcp_axes:   # never had a pose: draw no arm
            it.setVisible(self.has_state)

    def grab_image(self):
        """Framebuffer as a QImage (the native GL window is not included in QWidget.grab())."""
        return self.view.grabFramebuffer()
