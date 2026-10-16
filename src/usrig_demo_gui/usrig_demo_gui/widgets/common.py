"""Small building blocks shared by every panel, so they all look and align the same way."""
from __future__ import annotations

from typing import Optional

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

from .. import theme
from ..models import Health, StreamStatus

DASH = '—'
U = theme.UNIT


def fmt(v, f='{:.2f}', unit=''):
    """Format an optional number. None -> em dash (unavailable), never 0."""
    if v is None:
        return DASH
    s = f.format(v)
    return f'{s} {unit}' if unit else s


def style(w, css):
    """setStyleSheet only when it changes (re-styling every frame is expensive in Qt)."""
    if w.property('_css') != css:
        w.setStyleSheet(css)
        w.setProperty('_css', css)


def mono_font(size_px=theme.SMALL_PX):
    return theme.mono_font(size_px)


def vline():
    f = QtWidgets.QFrame()
    f.setObjectName('vline')
    f.setFixedWidth(1)
    return f


def hline():
    f = QtWidgets.QFrame()
    f.setObjectName('hline')
    f.setFixedHeight(1)
    return f


class Panel(QtWidgets.QFrame):
    """Panel with a title bar (like a docked tool window) and a body.

    The title bar has its own background and a hairline under it; optional status text sits on
    the right of the title (.info). Content goes into .body.
    """

    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setObjectName('panel')
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        bar = QtWidgets.QFrame()
        bar.setObjectName('titlebar')
        bar.setFixedHeight(theme.TITLE_H)
        head = QtWidgets.QHBoxLayout(bar)
        head.setContentsMargins(U + 4, 0, U + 4, 0)
        self.title = QtWidgets.QLabel(title)
        self.title.setObjectName('panelTitle')
        self.info = QtWidgets.QLabel('')
        self.info.setObjectName('small')
        self.info.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.info.setMinimumWidth(120)
        head.addWidget(self.title)
        head.addStretch(1)
        head.addWidget(self.info)
        lay.addWidget(bar)
        self.body = QtWidgets.QVBoxLayout()
        self.body.setContentsMargins(U + 4, U + 2, U + 4, U + 2)
        self.body.setSpacing(U)
        lay.addLayout(self.body, 1)

    def set_info(self, text, color=None):
        self.info.setText(text)
        style(self.info, f'color: {color or theme.MUTED}; font-size: {theme.SMALL_PX}px;')


class Dot(QtWidgets.QWidget):
    def __init__(self, size=8, parent=None):
        super().__init__(parent)
        self._c = QtGui.QColor(theme.GREY)
        self._hollow = False
        self.setFixedSize(size, size)

    def set_color(self, color, hollow=False):
        c = QtGui.QColor(color)
        if c != self._c or hollow != self._hollow:
            self._c, self._hollow = c, hollow
            self.update()

    def set_health(self, h: Health):
        self.set_color(theme.HEALTH_COLOR[h], hollow=(h == Health.UNKNOWN))

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = QtCore.QRectF(self.rect()).adjusted(1, 1, -1, -1)
        if self._hollow:
            p.setPen(QtGui.QPen(self._c, 1.3))
            p.setBrush(QtCore.Qt.NoBrush)
        else:
            p.setPen(QtCore.Qt.NoPen)
            p.setBrush(self._c)
        p.drawEllipse(r)


class Swatch(QtWidgets.QWidget):
    """Short line in a series colour (legend marker)."""

    def __init__(self, color, parent=None):
        super().__init__(parent)
        self._c = QtGui.QColor(color)
        self.setFixedSize(14, 10)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setPen(QtGui.QPen(self._c, 2))
        p.drawLine(1, 5, 13, 5)


class Bar(QtWidgets.QWidget):
    """Thin progress bar (QProgressBar repaints synchronously and is slow at 30 Hz)."""

    def __init__(self, height=4, parent=None):
        super().__init__(parent)
        self._v = 0.0
        self._color = theme.ACCENT
        self.setFixedHeight(height)

    def set_value(self, v, color=None):
        v = min(max(float(v or 0), 0.0), 1.0)
        color = color or theme.ACCENT
        if abs(v - self._v) > 1e-3 or color != self._color:
            self._v, self._color = v, color
            self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor(theme.DIVIDER))
        if self._v > 0:
            p.fillRect(QtCore.QRect(0, 0, int(self.width() * self._v), self.height()), QtGui.QColor(self._color))


class ElidedLabel(QtWidgets.QLabel):
    """Label that elides long text (paths, topic names) and shows the full text as a tooltip."""

    def __init__(self, text='', mode=QtCore.Qt.ElideMiddle, parent=None):
        super().__init__(parent)
        self._full = None
        self._mode = mode
        self.setMinimumWidth(40)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        self.setText(text)

    def setText(self, text):
        text = text or ''
        if text == self._full:
            return
        self._full = text
        self.setToolTip(text)
        self._elide()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._elide()

    def _elide(self):
        fm = self.fontMetrics()
        super().setText(fm.elidedText(self._full or '', self._mode, max(self.width() - 2, 10)))


class KVList(QtWidgets.QWidget):
    """Label / value rows with the same label width and row height in every panel.

    rows: list of (key, label) or ('#', section title).
    Values are right-aligned in a column of value_w px (or the full remaining width when value_w is
    None); elide=True makes them left-aligned and elided in the middle (paths); left=True left-aligns
    them right after the labels (text values).
    """

    def __init__(self, rows, label_w=theme.LABEL_W, value_w=None, elide=False, left=False, parent=None):
        super().__init__(parent)
        g = QtWidgets.QGridLayout(self)
        g.setContentsMargins(0, 0, 0, 0)
        g.setHorizontalSpacing(U)
        g.setVerticalSpacing(0)
        g.setColumnMinimumWidth(0, label_w)
        if value_w:
            g.setColumnMinimumWidth(1, value_w)
            g.setColumnStretch(2, 1)
        else:
            g.setColumnStretch(1, 1)
        self.vals, self.labels = {}, {}
        r = 0
        for key, text in rows:
            if key == '#':
                sec = QtWidgets.QLabel(text)
                sec.setObjectName('section')
                sec.setFixedHeight(theme.ROW_H + (U if r else 0))
                sec.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignBottom)
                g.addWidget(sec, r, 0, 1, 2)
            else:
                lab = QtWidgets.QLabel(text)
                lab.setObjectName('muted')
                lab.setFixedHeight(theme.ROW_H)
                if elide:
                    val = ElidedLabel(DASH)
                else:
                    val = QtWidgets.QLabel(DASH)
                    val.setAlignment((QtCore.Qt.AlignLeft if left else QtCore.Qt.AlignRight) | QtCore.Qt.AlignVCenter)
                val.setFixedHeight(theme.ROW_H)
                g.addWidget(lab, r, 0)
                g.addWidget(val, r, 1)
                self.vals[key], self.labels[key] = val, lab
            r += 1
        g.setRowStretch(r, 1)

    def set(self, key, text, color=None, tip=None, bold=False):
        v = self.vals[key]
        v.setText(text)
        style(v, f'color: {color or theme.TEXT};' + (' font-weight: 600;' if bold else ''))
        if tip is not None and v.toolTip() != tip:
            v.setToolTip(tip)
            self.labels[key].setToolTip(tip)


class ImageView(QtWidgets.QWidget):
    """Draws a numpy image with a fixed physical aspect ratio, plus status text when not live.

    kind: 'gray' (uint8 2D), 'rgb' (uint8 HxWx3) or 'depth' (float metres, 0 = invalid).
    The area outside the image is the panel colour, so there are no black bars.
    """

    def __init__(self, kind='gray', depth_ruler=False, colorbar=False, parent=None):
        super().__init__(parent)
        self.kind = kind
        self.depth_ruler = depth_ruler
        self.colorbar = colorbar
        self._img: Optional[QtGui.QImage] = None
        self._buf = None
        self._aspect = 4 / 3
        self._status = StreamStatus.DISCONNECTED
        self._depth_mm = None
        self._range = (0.0, 1.0)
        self.setMinimumSize(120, 90)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def set_frame(self, arr, status: StreamStatus, pixel_mm=(1.0, 1.0), depth_mm=None, depth_range=None):
        self._status = status
        self._depth_mm = depth_mm
        if depth_range:
            self._range = depth_range
        if arr is None or status != StreamStatus.LIVE:
            if status != StreamStatus.LIVE:
                self._img = None
            self.update()
            return
        h, w = arr.shape[:2]
        self._aspect = (w * pixel_mm[0]) / (h * pixel_mm[1])
        if self.kind == 'rgb':
            buf = np.ascontiguousarray(arr, np.uint8)
            img = QtGui.QImage(buf.data, w, h, 3 * w, QtGui.QImage.Format_RGB888)
        else:
            if self.kind == 'depth':
                lo, hi = self._range
                g = 255 - np.clip((arr - lo) / max(hi - lo, 1e-6), 0, 1) * 255     # near = bright
                g[arr <= 0] = 0                                                     # invalid = black
                buf = np.ascontiguousarray(g, np.uint8)
            else:
                buf = np.ascontiguousarray(arr, np.uint8)
            img = QtGui.QImage(buf.data, w, h, w, QtGui.QImage.Format_Grayscale8)
        self._buf = buf          # QImage does not own the numpy buffer
        self._img = img
        self.update()

    def _target(self):
        r = QtCore.QRectF(self.rect())
        if self.depth_ruler:
            r.adjust(30, 6, 0, -6)
        if self.colorbar:
            r.adjust(0, 0, -56, 0)
        if r.width() <= 0 or r.height() <= 0:
            return r
        if r.width() / r.height() > self._aspect:
            w = r.height() * self._aspect
            return QtCore.QRectF(r.center().x() - w / 2, r.top(), w, r.height())
        h = r.width() / self._aspect
        return QtCore.QRectF(r.left(), r.center().y() - h / 2, r.width(), h)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        t = self._target()
        p.fillRect(t, QtGui.QColor(theme.INSET))
        if self._img is not None and self._status == StreamStatus.LIVE:
            p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
            p.drawImage(t, self._img)
        else:
            msg = {StreamStatus.DISCONNECTED: 'No signal', StreamStatus.LOADING: 'Waiting for frames',
                   StreamStatus.NO_IMAGE: 'No image'}.get(self._status, '')
            p.setPen(QtGui.QColor(theme.FAINT))
            p.drawText(t, QtCore.Qt.AlignCenter, msg)
        p.setFont(theme.small_font())
        if self.depth_ruler and self._depth_mm and self._status == StreamStatus.LIVE:
            x = t.left() - 4
            for mm in range(0, int(self._depth_mm) + 1, 5):
                y = t.top() + t.height() * mm / self._depth_mm
                major = mm % 10 == 0
                p.setPen(QtGui.QColor(theme.FAINT))
                p.drawLine(QtCore.QPointF(x - (5 if major else 3), y), QtCore.QPointF(x, y))
                if major:
                    p.setPen(QtGui.QColor(theme.MUTED))
                    p.drawText(QtCore.QRectF(x - 30, y - 7, 22, 14), QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                               f'{mm}')
        if self.colorbar and self._status == StreamStatus.LIVE:
            bar = QtCore.QRectF(t.right() + U, t.top(), 8, t.height())
            g = QtGui.QLinearGradient(bar.topLeft(), bar.bottomLeft())
            g.setColorAt(0, QtGui.QColor(255, 255, 255))
            g.setColorAt(1, QtGui.QColor(0, 0, 0))
            p.fillRect(bar, g)
            lo, hi = self._range
            p.setPen(QtGui.QColor(theme.MUTED))
            p.drawText(QtCore.QPointF(bar.right() + 4, bar.top() + 9), f'{lo:.1f}')
            p.drawText(QtCore.QPointF(bar.right() + 4, bar.bottom()), f'{hi:.1f} m')
