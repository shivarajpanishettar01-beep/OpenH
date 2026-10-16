"""DashboardHeader: title + read-only status fields in equal-width cells. No buttons, by design."""
from __future__ import annotations

from PySide6 import QtCore, QtWidgets

from .. import theme
from ..models import ContactState, DashboardState, Outcome, PlaneState, SystemState, TaskPhase
from .common import DASH, U, style, vline

# value -> (colour, hollow)
SYSTEM_STYLE = {SystemState.RUNNING: (theme.GREEN, False), SystemState.IDLE: (theme.TEXT, False),
                SystemState.DEGRADED: (theme.ORANGE, False), SystemState.DISCONNECTED: (theme.RED, False)}
CONTACT_STYLE = {ContactState.IN_CONTACT: (theme.GREEN, False), ContactState.NO_CONTACT: (theme.TEXT, False),
                 ContactState.UNKNOWN: (theme.GREY, True)}
PLANE_STYLE = {PlaneState.IN_PLANE: (theme.GREEN, False), PlaneState.OUT_OF_PLANE: (theme.ORANGE, False),
               PlaneState.UNKNOWN: (theme.GREY, True)}
OUTCOME_STYLE = {Outcome.IN_PROGRESS: (theme.TEXT, False), Outcome.SUCCESS: (theme.GREEN, False),
                 Outcome.FAILURE: (theme.RED, False), Outcome.ABORTED: (theme.ORANGE, False)}


class Field(QtWidgets.QWidget):
    def __init__(self, caption):
        super().__init__()
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(2 * U, 0, 2 * U, 0)
        lay.setSpacing(2)
        cap = QtWidgets.QLabel(caption)
        cap.setObjectName('small')
        self.val = QtWidgets.QLabel(DASH)
        lay.addStretch(1)
        lay.addWidget(cap)
        lay.addWidget(self.val)
        lay.addStretch(1)

    def set(self, text, color, hollow=False):
        """Value text in the state colour; unknown / not applicable in grey."""
        self.val.setText(text)
        style(self.val, f'font-weight: 600; font-size: 13px; color: {theme.FAINT if hollow else color};')


class DashboardHeader(QtWidgets.QWidget):
    def __init__(self, title, subtitle):
        super().__init__()
        self.setObjectName('header')
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.setFixedHeight(52)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(U + 4, 0, U + 4, 0)
        lay.setSpacing(0)
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(1)
        t = QtWidgets.QLabel(title)
        t.setObjectName('title')
        s = QtWidgets.QLabel(subtitle)
        s.setObjectName('small')
        left.addStretch(1)
        left.addWidget(t)
        left.addWidget(s)
        left.addStretch(1)
        lay.addLayout(left)
        lay.addStretch(1)
        self.system = Field('System')
        self.contact = Field('Contact')
        self.plane = Field('Plane')
        self.phase = Field('Task phase')
        self.outcome = Field('Outcome')
        for f in (self.system, self.contact, self.plane, self.phase, self.outcome):
            lay.addWidget(vline())
            f.setFixedWidth(150)
            lay.addWidget(f)
        lay.addWidget(vline())
        lay.addSpacing(2 * U)
        self.badge = QtWidgets.QLabel('')
        self.badge.setObjectName('tag')
        self.badge.setAlignment(QtCore.Qt.AlignCenter)
        lay.addWidget(self.badge, 0, QtCore.Qt.AlignVCenter)

    def update_state(self, d: DashboardState):
        if d.paused_reason:
            self.badge.setText('Paused')
            self.badge.setToolTip('A real collection session is running. Nothing is subscribed, so nothing '
                                  'on this screen is current.')
            style(self.badge, f'color: {theme.MUTED}; border: 1px solid {theme.MUTED};')
            # no data while paused: the fields say so instead of showing a made-up "Idle"
            self.system.set('Paused', theme.MUTED)
            for f in (self.contact, self.plane, self.phase, self.outcome):
                f.set(DASH, theme.GREY, True)
            return
        if d.is_mock:
            self.badge.setText('Simulated data')
            self.badge.setToolTip('All values on screen are synthetic. No robot, scanner or camera is connected.')
            style(self.badge, f'color: {theme.ORANGE}; border: 1px solid {theme.ORANGE};')
        else:
            self.badge.setText('Live, read-only')
            self.badge.setToolTip('Live data from the ROS 2 topics. This screen only subscribes: it never '
                                  'publishes or commands the robot.')
            style(self.badge, f'color: {theme.GREEN}; border: 1px solid {theme.GREEN};')
        self.system.set(d.system.value, *SYSTEM_STYLE[d.system])
        self.contact.set(d.contact.value, *CONTACT_STYLE[d.contact])
        self.plane.set(d.plane.value, *PLANE_STYLE[d.plane])
        ph = d.task.phase
        if ph is None:
            self.phase.set(DASH, theme.GREY, True)
        else:
            self.phase.set(ph.value, theme.ORANGE if ph == TaskPhase.RECOVERY else theme.ACCENT)
        oc = d.task.outcome
        if oc is None:
            self.outcome.set(DASH, theme.GREY, True)
        else:
            self.outcome.set(oc.value, *OUTCOME_STYLE[oc])
