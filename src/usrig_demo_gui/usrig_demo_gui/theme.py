"""Colours, type and sizes.

Light theme for a lab monitor whose screenshots also go into reports: white panes on a light
grey frame, near-black text, one blue for the probe and the current phase, and the usual status
colours. Type is IBM Plex Sans (bundled, OFL) with tabular digits; IBM Plex Mono only for ROS
topic names and file paths. Colour carries status or a data series, nothing else.
"""
from pathlib import Path

from PySide6 import QtGui

from .models import Health

# surfaces
DIVIDER = '#d3d7dc'     # 1 px lines between panes and rail sections
PANEL = '#ffffff'
TITLE_BG = PANEL        # pane headings sit on the pane itself (no title bars)
INSET = '#eef0f2'       # behind images when there is no signal
VIEW_3D = '#f5f6f8'     # 3D robot view background
VIEW_BG = INSET
GRID_3D = (0, 0, 0, 34) # floor grid in the 3D view (RGBA)
GRID_2D = '#dde0e4'     # floor grid in the fallback robot drawing
# text
TEXT = '#1d232b'
MUTED = '#5b6470'
FAINT = '#9aa1aa'
# status
ACCENT = '#0b5cad'      # current phase, probe tip
ACCENT_DIM = '#b9d0ea'  # phases already done
GREEN = '#1a7f37'
ORANGE = '#b35c00'
RED = '#c62828'
GREY = '#6e7781'
# trajectories in the robot view
TRAJ_FLANGE = '#7b3fa0'   # flange / end effector
TRAJ_TIP = '#0b5cad'      # probe tip while in contact (or contact unknown)
TRAJ_TIP_AIR = '#7fb0e6'  # probe tip while not in contact
TRAJ_CAMERA = '#00838f'   # wrist camera (optical centre)

HEALTH_COLOR = {Health.OK: GREEN, Health.WARN: ORANGE, Health.DOWN: RED, Health.UNKNOWN: GREY}
AXIS_COLORS = ('#d1242f', '#1a7f37', '#0969da')     # x y z

# sizes (px)
UNIT = 8                 # spacing unit; paddings are multiples of it
FONT_PX = 12
SMALL_PX = 11
TITLE_PX = 13
TITLE_H = 30             # pane heading row height
ROW_H = 20               # label/value row height
LABEL_W = 116            # label column width in label/value lists

SANS = 'IBM Plex Sans'
MONO = 'IBM Plex Mono'
_FONT_DIR = Path(__file__).resolve().parent / 'assets' / 'fonts'
_loaded = False


def load_fonts():
    """Register the bundled fonts (call once after the QApplication exists)."""
    global _loaded
    if _loaded:
        return
    for f in sorted(_FONT_DIR.glob('*.ttf')):
        QtGui.QFontDatabase.addApplicationFont(str(f))
    _loaded = True


def app_font(px=FONT_PX, weight=None):
    f = QtGui.QFont(SANS)
    f.setPixelSize(px)
    if weight is not None:
        f.setWeight(weight)
    try:                                         # tabular digits so numbers line up
        f.setFeature(QtGui.QFont.Tag('tnum'), 1)
    except Exception:
        pass
    return f


def small_font(bold=False):
    return app_font(SMALL_PX, QtGui.QFont.DemiBold if bold else None)


def mono_font(px=SMALL_PX):
    f = QtGui.QFont(MONO)
    f.setStyleHint(QtGui.QFont.Monospace)
    f.setPixelSize(px)
    return f


STYLESHEET = f"""
QMainWindow {{ background: {DIVIDER}; }}
QWidget {{ color: {TEXT}; }}
QWidget#grid {{ background: {DIVIDER}; }}
QFrame#panel, QWidget#header, QWidget#rail {{ background: {PANEL}; }}
QFrame#titlebar {{ background: {TITLE_BG}; }}
QLabel#panelTitle {{ font-weight: 600; font-size: {TITLE_PX}px; }}
QLabel#section {{ color: {MUTED}; font-size: {SMALL_PX}px; font-weight: 600; }}
QLabel#tag {{ font-size: {SMALL_PX}px; font-weight: 500; padding: 1px 6px; border-radius: 2px; }}
QLabel#muted {{ color: {MUTED}; }}
QLabel#small {{ color: {MUTED}; font-size: {SMALL_PX}px; }}
QLabel#title {{ font-size: 15px; font-weight: 600; }}
QFrame#vline, QFrame#hline {{ background: {DIVIDER}; }}
QStatusBar {{ background: {PANEL}; color: {MUTED}; border-top: 1px solid {DIVIDER}; font-size: {SMALL_PX}px; }}
QStatusBar::item {{ border: none; }}
QComboBox {{ background: {PANEL}; border: 1px solid {DIVIDER}; padding: 1px 6px; min-width: 110px;
            font-size: {SMALL_PX}px; }}
QToolTip {{ background: {PANEL}; color: {TEXT}; border: 1px solid {DIVIDER}; padding: 4px; }}
QScrollArea {{ border: none; background: {DIVIDER}; }}
QScrollBar:vertical {{ background: {PANEL}; width: 10px; }}
QScrollBar::handle:vertical {{ background: {DIVIDER}; min-height: 30px; }}
"""
