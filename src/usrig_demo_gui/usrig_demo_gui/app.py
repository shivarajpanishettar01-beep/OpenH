"""Entry point.

  python -m usrig_demo_gui                         # mock data, scenario 'normal'
  python -m usrig_demo_gui --scenario recovery
  ros2 run usrig_demo_gui demo_gui                 # same, from the ROS workspace
  ros2 run usrig_demo_gui demo_gui --source ros    # read-only ROS 2 adapter (draft, see docs)
"""
from __future__ import annotations

import argparse
import signal
import sys


def parse(argv):
    ap = argparse.ArgumentParser(prog='demo_gui', description='UR7e robotic ultrasound demo monitor (read-only).')
    ap.add_argument('--source', choices=['mock', 'ros'], default='mock')
    ap.add_argument('--scenario', default='normal',
                    help='mock scenario: normal idle degraded disconnected recovery success failure aborted')
    ap.add_argument('--config', default=None, help='path to demo_gui.yaml (default: installed copy)')
    ap.add_argument('--size', default='1920x1080', help='window size WxH')
    ap.add_argument('--fullscreen', action='store_true')
    ap.add_argument('--robot-view', choices=['auto', 'gl', 'simple'], default='auto',
                    help='gl = UR meshes (needs OpenGL + PyOpenGL), simple = line drawing; auto picks gl if it loads')
    ap.add_argument('--screenshot', default=None, help='save a PNG of the window after --after seconds and quit')
    ap.add_argument('--after', type=float, default=4.0)
    return ap.parse_args(argv)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    try:                                    # drop --ros-args ... when started by ros2 run / launch
        from rclpy.utilities import remove_ros_args
        argv = remove_ros_args(['x'] + list(argv))[1:]
    except ImportError:
        pass
    args = parse(argv)

    from PySide6 import QtCore, QtWidgets
    import pyqtgraph as pg

    from . import config
    from .main_window import MainWindow, apply_theme

    cfg = config.load(args.config)
    if cfg.get('gui_nice'):
        try:
            import os
            os.nice(int(cfg['gui_nice']))       # same idea as usrig_dashboard: the GUI yields CPU
        except (AttributeError, OSError):
            pass
    if args.source == 'mock':
        from .sources.mock import MockSource
        source = MockSource(cfg, args.scenario)
    else:
        from .sources.ros_source import RosSource
        source = RosSource(cfg)

    pg.setConfigOptions(antialias=False)       # same as the existing dashboard; cheaper
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    apply_theme(app)
    source.start()
    win = MainWindow(source, cfg, robot_view=args.robot_view)
    w, h = (int(x) for x in args.size.lower().split('x'))
    win.resize(w, h)
    win.showFullScreen() if args.fullscreen else win.show()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())
    alive = QtCore.QTimer()
    alive.timeout.connect(lambda: None)     # lets Python run the signal handlers
    alive.start(200)

    if args.screenshot:
        def shot():
            img = win.grab()
            rv = win.robot.view
            if hasattr(rv, 'grab_image'):                     # the OpenGL view is not in grab(); paint it in
                from PySide6 import QtGui
                p = QtGui.QPainter(img)
                p.drawImage(rv.container.mapTo(win, QtCore.QPoint(0, 0)), rv.grab_image())
                p.end()
            img.save(args.screenshot)
            QtCore.QTimer.singleShot(0, app.quit)
        QtCore.QTimer.singleShot(int(args.after * 1000), shot)

    rc = app.exec()
    source.stop()
    return rc


if __name__ == '__main__':
    sys.exit(main())
