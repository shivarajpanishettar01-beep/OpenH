"""Sample the exposure and white balance the D415 settles on in auto mode, then lock them.

On the capture PC:
  ros2 launch usrig_bringup wrist_camera.launch.py auto:=true      # point at the phantom, rig lit as for collection
  ros2 run usrig_qc camera_autolock --settle-s 5 --sample-s 5 [--write]
TPV camera: tpv_camera.launch.py auto:=true, then camera_autolock --camera tpv_camera [--write].

Reads the per-frame metadata the camera reports (actual_exposure, white_balance) on the
colour metadata topic, takes the median over --sample-s after --settle-s, and with --write puts
them into session.yaml (<camera>.exposure_us, <camera>.white_balance_k). Then restart the camera without auto:=true.
Metadata keys are those of realsense2_camera 4.x (NOT VERIFIED until run with the camera).
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy

from usrig_common import config

EXPOSURE_KEYS = ('actual_exposure', 'exposure')
WB_KEYS = ('white_balance',)


def _first(d, keys):
    for k in keys:
        if k in d and d[k] is not None:
            return float(d[k])
    return None


def write_session(path, exposure, wb, camera='wrist_camera'):
    """The camera block's exposure_us / white_balance_k (comments and the other camera's values kept)."""
    try:
        config.write_yaml_path(path, f'{camera}.exposure_us', int(round(exposure)))
        config.write_yaml_path(path, f'{camera}.white_balance_k', int(round(wb)))
    except KeyError as e:
        raise RuntimeError(f'could not find {camera}.exposure_us / white_balance_k in {path}') from e


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--settle-s', type=float, default=5.0)
    ap.add_argument('--sample-s', type=float, default=5.0)
    ap.add_argument('--write', action='store_true', help='write the values into session.yaml')
    ap.add_argument('--camera', choices=sorted(config.CAMERAS), default='wrist_camera')
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    scfg = config.load_config('session', a.config_dir)
    topic = scfg[a.camera]['topics'].get('metadata')
    if not topic:
        raise SystemExit(f'{a.camera}.topics.metadata is null in session.yaml')
    from realsense2_camera_msgs.msg import Metadata
    rclpy.init()
    n = rclpy.create_node('camera_autolock')
    rows = []
    t_start = time.monotonic()

    def cb(m):
        if time.monotonic() - t_start < a.settle_s:
            return
        try:
            d = json.loads(m.json_data)
        except ValueError:
            return
        rows.append((_first(d, EXPOSURE_KEYS), _first(d, WB_KEYS), d.get('auto_exposure')))
    n.create_subscription(Metadata, topic, cb, 10)
    while time.monotonic() - t_start < a.settle_s + a.sample_s:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    rclpy.shutdown()
    if not rows:
        raise SystemExit(f'no metadata on {topic} (is the camera running with auto:=true?)')
    exp = [r[0] for r in rows if r[0] is not None]
    wb = [r[1] for r in rows if r[1] is not None]
    print(f'{len(rows)} frames; auto_exposure flag seen: {sorted({str(r[2]) for r in rows})}')
    if not exp or not wb:
        raise SystemExit(f'metadata lacks exposure or white balance (keys seen: exposure {bool(exp)}, wb {bool(wb)})')
    e_med, w_med = float(np.median(exp)), float(np.median(wb))
    print(f'exposure: median {e_med:.0f} (min {min(exp):.0f}, max {max(exp):.0f})  '
          f'[metadata units; librealsense reports colour exposure in units of 100 us on some firmware: check]')
    print(f'white balance: median {w_med:.0f} K (min {min(wb):.0f}, max {max(wb):.0f})')
    if a.write:
        cfg_dir = Path(a.config_dir) if a.config_dir else config.default_config_dir()
        target = cfg_dir / 'session.yaml'
        write_session(target, e_med, w_med, a.camera)
        print(f'wrote {a.camera}: exposure_us={int(round(e_med))}, white_balance_k={int(round(w_med))} to {target}')
        if 'install' in str(target):
            print('note: that is the installed copy; with --symlink-install it is the source file')


if __name__ == '__main__':
    main()
