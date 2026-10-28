#!/usr/bin/env python3
"""Bring the wrist camera's eye-in-hand calibration from the calibration rig into OpenH as wrist_cam_cali_mtx.

The wrist D415 is calibrated on the calibration rig (ur7e_calib/wrist: wrist_gui.py, touch method). Its
wrist_calibration.json holds T_flange_cam (p_flange = T_flange_cam @ p_cam, camera = the RealSense colour optical
frame, flange = UR tool0, metres) next to T_tcp_cam for the pointer TCP used while calibrating. T_flange_cam is
OpenH's T_flange_camera; it does not depend on the TCP, so it holds for the probe as long as the camera bracket on
the flange has not moved since the calibration.

  python tools/import_wrist_calibration.py ~/calib/ur7e_calib/wrist/wrist_calibration.json   # show + check
      [--write-rig] [--session <session_dir>] [--config-dir <dir>]

--write-rig stores it in rig.yaml (calibration.wrist_cam_cali_mtx + wrist_cam_cali_info, all later sessions);
--session stores it in that session's session.json (wrist_cam_cali_mtx, wrist_cam_cali), which wins over rig.yaml.
Re-calibrate (and re-run this) after the camera bracket is moved.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]


def homogeneous(T, what):
    """T as a checked 4x4 rigid transform; raises ValueError."""
    T = np.asarray(T, float)
    if T.shape != (4, 4) or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError(f'{what} is not a 4x4 homogeneous matrix')
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-4) or np.linalg.det(R) < 0:
        raise ValueError(f'{what} rotation is not orthonormal')
    return T


def load(path):
    """(T_flange_camera 4x4, provenance) from a wrist_calibration.json of the calibration rig; raises ValueError.
    When the file also has T_tcp_cam and the pendant TCP it was taken with, T_flange_cam must equal
    T_flange_tcp @ T_tcp_cam (catches a T_flange_cam written for another TCP)."""
    d = json.loads(Path(path).expanduser().read_text())
    if 'T_flange_cam' not in d:
        raise ValueError(f'{path}: no T_flange_cam (only a TCP-relative calibration; enter the TCP offset in the '
                         'calibration rig first)')
    T = homogeneous(d['T_flange_cam'], f'{path}: T_flange_cam')
    tcp = d.get('tcp_offset_pendant')
    if d.get('T_tcp_cam') is not None and tcp is not None:
        T_flange_tcp = np.eye(4)
        T_flange_tcp[:3, :3] = Rotation.from_rotvec(tcp[3:6]).as_matrix()
        T_flange_tcp[:3, 3] = np.asarray(tcp[:3], float) / 1000.0              # pendant TCP: mm + rotation vector
        if not np.allclose(T_flange_tcp @ homogeneous(d['T_tcp_cam'], f'{path}: T_tcp_cam'), T, atol=1e-6):
            raise ValueError(f'{path}: T_flange_cam != T_flange_tcp(tcp_offset_pendant) @ T_tcp_cam')
    cam = d.get('camera') or {}
    mm = lambda k: None if d.get(k) is None else round(float(d[k]), 3)
    info = {'date': time.strftime('%Y-%m-%d %H:%M'), 'source': str(Path(path).expanduser().resolve()),
            'frame': 'T_flange_camera (wrist colour optical frame in the UR flange / tool0 frame, metres)',
            'method': d.get('kind'), 'created': d.get('created'), 'calibration_session': d.get('session'),
            'rms_mm': mm('rms_mm'), 'loo_mm': mm('loo_mm'), 'max_mm': mm('max_mm'),
            'tcp_offset_pendant_while_calibrating': tcp,
            'camera_serial': cam.get('serial'), 'camera_name': cam.get('name'),
            'calibrated_color_profile': cam.get('color_profile')}
    return T, info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('calibration', help='wrist_calibration.json of the calibration rig (ur7e_calib/wrist)')
    ap.add_argument('--write-rig', action='store_true', help='store it in rig.yaml (all later sessions)')
    ap.add_argument('--session', help='also store it in <session>/session.json')
    ap.add_argument('--config-dir')
    a = ap.parse_args()
    sys.path.insert(0, str(REPO / 'src/usrig_common'))
    from usrig_common import config
    from usrig_common.session import update_session_json
    try:
        T, info = load(a.calibration)
    except ValueError as e:
        sys.exit(str(e))
    cfg_dir = Path(a.config_dir) if a.config_dir else REPO / 'src/usrig_bringup/config'
    scfg = config.load_config('session', str(cfg_dir))
    print('T_flange_camera (wrist):\n' + np.array2string(T, precision=5, suppress_small=True))
    print(f'residual: rms {info["rms_mm"]} mm, leave-one-out {info["loo_mm"]} mm, max {info["max_mm"]} mm'
          f'; calibrated {info["created"]}')
    wrist_serial = (scfg.get('wrist_camera') or {}).get('serial')
    if info['camera_serial'] and wrist_serial and str(info['camera_serial']) != str(wrist_serial):
        print(f'WARNING: calibrated camera {info["camera_serial"]} is not session.yaml wrist_camera.serial {wrist_serial}')
    if a.write_rig:
        rig_file = cfg_dir / 'rig.yaml'
        config.write_yaml_path(rig_file, 'calibration.wrist_cam_cali_mtx', np.round(T, 7).tolist())
        config.write_yaml_path(rig_file, 'calibration.wrist_cam_cali_info', info)
        print(f'written to {rig_file}: calibration.wrist_cam_cali_mtx + wrist_cam_cali_info')
    if a.session:
        update_session_json(a.session, wrist_cam_cali_mtx=T.tolist(), wrist_cam_cali=info)
        print(f'stored in {Path(a.session) / "session.json"}')


if __name__ == '__main__':
    main()
