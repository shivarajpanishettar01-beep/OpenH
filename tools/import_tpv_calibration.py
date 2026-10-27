#!/usr/bin/env python3
"""Bring the third-person view (TPV) camera's eye-to-hand calibration into OpenH as tpv_cam_cali_mtx.

The TPV camera is calibrated on the calibration rig (ur7e_calib/tpv: manual_gui, re-located from the fixed
board with relocate.py). Its calibration.json holds T_base_cam (p_base = T_base_cam @ p_cam, camera = the
RealSense colour optical frame, base = the UR base frame, metres). That is OpenH's T_base_camera.

  python tools/import_tpv_calibration.py ~/calib/ur7e_calib/tpv/calibration.json            # show + check
      [--write-rig] [--session <session_dir>] [--config-dir <dir>]

--write-rig stores it in rig.yaml (calibration.tpv_cam_cali_mtx + tpv_cam_cali_info, all later sessions);
--session stores it in that session's session.json (tpv_cam_cali_mtx, tpv_cam_cali), which wins over rig.yaml.
Re-run after every relocate.py run: the matrix only holds while the camera has not moved.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]


def load(path):
    """(T_base_camera 4x4, provenance) from a calibration.json of the calibration rig; raises ValueError."""
    d = json.loads(Path(path).expanduser().read_text())
    if 'T_base_cam' not in d:
        raise ValueError(f'{path}: no T_base_cam')
    if d.get('units', 'm') != 'm':
        raise ValueError(f'{path}: units {d.get("units")!r}, expected m')
    T = np.asarray(d['T_base_cam'], float)
    if T.shape != (4, 4) or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError(f'{path}: T_base_cam is not a 4x4 homogeneous matrix')
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-4) or np.linalg.det(R) < 0:
        raise ValueError(f'{path}: T_base_cam rotation is not orthonormal')
    cam = d.get('camera') or {}
    reloc = d.get('relocations') or []
    info = {'date': time.strftime('%Y-%m-%d %H:%M'), 'source': str(Path(path).expanduser().resolve()),
            'frame': 'T_base_camera (TPV colour optical frame in the UR base frame, metres)',
            'method': d.get('camera_source'), 'created': d.get('created'),
            'rmse_mm': None if d.get('rmse_m') is None else round(1000 * d['rmse_m'], 3),
            'loo_rmse_mm': None if d.get('loo_rmse_m') is None else round(1000 * d['loo_rmse_m'], 3),
            'max_residual_mm': None if d.get('max_residual_m') is None else round(1000 * d['max_residual_m'], 3),
            'camera_serial': cam.get('serial'), 'camera_name': cam.get('name'),
            'calibrated_color_profile': cam.get('color_profile'),
            'last_relocation': reloc[-1].get('time') if reloc else None}
    return T, info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('calibration', help='calibration.json of the calibration rig (ur7e_calib/tpv)')
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
    print('T_base_camera (TPV):\n' + np.array2string(T, precision=5, suppress_small=True))
    print(f'residual: rmse {info["rmse_mm"]} mm, leave-one-out {info["loo_rmse_mm"]} mm, max {info["max_residual_mm"]} mm'
          f'; calibrated {info["created"]}, last relocation {info["last_relocation"]}')
    tpv_serial = (scfg.get('tpv_camera') or {}).get('serial')
    if info['camera_serial'] and tpv_serial and str(info['camera_serial']) != str(tpv_serial):
        print(f'WARNING: calibrated camera {info["camera_serial"]} is not session.yaml tpv_camera.serial {tpv_serial}')
    elif tpv_serial is None:
        print('note: session.yaml tpv_camera.serial is null: cannot check this is the TPV camera')
    if a.write_rig:
        rig_file = cfg_dir / 'rig.yaml'
        config.write_yaml_path(rig_file, 'calibration.tpv_cam_cali_mtx', np.round(T, 7).tolist())
        config.write_yaml_path(rig_file, 'calibration.tpv_cam_cali_info', info)
        print(f'written to {rig_file}: calibration.tpv_cam_cali_mtx + tpv_cam_cali_info')
    if a.session:
        update_session_json(a.session, tpv_cam_cali_mtx=T.tolist(), tpv_cam_cali=info)
        print(f'stored in {Path(a.session) / "session.json"}')


if __name__ == '__main__':
    main()
