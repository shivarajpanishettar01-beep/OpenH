"""Phantom registration: touch 4 marked fiducials with the probe tip, solve with Kabsch.

  ros2 run usrig_episode register_phantom <session_dir> [--phantom-file <root>/phantoms/<id>.yaml]

phantoms/<id>.yaml:  fiducials_mm: {F1: [x, y, z], F2: ..., F3: ..., F4: ...}  (phantom frame)
Teleop the probe tip onto each fiducial (keyboard_teleop runs separately), press Enter here.
With --tool pointer the calibrated pointer tip (pivot_calib) is used instead: sharper, more accurate.
Rejected if the RMS residual exceeds rig.yaml registration_max_residual_mm.
Saves T_base_phantom to <session>/session.json; rig_static_tf publishes it on /tf_static.
"""
import argparse
from pathlib import Path

import numpy as np

from usrig_common import config, geometry
from usrig_common.session import update_session_json
from usrig_common.timeutil import now_ns

from usrig_common.tool_node import run_tool


def phantom_file(scfg, session):
    """<root>/phantoms/<phantom>.yaml of the session's phantom (session.json, written by episode_manager from the
    launch's phantom_id; session.yaml's default only if the session has none)."""
    from usrig_common.session import data_dir, read_json, session_is_sim
    pid = read_json(Path(session) / 'session.json').get('phantom_id') or scfg['session'].get('phantom_id')
    if not pid:
        raise ValueError('no phantom_id in session.json or session.yaml: give --phantom-file')
    return data_dir(scfg, 'phantoms', session_is_sim(session)) / f'{pid}.yaml'


def load_fiducials(path):
    """{name: [x, y, z] mm} from phantoms/<id>.yaml; every value must be measured (no nulls), at least 4."""
    fid = (config.load_yaml(path) or {}).get('fiducials_mm') or {}
    bad = [k for k, v in fid.items() if not isinstance(v, (list, tuple)) or len(v) != 3
           or any(not isinstance(x, (int, float)) for x in v)]
    if bad:
        raise ValueError(f'{path}: fiducials {bad} are not three measured numbers (mm, phantom frame)')
    if len(fid) < 4:
        raise ValueError(f'{path}: need at least 4 fiducials, has {len(fid)}')
    return fid


def _register(node, a):
    max_res = config.get_path(node.rig, 'registration_max_residual_mm')
    if max_res is None:
        raise config.UnsetLimitError(['registration_max_residual_mm'], 'rig.yaml')
    pf = a.phantom_file or phantom_file(node.scfg, a.session)
    fid = load_fiducials(pf)
    src, dst = [], []
    for name, p_mm in fid.items():
        input(f'Put the probe tip on fiducial {name} {p_mm} mm and press Enter... ')
        p = node.averaged_tip_position(tool=a.tool)
        print(f'  {name}: base {np.round(p * 1000, 2)} mm')
        src.append(np.asarray(p_mm, float) / 1000.0)
        dst.append(p)
    T, rms = geometry.kabsch(src, dst)
    rms_mm = rms * 1000.0
    per = np.linalg.norm((T[:3, :3] @ np.array(src).T).T + T[:3, 3] - np.array(dst), axis=1) * 1000
    print(f'RMS residual {rms_mm:.2f} mm (per fiducial {np.round(per, 2)} mm), limit {max_res} mm')
    if rms_mm > max_res:
        raise ValueError('registration rejected: residual above registration_max_residual_mm')
    update_session_json(a.session, phantom_registration={
        'T_base_phantom': T.tolist(), 'rms_residual_mm': rms_mm, 'per_fiducial_mm': per.tolist(),
        'fiducials_mm': fid, 'phantom_file': str(pf), 'tool': a.tool, 'stamp_ns': now_ns()})
    print(f'saved to {Path(a.session) / "session.json"}')
    # robot_control enforces the phantom-frame box (D9) once it has the registration
    import json as _json
    from std_msgs.msg import String
    pub = node.create_publisher(String, '/operator/command', 10)
    import time as _time
    _time.sleep(0.5)
    pub.publish(String(data=_json.dumps({'cmd': 'reload_registration'})))
    print('asked robot_control to reload the registration (phantom box active) and rig_static_tf to republish it')
    # robot_state computes the recorded tcp_phantom from its T_base_phantom parameter: set it on the running node
    from usrig_common import registration_sync
    ok, msg = registration_sync.push(node, T)
    print(f'robot_state tcp_phantom: {"updated" if ok else "NOT updated"} ({msg})')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--phantom-file')
    ap.add_argument('--tool', choices=['probe', 'pointer'], default='probe')
    a, _ = ap.parse_known_args()
    run_tool(lambda n: _register(n, a), 'register_phantom')


if __name__ == '__main__':
    main()
