"""Pivot calibration of a tool tip on the arm (read-only: you move the robot, this tool only reads).

  ros2 run usrig_robot pivot_calib --tool pointer [--ip <controller>] [--write]

Put the tool tip (e.g. the PH calibration pointer) in a fixed divot. Move the arm in freedrive so the
tip stays in the divot while the orientation changes a lot (tilt it 20-40 deg in several directions,
and rotate about the tool axis). Press Enter at each pose (12-20 poses); `q` + Enter solves.

Solves R_i p + t_i = c (tip p in the active-TCP frame, divot c in the base frame) by least squares and
reports the residual per pose. The tip is converted to the flange frame with rig.yaml robot.tcp_setting
(the controller's active TCP offset) and, with --write, stored in rig.yaml as
<tool>.tip_offset_flange_m with a provenance record (<tool>.tip_calibration). Raw poses are saved
under <root>/calibration (session.root, or session.sim_root with --sim). Acceptance (rms, spread) is printed; the decision is yours.
"""
import argparse
import json
import time

import numpy as np

from usrig_common import calibration as C
from usrig_common import config, geometry

from .rig_measure import averaged, connect, rig_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tool', default='pointer', choices=['pointer', 'probe'])
    ap.add_argument('--ip')
    ap.add_argument('--min-poses', type=int, default=12)
    ap.add_argument('--write', action='store_true', help='store the result in rig.yaml')
    ap.add_argument('--sim', action='store_true', help='simulation (raw poses under session.sim_root)')
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    rig_file = rig_path(a.config_dir)
    rig = config.load_yaml(rig_file)
    scfg = config.load_config('session', a.config_dir)
    ip = a.ip or config.get_path(rig, 'robot.ip')
    if not ip:
        raise SystemExit('give --ip (rig.yaml robot.ip is null)')
    tcp = config.get_path(rig, 'robot.tcp_setting')
    if a.write and tcp is None:
        raise SystemExit('rig.yaml robot.tcp_setting is null: needed to express the tip in the flange frame')

    try:
        r = connect(ip)
    except RuntimeError as e:
        raise SystemExit(str(e))
    poses = []
    try:
        print(__doc__.split('\n\n')[1])
        while True:
            s = input(f'[{len(poses)} poses] Enter = record (tip in the divot, arm still), q = solve: ').strip().lower()
            if s == 'q':
                break
            Ts, _ = averaged(r, 0.3)
            T = Ts[len(Ts) // 2]
            poses.append(T)
            print(f'  TCP {np.round(T[:3, 3] * 1000, 1).tolist()} mm')
    finally:
        r.disconnect()
    if len(poses) < a.min_poses:
        raise SystemExit(f'only {len(poses)} poses, need {a.min_poses}')

    res = C.pivot_calibration(poses)
    tip_tcp = res['tip_in_tcp_m']
    print(f'\ntip in active-TCP frame: {np.round(tip_tcp * 1000, 3).tolist()} mm')
    print(f'divot in base frame:    {np.round(res["pivot_in_base_m"] * 1000, 2).tolist()} mm')
    print(f'residual rms {res["rms_mm"]:.3f} mm, max {res["max_mm"]:.3f} mm over {res["n_poses"]} poses; '
          f'orientation spread {res["orientation_spread_deg"]:.1f} deg (use > 40 deg)')
    print('per pose (mm): ' + ' '.join(f'{x:.2f}' for x in res['residuals_mm']))
    tip_flange = None
    if tcp is not None:
        T_flange_tcp = geometry.pose_from_ur(tcp)
        tip_flange = (T_flange_tcp @ np.append(tip_tcp, 1.0))[:3]
        print(f'tip in flange frame:    {np.round(tip_flange * 1000, 3).tolist()} mm '
              f'(UR TCP to enter on the pendant for this tool: x, y, z = these values, orientation as needed)')

    stamp = time.strftime('%Y%m%d_%H%M%S')
    from usrig_common.session import data_dir
    out_dir = data_dir(scfg, 'calibration', a.sim)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw = out_dir / f'pivot_{a.tool}_{stamp}.json'
    raw.write_text(json.dumps({'tool': a.tool, 'tcp_setting': tcp, 'poses': [p.tolist() for p in poses],
                               'tip_in_tcp_m': tip_tcp.tolist(),
                               'tip_in_flange_m': None if tip_flange is None else tip_flange.tolist(),
                               'pivot_in_base_m': res['pivot_in_base_m'].tolist(),
                               'residuals_mm': res['residuals_mm'].tolist(), 'rms_mm': res['rms_mm'],
                               'orientation_spread_deg': res['orientation_spread_deg']}, indent=1))
    print(f'raw poses and result: {raw}')
    if not a.write:
        return
    if a.tool == 'probe':
        print('probe: not written (a flat probe face cannot be pivoted precisely); '
              'if you trust the value, put it in rig.yaml probe.tip_offset_flange_m yourself')
        return
    # by full path: tip_offset_flange_m exists under both pointer: and probe:
    config.write_yaml_path(rig_file, 'pointer.tip_offset_flange_m', [round(float(v), 6) for v in tip_flange])
    config.write_yaml_path(rig_file, 'pointer.tip_calibration', {
        'date': time.strftime('%Y-%m-%d %H:%M'), 'method': 'pivot', 'n_poses': res['n_poses'],
        'rms_mm': round(res['rms_mm'], 3), 'max_mm': round(res['max_mm'], 3),
        'orientation_spread_deg': round(res['orientation_spread_deg'], 1), 'file': str(raw)})
    print(f'written to {rig_file}: pointer.tip_offset_flange_m + pointer.tip_calibration')

if __name__ == '__main__':
    main()
