"""Read-only measurements for filling rig.yaml. Never commands the robot: it only reads RTDE.
You move the robot (freedrive or pendant); this tool records what the controller reports.

  ros2 run usrig_robot rig_measure pose                       # current TCP pose, force, modes
  ros2 run usrig_robot rig_measure box   [--write]            # workspace box from touched corner points
  ros2 run usrig_robot rig_measure tilt-axis [--write]        # probe axis with the probe square to the phantom
  ros2 run usrig_robot rig_measure force-noise --seconds 10   # F/T noise in free air (contact threshold input)
  add --ip <controller>  (default: rig.yaml robot.ip; URSim: 127.0.0.1)

--write puts the measured value into rig.yaml (same key, comments kept). Margins, padding and
every force limit stay your decision: this tool reports measurements, it does not choose limits.
"""
import argparse
import re
import time
from pathlib import Path

import numpy as np

from usrig_common import config, geometry


def connect(ip):
    import rtde_receive
    return rtde_receive.RTDEReceiveInterface(ip)


def tcp(r):
    return geometry.pose_from_ur(r.getActualTCPPose())


def averaged(r, seconds=0.5):
    Ts, fs = [], []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        Ts.append(tcp(r))
        fs.append(np.asarray(r.getActualTCPForce(), float))
        time.sleep(0.005)
    return Ts, np.array(fs)


def rig_path(config_dir):
    d = Path(config_dir) if config_dir else config.default_config_dir()
    return d / 'rig.yaml'


def write_key(path, key, value):
    """Replace `key: <anything>` (first occurrence) in rig.yaml, keeping comments."""
    text = Path(path).read_text()
    flow = '[' + ', '.join(f'{v:.5f}' if isinstance(v, float) else str(v) for v in value) + ']' \
        if isinstance(value, (list, tuple)) else str(value)
    pat = re.compile(rf'^(\s*{re.escape(key)}:\s*)([^#\n]*?)(\s*(#.*)?)$', re.M)
    new, n = pat.subn(lambda m: m.group(1) + flow + (('   ' + m.group(4)) if m.group(4) else ''), text, count=1)
    if n != 1:
        raise RuntimeError(f'{key} not found in {path}')
    Path(path).write_text(new)


def cmd_pose(r, a):
    T = tcp(r)
    p6 = geometry.ur_from_pose(T)
    print(f'TCP [m, rotvec rad]: {np.round(p6, 5).tolist()}')
    print(f'probe axis (TCP z) in base: {np.round(T[:3, 2], 4).tolist()}')
    print(f'force/torque: {np.round(r.getActualTCPForce(), 3).tolist()}')
    print(f'robot mode {r.getRobotMode()}  safety mode {r.getSafetyMode()}')


def cmd_box(r, a):
    print('Move the probe tip to each extreme of the allowed scanning volume (corners / limits),')
    print('press Enter at each; type q + Enter when done (at least 2 points).')
    pts = []
    while True:
        s = input(f'[{len(pts)} points] Enter = record, q = done: ').strip().lower()
        if s == 'q':
            break
        Ts, _ = averaged(r)
        p = np.mean([T[:3, 3] for T in Ts], axis=0)
        pts.append(p)
        print(f'  {np.round(p * 1000, 1).tolist()} mm')
    if len(pts) < 2:
        raise SystemExit('need at least 2 points')
    P = np.array(pts)
    lo, hi = P.min(0), P.max(0)
    print(f'measured box min {np.round(lo, 4).tolist()} m, max {np.round(hi, 4).tolist()} m '
          f'(size {np.round((hi - lo) * 1000, 1).tolist()} mm). No padding added.')
    if a.write:
        write_key(rig_path(a.config_dir), 'min', lo.tolist())
        write_key(rig_path(a.config_dir), 'max', hi.tolist())
        print('written to limits.workspace_box_base_m in rig.yaml')


def cmd_tilt_axis(r, a):
    input('Hold the probe square to the phantom surface (as at a normal scan start), press Enter... ')
    Ts, _ = averaged(r, 1.0)
    z = np.mean([T[:3, 2] for T in Ts], axis=0)
    z = z / np.linalg.norm(z)
    print(f'probe axis in base frame: {np.round(z, 4).tolist()}')
    if a.write:
        write_key(rig_path(a.config_dir), 'tilt_reference_axis_base', [round(float(x), 4) for x in z])
        print('written to limits.tilt_reference_axis_base in rig.yaml')


def cmd_force_noise(r, a):
    input(f'Probe in free air, robot still. Press Enter to record {a.seconds:.0f} s... ')
    Ts, F = averaged(r, a.seconds)
    axis = np.array([T[:3, 2] @ f[:3] for T, f in zip(Ts, F)])
    norm = np.linalg.norm(F[:, :3], axis=1)
    print(f'{len(F)} samples')
    print(f'probe-axis force: mean {axis.mean():.3f} N, std {axis.std():.3f} N, '
          f'peak-to-peak {np.ptp(axis):.3f} N')
    print(f'|F|: mean {norm.mean():.3f} N, max {norm.max():.3f} N')
    print('Use this noise floor when choosing contact_detect_threshold_n (the choice is yours).')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('what', choices=['pose', 'box', 'tilt-axis', 'force-noise'])
    ap.add_argument('--ip')
    ap.add_argument('--write', action='store_true')
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    rig = config.load_yaml(rig_path(a.config_dir))
    ip = a.ip or config.get_path(rig, 'robot.ip')
    if not ip:
        raise SystemExit('give --ip (rig.yaml robot.ip is null)')
    r = connect(ip)
    try:
        {'pose': cmd_pose, 'box': cmd_box, 'tilt-axis': cmd_tilt_axis, 'force-noise': cmd_force_noise}[a.what](r, a)
    finally:
        r.disconnect()


if __name__ == '__main__':
    main()
