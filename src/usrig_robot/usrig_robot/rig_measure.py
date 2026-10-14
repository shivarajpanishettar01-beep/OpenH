"""Read-only measurements for filling rig.yaml. Never commands the robot: it only reads RTDE.
You move the robot (freedrive or pendant); this tool records what the controller reports.

  ros2 run usrig_robot rig_measure pose                       # current TCP pose, force, modes
  ros2 run usrig_robot rig_measure box   [--write]            # workspace box from touched corner points
  ros2 run usrig_robot rig_measure tilt-axis [--write]        # probe axis with the probe square to the phantom
  ros2 run usrig_robot rig_measure force-noise --seconds 10   # F/T noise in free air (contact threshold input)
  ros2 run usrig_robot rig_measure force-tilt                 # R4: free-air force vs tilt (payload / CoG check)
  add --ip <controller>  (default: rig.yaml robot.ip; URSim: 127.0.0.1)

--write puts the measured value into rig.yaml (same key, comments kept). Margins, padding and
every force limit stay your decision: this tool reports measurements, it does not choose limits.
"""
import argparse
import time
from pathlib import Path

import numpy as np

from usrig_common import config, geometry


def connect(ip, timeout_s=3.0):
    """Read-only RTDE connection. Fails within timeout_s on an unreachable robot instead of blocking
    (a blocked TCP connect takes minutes and ignores Ctrl+C)."""
    import socket
    try:
        socket.create_connection((ip, 30004), timeout=timeout_s).close()
    except OSError as e:
        raise RuntimeError(f'robot {ip}:30004 (RTDE) not reachable within {timeout_s:g} s ({e}): check robot.ip, '
                           'the cable / PC address, and that the controller is booted') from e
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
    """Set a rig.yaml value by its full dotted path (e.g. limits.workspace_box_base_m.min), keeping comments."""
    if isinstance(value, (list, tuple)):
        value = [round(float(v), 5) for v in value]
    config.write_yaml_path(path, key, value)


def cmd_pose(r, a):
    T = tcp(r)
    p6 = geometry.ur_from_pose(T)
    print(f'TCP [m, rotvec rad]: {np.round(p6, 5).tolist()}')
    print(f'probe axis (TCP z) in base: {np.round(T[:3, 2], 4).tolist()}')
    print(f'force/torque: {np.round(r.getActualTCPForce(), 3).tolist()}')
    print(f'robot mode {r.getRobotMode()}  safety mode {r.getSafetyMode()}')


def measure_box(r, ask=input):
    """Box around the TCP points you touch (no padding added). Returns (min, max) in metres."""
    print('Move the probe tip to each extreme of the allowed scanning volume (corners / limits),')
    print('press Enter at each; type q + Enter when done (at least 2 points).')
    pts = []
    while True:
        s = ask(f'[{len(pts)} points] Enter = record, q = done: ').strip().lower()
        if s == 'q':
            break
        Ts, _ = averaged(r)
        p = np.mean([T[:3, 3] for T in Ts], axis=0)
        pts.append(p)
        print(f'  {np.round(p * 1000, 1).tolist()} mm')
    if len(pts) < 2:
        raise ValueError('need at least 2 points')
    P = np.array(pts)
    lo, hi = P.min(0), P.max(0)
    print(f'measured box min {np.round(lo, 4).tolist()} m, max {np.round(hi, 4).tolist()} m '
          f'(size {np.round((hi - lo) * 1000, 1).tolist()} mm). No padding added.')
    return lo, hi


def measure_tilt_axis(r, ask=input):
    ask('Hold the probe square to the phantom surface (as at a normal scan start), press Enter... ')
    Ts, _ = averaged(r, 1.0)
    z = np.mean([T[:3, 2] for T in Ts], axis=0)
    z = z / np.linalg.norm(z)
    print(f'probe axis in base frame: {np.round(z, 4).tolist()}')
    return z


def measure_force_noise(r, seconds, ask=input):
    ask(f'Probe in free air, robot still. Press Enter to record {seconds:.0f} s... ')
    Ts, F = averaged(r, seconds)
    axis = np.array([T[:3, 2] @ f[:3] for T, f in zip(Ts, F)])
    norm = np.linalg.norm(F[:, :3], axis=1)
    out = {'samples': len(F), 'axis_mean_n': float(axis.mean()), 'axis_std_n': float(axis.std()),
           'axis_p2p_n': float(np.ptp(axis)), 'norm_mean_n': float(norm.mean()), 'norm_max_n': float(norm.max())}
    print(f'{len(F)} samples')
    print(f'probe-axis force: mean {out["axis_mean_n"]:.3f} N, std {out["axis_std_n"]:.3f} N, '
          f'peak-to-peak {out["axis_p2p_n"]:.3f} N')
    print(f'|F|: mean {out["norm_mean_n"]:.3f} N, max {out["norm_max_n"]:.3f} N')
    return out


def cmd_box(r, a):
    try:
        lo, hi = measure_box(r)
    except ValueError as e:
        raise SystemExit(str(e))
    if a.write:
        write_key(rig_path(a.config_dir), 'limits.workspace_box_base_m.min', lo.tolist())
        write_key(rig_path(a.config_dir), 'limits.workspace_box_base_m.max', hi.tolist())
        print('written to limits.workspace_box_base_m in rig.yaml')


def cmd_tilt_axis(r, a):
    z = measure_tilt_axis(r)
    if a.write:
        write_key(rig_path(a.config_dir), 'limits.tilt_reference_axis_base', [round(float(x), 4) for x in z])
        print('written to limits.tilt_reference_axis_base in rig.yaml')


def cmd_force_noise(r, a):
    measure_force_noise(r, a.seconds)
    print('Use this noise floor when choosing contact_detect_threshold_n (the choice is yours).')


def tilt_deg(T, ref):
    return float(np.degrees(np.arccos(np.clip(T[:3, 2] @ ref, -1.0, 1.0))))


def force_tilt_table(rows, ref):
    """rows: [(T, mean wrench [6])]. Free-air force should not depend on tilt if the controller's
    payload and CoG are right; returns per-pose tilt, |F| and probe-axis force, and the spread."""
    out = [{'tilt_deg': tilt_deg(T, ref), 'f_norm_n': float(np.linalg.norm(f[:3])), 'f_axis_n': float(T[:3, 2] @ f[:3])}
           for T, f in rows]
    fa = np.array([r['f_axis_n'] for r in out])
    fn = np.array([r['f_norm_n'] for r in out])
    return out, {'f_axis_spread_n': float(np.ptp(fa)) if len(fa) else 0.0, 'f_norm_max_n': float(fn.max()) if len(fn) else 0.0}


def cmd_force_tilt(r, a):
    rig = config.load_yaml(rig_path(a.config_dir))
    ref = config.get_path(rig, 'limits.tilt_reference_axis_base') or [0.0, 0.0, -1.0]
    ref = np.asarray(ref, float) / np.linalg.norm(ref)
    print(f'controller payload: {r.getPayload():.3f} kg, CoG {np.round(r.getPayloadCog(), 4).tolist()} m '
          f'(rig.yaml: {config.get_path(rig, "robot.payload_kg")} kg, {config.get_path(rig, "robot.payload_cog_m")})')
    input('Probe in free air, straight (no tilt), robot still. Zero the F/T sensor on the pendant now, then press Enter... ')
    rows = []
    while True:
        Ts, F = averaged(r, 1.0)
        T = Ts[len(Ts) // 2]
        rows.append((T, F.mean(0)))
        print(f'  tilt {tilt_deg(T, ref):5.1f} deg  |F| {np.linalg.norm(F.mean(0)[:3]):.3f} N  '
              f'probe-axis {T[:3, 2] @ F.mean(0)[:3]:+.3f} N')
        s = input('Tilt the probe (freedrive), hold still, Enter = record, q = done: ').strip().lower()
        if s == 'q':
            break
    _, sm = force_tilt_table(rows, ref)
    print(f'{len(rows)} poses: probe-axis force spread {sm["f_axis_spread_n"]:.3f} N, max |F| {sm["f_norm_max_n"]:.3f} N')
    print('A spread well above the force-noise floor means the payload / CoG on the controller is wrong.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('what', choices=['pose', 'box', 'tilt-axis', 'force-noise', 'force-tilt'])
    ap.add_argument('--ip')
    ap.add_argument('--write', action='store_true')
    ap.add_argument('--seconds', type=float, default=10.0)
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    rig = config.load_yaml(rig_path(a.config_dir))
    ip = a.ip or config.get_path(rig, 'robot.ip')
    if not ip:
        raise SystemExit('give --ip (rig.yaml robot.ip is null)')
    try:
        r = connect(ip)
    except RuntimeError as e:
        raise SystemExit(str(e))
    try:
        {'pose': cmd_pose, 'box': cmd_box, 'tilt-axis': cmd_tilt_axis, 'force-noise': cmd_force_noise,
         'force-tilt': cmd_force_tilt}[a.what](r, a)
    finally:
        r.disconnect()


if __name__ == '__main__':
    main()
