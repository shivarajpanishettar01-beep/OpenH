"""Shared pieces of the real-robot check tools (teleop_check, preset_check, freedrive_check, pose_monitor).

Pure functions (no ROS): pose deltas expressed in the base frame and in the probe frame at the start,
pass / fail evaluation of a single-axis motion, formatting, and the operator confirmation.
"""
import numpy as np
from scipy.spatial.transform import Rotation

# probe-frame axes of /teleop/twist (x along the array, y across, z along the beam into tissue)
AXES = {'vx': (0, False), 'vy': (1, False), 'vz': (2, False), 'wx': (0, True), 'wy': (1, True), 'wz': (2, True)}
ALIASES = {'slide_along': 'vx', 'slide_across': 'vy', 'beam': 'vz', 'tilt': 'wx', 'rock': 'wy', 'rotate': 'wz'}
KEYS = {'vx': 'W/S', 'vy': 'A/D', 'vz': '-', 'wx': 'R/F', 'wy': 'Q/E', 'wz': 'Z/C'}


def axis_name(a):
    a = ALIASES.get(a, a)
    if a not in AXES:
        raise ValueError(f'axis must be one of {sorted(AXES)} or {sorted(ALIASES)}')
    return a


def pose_delta(T0, T1):
    """Motion from T0 to T1. Translation in mm, rotation in deg; '_probe' = expressed in the probe frame at T0."""
    R0, R1 = T0[:3, :3], T1[:3, :3]
    dp = (T1[:3, 3] - T0[:3, 3]) * 1000.0
    rot_base = Rotation.from_matrix(R1 @ R0.T).as_rotvec()
    rot_probe = Rotation.from_matrix(R0.T @ R1).as_rotvec()
    return {'dp_base_mm': dp, 'dp_probe_mm': R0.T @ dp,
            'drot_base_deg': np.degrees(rot_base), 'drot_probe_deg': np.degrees(rot_probe),
            'dist_mm': float(np.linalg.norm(dp)), 'angle_deg': float(np.degrees(np.linalg.norm(rot_base)))}


def evaluate_axis(delta, axis, expected, tol_frac=0.3, off_axis_frac=0.2, pivot_mm=2.0):
    """Check a single-axis probe-frame motion (teleop_check).

    expected: signed distance (mm) for v*, angle (deg) for w*. Pass if the motion along the axis has the
    right sign and is within tol_frac of expected, motion on the other axes is below off_axis_frac of it
    (+ a small floor), and for rotations the TCP (probe face) moves less than pivot_mm (D4: rotation
    about the probe face). Returns (ok, rows) with rows = [(check, ok, detail)]."""
    i, rot = AXES[axis_name(axis)]
    v = delta['drot_probe_deg'] if rot else delta['dp_probe_mm']
    unit = 'deg' if rot else 'mm'
    along = float(v[i])
    others = float(np.linalg.norm(np.delete(v, i)))
    rows = [('direction (sign)', np.sign(along) == np.sign(expected) and abs(along) > 0.1 * abs(expected),
             f'{along:+.2f} {unit} along probe {axis} (expected {expected:+.2f})'),
            ('amount', abs(along - expected) <= tol_frac * abs(expected),
             f'{abs(along) / abs(expected) * 100 if expected else 0:.0f} % of expected (tolerance +-{tol_frac * 100:.0f} %)'),
            ('other axes', others <= off_axis_frac * abs(expected) + (0.5 if not rot else 0.3),
             f'{others:.2f} {unit} on the other {"rotation" if rot else "translation"} axes')]
    if rot:
        rows.append(('pivot at the probe face', delta['dist_mm'] <= pivot_mm,
                     f'TCP moved {delta["dist_mm"]:.2f} mm during the rotation (limit {pivot_mm} mm)'))
    else:
        rows.append(('orientation kept', delta['angle_deg'] <= 1.0, f'{delta["angle_deg"]:.2f} deg rotation'))
    return all(r[1] for r in rows), rows


def fmt_pose(T):
    p = T[:3, 3] * 1000.0
    rv = np.degrees(Rotation.from_matrix(T[:3, :3]).as_rotvec())
    return f'[{p[0]:8.2f} {p[1]:8.2f} {p[2]:8.2f}] mm  rotvec [{rv[0]:7.2f} {rv[1]:7.2f} {rv[2]:7.2f}] deg'


def fmt_delta(d):
    b, p, rp = d['dp_base_mm'], d['dp_probe_mm'], d['drot_probe_deg']
    return (f'base  d[{b[0]:+7.2f} {b[1]:+7.2f} {b[2]:+7.2f}] mm   |d| {d["dist_mm"]:.2f} mm\n'
            f'probe d[{p[0]:+7.2f} {p[1]:+7.2f} {p[2]:+7.2f}] mm   rot [{rp[0]:+6.2f} {rp[1]:+6.2f} {rp[2]:+6.2f}] deg'
            f'   angle {d["angle_deg"]:.2f} deg')


def print_table(title, rows):
    print(f'\n{title}')
    w = max(len(r[0]) for r in rows)
    for name, ok, detail in rows:
        print(f'  {name.ljust(w)}  {"PASS" if ok else "FAIL"}  {detail}')
    ok = all(r[1] for r in rows)
    print(f'  RESULT: {"PASS" if ok else "FAIL"}\n')
    return ok


def confirm(text, ask=input, assume_yes=False):
    """The operator types 'yes' before anything moves (real robot)."""
    print(text)
    if assume_yes:
        print('  (--yes given)')
        return True
    return ask('  Area clear, e-stop in hand, pendant speed slider low? Type yes to move: ').strip().lower() == 'yes'
