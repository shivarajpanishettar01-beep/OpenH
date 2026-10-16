"""Safety envelope: pure functions, no ROS, so every limit can be unit-tested.

Hard limits (violation -> FAULT, stop, acknowledgement required):
  workspace box, tilt cone, max contact force, measured speed above the limit.
Command filtering (never FAULT by itself):
  speed and acceleration clamping; motion that would leave the box or the tilt cone within
  ``lookahead_s`` is removed from the command and FLAG_ENVELOPE / FLAG_TILT_CONE set.
"""
from dataclasses import dataclass

import numpy as np

from usrig_common import config, geometry

FLAG_ENVELOPE = 1
FLAG_TILT_CONE = 2
FLAG_FORCE_MAX = 4
FLAG_DEADMAN = 8
FLAG_SPEED = 16

FLAG_NAMES = {FLAG_ENVELOPE: 'envelope', FLAG_TILT_CONE: 'tilt_cone', FLAG_FORCE_MAX: 'force_max',
              FLAG_DEADMAN: 'deadman', FLAG_SPEED: 'speed'}


# UR RTDE robot_mode / safety_mode values (ur_rtde RTDEReceiveInterface getRobotMode / getSafetyMode)
ROBOT_MODE_RUNNING = 7
SAFETY_MODES_OK = (1, 2)            # NORMAL, REDUCED
SAFETY_MODE_NAMES = {3: 'protective stop', 4: 'recovery mode', 5: 'safeguard stop',
                     6: 'system emergency stop', 7: 'robot emergency stop', 8: 'safety violation',
                     9: 'safety fault', 10: 'validate joint id', 11: 'undefined safety mode',
                     12: 'automatic-mode safeguard stop', 13: 'three-position enabling stop'}


def controller_problem(robot_mode, safety_mode):
    """Why the controller cannot execute commands now (None = fine, or not reported by the source)."""
    if safety_mode is not None and safety_mode not in SAFETY_MODES_OK:
        return f'controller in {SAFETY_MODE_NAMES.get(safety_mode, f"safety mode {safety_mode}")}'
    if robot_mode is not None and robot_mode != ROBOT_MODE_RUNNING:
        return f'robot not running (robot_mode {robot_mode}; 7 = RUNNING)'
    return None


def flag_names(flags):
    return [n for b, n in FLAG_NAMES.items() if flags & b]


@dataclass
class Limits:
    box_min: np.ndarray
    box_max: np.ndarray
    tilt_cone_deg: float
    tilt_ref_axis: np.ndarray
    max_force_n: float
    force_band_n: tuple
    v_max: float           # m/s
    w_max: float           # rad/s
    a_max: float           # m/s^2
    alpha_max: float       # rad/s^2
    deadman_s: float
    contact_threshold_n: float
    contact_lost_n: float = 0.0
    contact_lost_s: float = 0.0
    stop_decel: float = 2.0       # m/s^2
    measured_speed_margin: float = 1.2
    lookahead_s: float = 0.1
    local_cone_deg: float = None   # D10 cone around a waypoint axis (set at run time)
    local_axis: np.ndarray = None
    extra_boxes: list = None       # D9: [(T_base_frame, min, max)] e.g. the phantom box

    @classmethod
    def from_rig(cls, rig):
        config.require_limits(rig)
        g = lambda k: config.get_path(rig, k)  # noqa: E731
        return cls(
            box_min=np.asarray(g('limits.workspace_box_base_m.min'), float),
            box_max=np.asarray(g('limits.workspace_box_base_m.max'), float),
            tilt_cone_deg=float(g('limits.tilt_cone_deg')),
            tilt_ref_axis=np.asarray(g('limits.tilt_reference_axis_base'), float),
            max_force_n=float(g('limits.max_contact_force_n')),
            force_band_n=tuple(float(x) for x in g('limits.force_band_n')),
            v_max=float(g('limits.max_linear_speed_mm_s')) / 1000.0,
            w_max=np.radians(float(g('limits.max_angular_speed_deg_s'))),
            a_max=float(g('limits.max_linear_accel_mm_s2')) / 1000.0,
            alpha_max=np.radians(float(g('limits.max_angular_accel_deg_s2'))),
            deadman_s=float(g('limits.deadman_timeout_s')),
            contact_threshold_n=float(g('limits.contact_detect_threshold_n')),
            contact_lost_n=float(g('limits.contact_lost_threshold_n')),
            contact_lost_s=float(g('limits.contact_lost_time_s')),
            stop_decel=float(g('limits.stop_decel_mm_s2')) / 1000.0,
            measured_speed_margin=float(g('limits.measured_speed_margin') or 1.2),
            lookahead_s=float(g('limits.envelope_lookahead_s') or 0.1),
            local_cone_deg=float(g('limits.local_tilt_cone_deg')),
            extra_boxes=[],
        )


def probe_axis_force(T_base_probe, wrench_base):
    """Force along the probe beam axis (+z, into tissue), from a base-frame wrench."""
    f = np.asarray(wrench_base[:3], float)
    return float(T_base_probe[:3, 2] @ f)


def pose_violations(lim: Limits, T):
    """Flags for a pose outside the workspace box(es) or the global tilt cone."""
    flags = 0
    p = T[:3, 3]
    if np.any(p < lim.box_min) or np.any(p > lim.box_max):
        flags |= FLAG_ENVELOPE
    for T_bf, bmin, bmax in lim.extra_boxes or []:
        pf = geometry.inv(T_bf)[:3, :3] @ p + geometry.inv(T_bf)[:3, 3]
        if np.any(pf < bmin) or np.any(pf > bmax):
            flags |= FLAG_ENVELOPE
    if geometry.tilt_deg(T, lim.tilt_ref_axis) > lim.tilt_cone_deg:
        flags |= FLAG_TILT_CONE
    return flags


def interpolate_poses(T0, T1, n):
    """n poses from T0 to T1 (both included): position linear, orientation slerp, as LinearMove moves."""
    from scipy.spatial.transform import Rotation, Slerp
    sl = Slerp([0.0, 1.0], Rotation.from_matrix([T0[:3, :3], T1[:3, :3]]))
    out = []
    for u in np.linspace(0.0, 1.0, max(int(n), 2)):
        T = np.eye(4)
        T[:3, :3] = sl([u]).as_matrix()[0]
        T[:3, 3] = (1.0 - u) * T0[:3, 3] + u * T1[:3, 3]
        out.append(T)
    return out


def local_cone_violated(lim: Limits, T, tol_deg=0.05):
    """True if T is outside the local tilt cone (D10) around the current waypoint axis, when one is set.
    Not a hard limit (the command filter enforces it), so it is checked only for planned motion."""
    if lim.local_axis is None or lim.local_cone_deg is None:
        return False
    return geometry.tilt_deg(T, lim.local_axis) > lim.local_cone_deg + tol_deg


def check_targets(lim: Limits, poses, allowed_flags=0, local_cone=False):
    """Check planned poses before any motion starts. ``allowed_flags``: violations the robot already
    has at the start (a recovery move back inside), tolerated on the way but never at the last pose.
    ``local_cone``: also refuse poses outside the local tilt cone (D10), which the command filter would
    otherwise clip silently during the motion (a scripted fan that never reaches its end angles).
    Returns None if fine, else a message naming the first bad pose."""
    poses = list(poses)
    for i, T in enumerate(poses):
        f = pose_violations(lim, T)
        if local_cone and local_cone_violated(lim, T):
            f |= FLAG_TILT_CONE
        if i == len(poses) - 1:
            bad = f
        else:
            bad = f & ~allowed_flags
        if bad:
            p = np.round(T[:3, 3] * 1000.0, 1).tolist()
            return (f'planned pose {i + 1}/{len(poses)} at {p} mm (base) is outside the limits '
                    f'{flag_names(bad)}: refused before moving')
    return None


def hard_violations(lim: Limits, T, wrench_base, tcp_speed):
    """Flags for limits that are actually violated by the measured state."""
    flags = pose_violations(lim, T)
    if np.linalg.norm(np.asarray(wrench_base[:3], float)) > lim.max_force_n:
        flags |= FLAG_FORCE_MAX
    v = np.linalg.norm(tcp_speed[:3])
    w = np.linalg.norm(tcp_speed[3:6])
    if v > lim.v_max * lim.measured_speed_margin or w > lim.w_max * lim.measured_speed_margin:
        flags |= FLAG_SPEED
    return flags


def _clamp_norm(x, n):
    m = np.linalg.norm(x)
    return x if m <= n or m == 0 else x * (n / m)


def _clip_in_frame(p, v, bmin, bmax, lookahead_s):
    """Zero the components of v (in the box's frame) that would carry p out of [bmin, bmax]."""
    v = v.copy()
    clipped = False
    nxt = p + v * lookahead_s
    for i in range(3):
        if (nxt[i] < bmin[i] and v[i] < 0) or (nxt[i] > bmax[i] and v[i] > 0):
            v[i] = 0.0
            clipped = True
    return v, clipped


def _clip_boxes(lim: Limits, T, v, max_passes=4):
    """(v, flags): v with every outward component removed for the base box and all extra boxes."""
    p = T[:3, 3]
    frames = [(np.eye(4), lim.box_min, lim.box_max)] + list(lim.extra_boxes or [])
    flags = 0
    for _ in range(max_passes):
        any_clip = False
        for T_bf, bmin, bmax in frames:
            R = T_bf[:3, :3]
            vf, clipped = _clip_in_frame(R.T @ (p - T_bf[:3, 3]), R.T @ v, bmin, bmax, lim.lookahead_s)
            if clipped:
                v = R @ vf
                any_clip = True
                flags |= FLAG_ENVELOPE
        if not any_clip:
            return v, flags
    return np.zeros(3), flags | FLAG_ENVELOPE     # the boxes keep re-clipping each other: do not move


def filter_command(lim: Limits, T, v, w, v_prev, w_prev, dt):
    """Clamp a base-frame twist to the speed, acceleration, envelope and tilt limits.

    Speed and acceleration are limited first; envelope and tilt clipping come last so that
    they always win (a blocked direction is never re-introduced by acceleration smoothing).
    Returns (v, w, flags). Flags here mean "limited", not "violated".
    """
    flags = 0
    v = np.asarray(v, float)
    w = np.asarray(w, float)

    v_c = _clamp_norm(v, lim.v_max)
    w_c = _clamp_norm(w, lim.w_max)
    if not np.allclose(v_c, v) or not np.allclose(w_c, w):
        flags |= FLAG_SPEED
    v_prev = np.asarray(v_prev, float)
    w_prev = np.asarray(w_prev, float)
    v = v_prev + _clamp_norm(v_c - v_prev, lim.a_max * dt)
    w = w_prev + _clamp_norm(w_c - w_prev, lim.alpha_max * dt)

    # Workspace box and extra boxes (D9, e.g. the phantom box, rotated relative to base): drop velocity
    # components that would cross a face within the lookahead. Clipping in a rotated frame can put back a
    # component the base box had removed, so repeat until no box clips; if they still disagree, stop.
    v, box_flags = _clip_boxes(lim, T, v)
    flags |= box_flags

    # Tilt cones: refuse rotation that increases tilt beyond the global cone, or beyond the local
    # cone around the current waypoint axis (D10) when one is set.
    if np.linalg.norm(w) > 0:
        from scipy.spatial.transform import Rotation
        T_next = np.eye(4)
        T_next[:3, :3] = Rotation.from_rotvec(w * lim.lookahead_s).as_matrix() @ T[:3, :3]
        cones = [(lim.tilt_ref_axis, lim.tilt_cone_deg)]
        if lim.local_axis is not None and lim.local_cone_deg is not None:
            cones.append((lim.local_axis, lim.local_cone_deg))
        for axis, cone in cones:
            now = geometry.tilt_deg(T, axis)
            nxt = geometry.tilt_deg(T_next, axis)
            if nxt > cone and nxt > now:
                z = T[:3, 2]
                w = z * (w @ z)        # keep only rotation about the probe axis (does not change tilt)
                flags |= FLAG_TILT_CONE
                break
    return v, w, flags
