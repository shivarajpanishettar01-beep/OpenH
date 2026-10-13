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
    stop_decel: float = 2.0       # m/s^2
    measured_speed_margin: float = 1.2
    lookahead_s: float = 0.1

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
            stop_decel=float(g('limits.stop_decel_mm_s2')) / 1000.0,
            measured_speed_margin=float(g('limits.measured_speed_margin') or 1.2),
            lookahead_s=float(g('limits.envelope_lookahead_s') or 0.1),
        )


def probe_axis_force(T_base_probe, wrench_base):
    """Force along the probe beam axis (+z, into tissue), from a base-frame wrench."""
    f = np.asarray(wrench_base[:3], float)
    return float(T_base_probe[:3, 2] @ f)


def hard_violations(lim: Limits, T, wrench_base, tcp_speed):
    """Flags for limits that are actually violated by the measured state."""
    flags = 0
    p = T[:3, 3]
    if np.any(p < lim.box_min) or np.any(p > lim.box_max):
        flags |= FLAG_ENVELOPE
    if geometry.tilt_deg(T, lim.tilt_ref_axis) > lim.tilt_cone_deg:
        flags |= FLAG_TILT_CONE
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

    # Workspace box: drop velocity components that would cross a face within the lookahead.
    p_next = T[:3, 3] + v * lim.lookahead_s
    for i in range(3):
        if (p_next[i] < lim.box_min[i] and v[i] < 0) or (p_next[i] > lim.box_max[i] and v[i] > 0):
            v[i] = 0.0
            flags |= FLAG_ENVELOPE

    # Tilt cone: refuse rotation that increases tilt beyond the cone.
    if np.linalg.norm(w) > 0:
        from scipy.spatial.transform import Rotation
        T_next = np.eye(4)
        T_next[:3, :3] = Rotation.from_rotvec(w * lim.lookahead_s).as_matrix() @ T[:3, :3]
        now = geometry.tilt_deg(T, lim.tilt_ref_axis)
        nxt = geometry.tilt_deg(T_next, lim.tilt_ref_axis)
        if nxt > lim.tilt_cone_deg and nxt > now:
            z = T[:3, 2]
            w = z * (w @ z)        # keep only rotation about the probe axis (does not change tilt)
            flags |= FLAG_TILT_CONE
    return v, w, flags
