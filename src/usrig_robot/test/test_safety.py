import numpy as np
import pytest

from usrig_common import config
from usrig_robot import safety

RIG = {
    'robot': {'ip': '127.0.0.1', 'control_backend': 'rtde', 'tcp_setting': [0, 0, 0.2, 0, 0, 0],
              'payload_kg': 0.5, 'payload_cog_m': [0, 0, 0.05]},
    'limits': {'max_contact_force_n': 20.0, 'force_band_n': [2.0, 10.0], 'tilt_cone_deg': 30.0,
               'tilt_reference_axis_base': [0, 0, -1],
               'workspace_box_base_m': {'min': [-0.5, -0.5, 0.0], 'max': [0.5, 0.5, 0.5]},
               'max_linear_speed_mm_s': 50.0, 'max_angular_speed_deg_s': 20.0,
               'max_linear_accel_mm_s2': 500.0, 'max_angular_accel_deg_s2': 200.0, 'stop_decel_mm_s2': 1000.0,
               'deadman_timeout_s': 0.2, 'contact_detect_threshold_n': 1.5,
               'contact_lost_threshold_n': 0.5, 'contact_lost_time_s': 0.1,
               'force_setpoint_default_n': 5.0, 'local_tilt_cone_deg': 10.0},
    'motion': {'transit_speed_mm_s': 30.0, 'descent_speed_mm_s': 5.0,
               'descent_max_travel_mm': 50.0, 'retract_distance_mm': 50.0},
    'probe': {'tcp_already_includes_probe': True},
}


def probe_down(p=(0, 0, 0.2)):
    T = np.eye(4)
    T[:3, :3] = np.diag([1, -1, -1])   # probe z points down (base -z)
    T[:3, 3] = p
    return T


def lim():
    return safety.Limits.from_rig(RIG)


def test_null_limit_refuses():
    rig = {**RIG, 'limits': {**RIG['limits'], 'tilt_cone_deg': None}}
    with pytest.raises(config.UnsetLimitError, match='limits.tilt_cone_deg'):
        safety.Limits.from_rig(rig)


@pytest.mark.parametrize('case,flag', [
    ('box', safety.FLAG_ENVELOPE), ('tilt', safety.FLAG_TILT_CONE),
    ('force', safety.FLAG_FORCE_MAX), ('speed', safety.FLAG_SPEED)])
def test_every_hard_limit_trips(case, flag):
    T, wrench, speed = probe_down(), np.zeros(6), np.zeros(6)
    if case == 'box':
        T[:3, 3] = [0.6, 0, 0.2]
    if case == 'tilt':
        from scipy.spatial.transform import Rotation
        T[:3, :3] = Rotation.from_euler('x', 40, degrees=True).as_matrix() @ T[:3, :3]
    if case == 'force':
        wrench[2] = 25.0
    if case == 'speed':
        speed[0] = 0.2
    assert safety.hard_violations(lim(), T, wrench, speed) == flag


def test_filter_clamps_speed_and_box():
    L = lim()
    T = probe_down((0.49, 0, 0.2))
    v, w, f = safety.filter_command(L, T, [1.0, 0, 0], [0, 0, 0], [1.0, 0, 0], [0, 0, 0], 0.008)
    assert v[0] == 0.0 and f & safety.FLAG_ENVELOPE
    T = probe_down()
    v, w, f = safety.filter_command(L, T, [1.0, 0, 0], [0, 0, 0], [0.05, 0, 0], [0, 0, 0], 0.008)
    assert np.linalg.norm(v) == pytest.approx(0.05) and f & safety.FLAG_SPEED


def test_filter_limits_accel():
    L = lim()
    v, w, _ = safety.filter_command(L, probe_down(), [0.04, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], 0.008)
    assert np.linalg.norm(v) == pytest.approx(0.5 * 0.008)


def test_filter_blocks_tilt_increase_but_allows_probe_axis_rotation():
    from scipy.spatial.transform import Rotation
    L = lim()
    T = probe_down()
    T[:3, :3] = Rotation.from_euler('x', 29.9, degrees=True).as_matrix() @ T[:3, :3]
    w_in = np.array([0.3, 0, 0])               # about base x: increases tilt
    v, w, f = safety.filter_command(L, T, np.zeros(3), w_in, np.zeros(3), w_in, 0.008)
    assert f & safety.FLAG_TILT_CONE
    assert abs(np.cross(w, T[:3, 2])).max() < 1e-12   # only rotation about the probe axis remains
