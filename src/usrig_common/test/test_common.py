import numpy as np
import pytest

from usrig_common import config, geometry
from usrig_common.legacy_packet import PacketParser, format_command


def test_require_limits_names_every_null():
    rig = {'robot': {'ip': None}, 'limits': {'max_contact_force_n': 10.0,
                                             'workspace_box_base_m': {'min': [0, 0, None],
                                                                      'max': [1, 1, 1]}}}
    with pytest.raises(config.UnsetLimitError) as e:
        config.require_limits(rig, ['robot.ip', 'limits.max_contact_force_n',
                                    'limits.workspace_box_base_m.min', 'limits.missing'])
    assert e.value.missing == ['robot.ip', 'limits.workspace_box_base_m.min', 'limits.missing']


def test_rotvec_roundtrip():
    p6 = [0.1, -0.2, 0.3, 0.2, -1.0, 2.5]
    assert np.allclose(geometry.ur_from_pose(geometry.pose_from_ur(p6)), p6)


def test_kabsch_recovers_transform():
    rng = np.random.default_rng(0)
    T = geometry.pose_from_ur([0.3, 0.1, -0.2, 0.1, 0.4, -0.3])
    src = rng.uniform(-0.1, 0.1, (4, 3))
    dst = (T[:3, :3] @ src.T).T + T[:3, 3]
    T_est, res = geometry.kabsch(src, dst)
    assert np.allclose(T_est, T, atol=1e-9) and res < 1e-9


def test_constant_speed_path():
    a = np.eye(4)
    b = np.eye(4); b[:3, 3] = [0.1, 0, 0]
    c = np.eye(4); c[:3, 3] = [0.1, 0.1, 0]
    path = geometry.ConstantSpeedPath([a, b, c], speed=0.01, accel=0.05)
    assert path.length == pytest.approx(0.2)
    T, v, done = path.sample(path.duration / 2)
    assert v == pytest.approx(0.01) and not done
    T, v, done = path.sample(path.duration + 1)
    assert done and np.allclose(T[:3, 3], [0.1, 0.1, 0])


def test_packet_parser_handles_split_and_garbage():
    p = PacketParser()
    vals = ','.join(str(float(i)) for i in range(18))
    out = p.feed(b'junk[' + vals[:10].encode())
    assert out == []
    out = p.feed(vals[10:].encode() + b'][1,2]')
    assert len(out) == 1 and out[0][17] == 17.0 and p.bad == 1
    assert format_command(0, 0, 0, 0, 0, 0, 1, 1).startswith(b'(')


def test_presession_result_required_for_collection():
    from usrig_common.gates import presession_ok
    H = int(3.6e12)
    good = {'presession': {'stage2': {'pass': True, 'control': True, 'checked_ns': 100 * H, 'rows': []}}}
    assert presession_ok(good, 2, True, 101 * H, 12)[0]
    assert 'no pre-session' in presession_ok({}, 2, True, 101 * H, 12)[1]
    assert 'stage 1' in presession_ok(good, 1, True, 101 * H, 12)[1]
    assert 'old' in presession_ok(good, 2, True, 113 * H, 12)[1]
    no_ctl = {'presession': {'stage2': {**good['presession']['stage2'], 'control': False}}}
    assert '--control' in presession_ok(no_ctl, 2, True, 101 * H, 12)[1]
    assert presession_ok(no_ctl, 2, False, 101 * H, 12)[0]
    failed = {'presession': {'stage2': {'pass': False, 'checked_ns': 100 * H,
                                        'rows': [{'check': 'free disk', 'result': 'FAIL'}, {'check': 'x', 'result': 'PASS'}]}}}
    assert "['free disk']" in presession_ok(failed, 2, False, 101 * H, 12)[1]


def test_motion_check_delta_and_axis_evaluation():
    from scipy.spatial.transform import Rotation
    from usrig_common import motion_check as mc
    T0 = np.eye(4)
    T0[:3, :3] = np.diag([1.0, -1.0, -1.0])            # probe down: probe y = base -y, probe z = base -z
    T1 = T0.copy()
    T1[:3, 3] = [0.0, -0.010, 0.0]                      # 10 mm along probe +y
    d = mc.pose_delta(T0, T1)
    assert np.allclose(d['dp_probe_mm'], [0, 10, 0]) and np.allclose(d['dp_base_mm'], [0, -10, 0])
    ok, rows = mc.evaluate_axis(d, 'slide_across', 10.0)
    assert ok
    ok, rows = mc.evaluate_axis(d, 'slide_across', -10.0)   # wrong sign: FAIL
    assert not ok and not rows[0][1]
    T2 = T0.copy()
    T2[:3, :3] = T0[:3, :3] @ Rotation.from_euler('x', 5, degrees=True).as_matrix()   # tilt about probe x
    ok, rows = mc.evaluate_axis(mc.pose_delta(T0, T2), 'tilt', 5.0)
    assert ok
    T2[:3, 3] = [0.005, 0, 0]                           # rotation that also slides the face: pivot FAIL
    ok, rows = mc.evaluate_axis(mc.pose_delta(T0, T2), 'tilt', 5.0)
    assert not ok and rows[-1][0] == 'pivot at the probe face'


def test_pose_monitor_render():
    from usrig_interfaces.msg import RobotState
    from usrig_robot.pose_monitor import render
    m = RobotState()
    m.mode, m.action_source, m.action_is_expert, m.force_setpoint_n = 7, 5, True, float('nan')
    m.tcp_phantom.orientation.w = 1.0
    T = np.eye(4)
    T2 = T.copy()
    T2[0, 3] = 0.012
    out = render(m, T2, T, 3.0, 0.1)
    assert 'FREEDRIVE' in out and 'kinesthetic' in out and 'setpoint none' in out and '+12.00' in out
    assert 'STALE' in render(m, T2, T, 0.0, 2.0)
