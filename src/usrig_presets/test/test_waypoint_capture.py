"""capture_waypoint (shared by waypoint_teach and the operator console) with a fake node."""
import json
import types

import numpy as np
import pytest
import yaml

from usrig_common import geometry
from usrig_presets import waypoints as W


def fake_node(tmp_path):
    reg = np.eye(4)
    reg[:3, 3] = [0.4, 0.0, 0.0]
    sdir = tmp_path / 'sessions' / 's1'
    sdir.mkdir(parents=True)
    (sdir / 'session.json').write_text(json.dumps({'phantom_registration': {'T_base_phantom': reg.tolist()}}))
    T = np.eye(4)
    T[:3, :3] = np.diag([1.0, -1.0, -1.0])                  # probe z down
    T[:3, 3] = [0.42, 0.01, 0.03]
    state = types.SimpleNamespace(wrench=[0.0, 0.0, -4.0, 0, 0, 0])
    node = types.SimpleNamespace(
        scfg={'session': {'phantom_id': 'ph', 'root': str(tmp_path), 'sim_root': str(tmp_path / 'sim')}},
        rig={'motion': {'approach_offset_mm': 50}, 'robot': {'tcp_setting': [0, 0, 0.1, 0, 0, 0]}, 'probe': {'type': 'L'}},
        tip_pose_base=lambda: T, state=lambda: state, create_client=lambda *a: None,
        call=lambda c, r: types.SimpleNamespace(success=True, message=str(snap)))
    snap = tmp_path / 'snap_0001.png'
    snap.write_bytes(b'png')
    return node, sdir, T


def test_capture_saves_phantom_frame_pose_force_and_approach(tmp_path):
    node, sdir, T = fake_node(tmp_path)
    wp, f = W.capture_waypoint(node, sdir, 'femoral', 'vein', 'long axis', ['a', 'b', 'c'],
                               json.dumps({'settings': {'gain': 50}}))
    assert f == tmp_path / 'waypoints' / 'ph.yaml'
    data = yaml.safe_load(f.read_text())
    assert [w['name'] for w in data['waypoints']] == ['femoral']
    assert np.allclose(wp['position_m'], [0.02, 0.01, 0.03])          # base -> phantom frame
    assert wp['approach_position_m'][2] == pytest.approx(0.08)        # 50 mm back along probe z (down)
    assert wp['force_n'] == pytest.approx(4.0) and wp['preset'] == {'gain': 50} and wp['verified'] is False
    wp2, _ = W.capture_waypoint(node, sdir, 'femoral', 'vein', 'long', ['a', 'b', 'c', 'd'])
    assert len(yaml.safe_load(f.read_text())['waypoints']) == 1      # same name replaces
    assert W.list_waypoints(node.scfg, sdir)[0]['view'] == 'long'


def test_capture_refuses_bad_input(tmp_path):
    node, sdir, _ = fake_node(tmp_path)
    with pytest.raises(ValueError, match='3-5'):
        W.capture_waypoint(node, sdir, 'x', 'o', 'v', ['only one'])
    with pytest.raises(ValueError, match='empty'):
        W.capture_waypoint(node, sdir, ' ', 'o', 'v', ['a', 'b', 'c'])


def test_capture_refuses_without_a_reference_snapshot(tmp_path):
    """A failed snapshot saved the waypoint with reference_snapshot None: it could never be verified."""
    node, sdir, _ = fake_node(tmp_path)
    node.call = lambda c, r: types.SimpleNamespace(success=False, message='no scanner')
    with pytest.raises(RuntimeError, match='snapshot'):
        W.capture_waypoint(node, sdir, 'femoral', 'vein', 'long', ['a', 'b', 'c'])
    assert W.list_waypoints(node.scfg, sdir) == []


def test_failed_verification_unverifies_the_waypoint(tmp_path):
    """A move / descend failure raised before saving: an unreachable waypoint stayed verified (and a batch
    verify stopped at the first bad waypoint)."""
    node, sdir, _ = fake_node(tmp_path)
    W.capture_waypoint(node, sdir, 'femoral', 'vein', 'long', ['a', 'b', 'c'])
    f = tmp_path / 'waypoints' / 'ph.yaml'
    data = yaml.safe_load(f.read_text())
    data['waypoints'][0]['verified'] = True
    f.write_text(yaml.safe_dump(data))
    node.rig['waypoint_verify_min_ncc'] = 0.5
    calls = []

    def call(client, req, timeout=30.0):
        calls.append(client)
        if client == 'move':
            return types.SimpleNamespace(success=False, message='target outside the phantom box')
        return types.SimpleNamespace(success=True, message='')
    node.call = call
    c = types.SimpleNamespace(snap='snap', move='move', descend='descend', retract='retract', zero='zero')
    v = W.verify_waypoint(node, sdir, 'femoral', c)
    assert v['verified'] is False and 'outside the phantom box' in v['error']
    assert yaml.safe_load(f.read_text())['waypoints'][0]['verified'] is False
    assert calls[-1] == 'retract'
