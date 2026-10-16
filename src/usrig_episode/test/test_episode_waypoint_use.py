"""episode_manager's use of taught waypoints: local tilt cone centre (D10) and descent force (force_n)."""
import types

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from usrig_episode.episode_manager import EpisodeManager


def test_taught_force_is_the_descent_setpoint():
    f = EpisodeManager._taught_force
    assert f({'force_n': 4.2}) == 4.2
    assert f({'force_n': -4.2}) == 4.2            # magnitude: the wrench sign convention is not verified yet
    assert f({'force_n': None}) is None and f({}) is None and f(None) is None


def test_move_centres_local_cone_on_the_waypoint_not_the_start_pose():
    calls = []
    fake = types.SimpleNamespace(call=lambda name, req: calls.append(name),
                                 _set_local_tilt=lambda T: calls.append(('cone', T)))
    T_wp = np.eye(4)
    T_start = np.eye(4)
    T_start[:3, :3] = Rotation.from_euler('x', 10, degrees=True).as_matrix()     # offset start (Stage 2)
    EpisodeManager._move(fake, T_start, cone_T=T_wp)
    assert calls[0] == 'move' and calls[1][0] == 'cone' and np.allclose(calls[1][1], T_wp)
    calls.clear()
    EpisodeManager._move(fake, T_start)                                           # no waypoint: the target
    assert np.allclose(calls[1][1], T_start)
    calls.clear()
    EpisodeManager._move(fake, T_start, frame='base', cone_T=T_wp)                # base-frame moves: no cone
    assert calls == ['move']


def test_path_editor_fan_records_its_centre_waypoint(tmp_path, monkeypatch):
    from usrig_presets import path_editor
    cfg = tmp_path / 'config'
    cfg.mkdir()
    import shutil
    from pathlib import Path
    src = Path(path_editor.__file__).resolve().parents[2] / 'usrig_bringup' / 'config'
    for f in ('session.yaml', 'rig.yaml', 'keymap.yaml'):
        shutil.copy(src / f, cfg / f)
    s = yaml.safe_load((cfg / 'session.yaml').read_text())
    s['session']['root'] = str(tmp_path / 'data')
    s['session']['phantom_id'] = 'ph1'
    (cfg / 'session.yaml').write_text(yaml.safe_dump(s))
    wdir = tmp_path / 'data' / 'waypoints'
    wdir.mkdir(parents=True)
    (wdir / 'ph1.yaml').write_text(yaml.safe_dump({'phantom_id': 'ph1', 'waypoints': [
        {'name': 'liver', 'position_m': [0.0, 0.0, 0.0], 'quaternion_xyzw': [1.0, 0.0, 0.0, 0.0]}]}))
    path_editor.main(['fan', 'liver_fan', '--at', 'liver', '--deg', '8', '--steps', '3', '--config-dir', str(cfg)])
    path_editor.main(['line', 'l1', '--through', 'liver', 'liver', '--config-dir', str(cfg)])
    paths = yaml.safe_load((tmp_path / 'data' / 'paths' / 'ph1.yaml').read_text())['paths']
    assert paths['liver_fan']['center'] == 'liver'
    assert 'center' not in paths['l1']


def _fake_manager(tmp_path, session_json):
    import json as _j
    import types as _t
    (tmp_path / 'session.json').write_text(_j.dumps(session_json))
    logs = []
    log = _t.SimpleNamespace(error=logs.append, warn=logs.append, info=logs.append)
    return _t.SimpleNamespace(sim=False, sdir=tmp_path, stage=0, control=False, camera_recorded=False,
                              scfg={'session': {'presession_max_age_h': 12}}, presession_problem=None,
                              get_logger=lambda: log, logs=logs)


def test_missing_presession_check_blocks_start_instead_of_killing_the_manager(tmp_path):
    """episode_manager used to exit at start-up without a passing check; the check needs the launch's streams
    running, so every new session needed launch -> exit -> check -> relaunch. Now the gate is at Start."""
    import time as _time
    fake = _fake_manager(tmp_path, {})
    assert EpisodeManager._presession_ready(fake) is False
    assert 'no pre-session check' in fake.presession_problem
    fake2 = _fake_manager(tmp_path, {'presession': {'stage0': {'pass': True, 'control': False, 'camera': False,
                                                               'checked_ns': _time.time_ns()}}})
    assert EpisodeManager._presession_ready(fake2) is True and fake2.presession_problem is None
    fake2.sim = True
    (tmp_path / 'session.json').write_text('{}')
    assert EpisodeManager._presession_ready(fake2) is True                    # simulation: no check needed
