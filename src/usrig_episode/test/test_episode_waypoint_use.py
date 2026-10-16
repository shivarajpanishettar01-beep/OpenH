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
                              scfg={'session': {'presession_max_age_h': 12, 'min_free_disk_gb': 0}}, presession_problem=None,
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


def test_start_refused_while_robot_state_uses_another_registration(tmp_path, monkeypatch):
    """A registration made after the launch left /robot/state.tcp_phantom = the base pose until a restart."""
    from usrig_common import registration_sync
    T = [[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    fake = _fake_manager(tmp_path, {'phantom_registration': {'T_base_phantom': T}})
    fake.sim = True
    monkeypatch.setattr(registration_sync, 'matches', lambda node, T: (False, 'robot_state has no phantom registration'))
    assert EpisodeManager._presession_ready(fake) is False and 'register_phantom' in fake.presession_problem
    monkeypatch.setattr(registration_sync, 'matches', lambda node, T: (True, 'ok'))
    assert EpisodeManager._presession_ready(fake) is True
    monkeypatch.setattr(registration_sync, 'matches', lambda node, T: (None, 'not running'))
    assert EpisodeManager._presession_ready(fake) is True          # no robot_state: the bag readiness gate stops it


def test_plan_progress_spans_sessions(tmp_path):
    """Plan progress was per session: each new session (new id daily) restarted the plan at row 0."""
    import json as _j
    import types as _t
    from usrig_common import gates
    root = tmp_path / 'data'

    def ep(session, row, outcome='success', qc=True, sha='abc'):
        d = root / 'sessions' / session / f'ep_{row:04d}'
        d.mkdir(parents=True)
        (d / 'episode.json').write_text(_j.dumps({'plan_sha256': sha, 'plan_row': row, 'outcome': outcome,
                                                  'qc': {'pass': qc}}))
    ep('S1', 1)
    ep('S1', 2, qc=False)                    # failed QC: redo
    ep('S2', 3, outcome='discard')
    ep('S2', 4, sha='other')                 # another plan
    assert gates.plan_rows_done(root, 'abc') == {1}
    sdir = root / 'sessions' / 'S3'
    sdir.mkdir(parents=True)
    fake = _t.SimpleNamespace(root=root, sdir=sdir)
    rows = [{'row': str(i)} for i in range(1, 6)]
    assert [r['row'] for r in EpisodeManager._pending_rows(fake, rows, 'abc')] == ['2', '3', '4', '5']


def test_micrus_stop_failure_keeps_the_episode(tmp_path):
    """A failed /micrus/record false raised out of _finish before episode.json was written: a complete episode
    was lost and the stage thread died. The error is now recorded, and the lock file is kept so record_guard
    still stops micrus."""
    import types as _t
    from usrig_episode.recorder import EpisodeRecorder, RecorderError
    lock = tmp_path / 'lock'
    lock.write_text('1')

    def fail(on, timeout=10.0):
        raise RecorderError('/micrus/record False timed out')
    fake = _t.SimpleNamespace(_stop_bag=lambda: 0, recording_micrus=True, _call_micrus=fail,
                              lock_file=lock, hdf5_path='/x.h5', micrus_stop_error=None)
    rc, h5 = EpisodeRecorder.stop(fake)
    assert rc == 0 and h5 == '/x.h5'
    assert 'timed out' in fake.micrus_stop_error and lock.exists() and fake.recording_micrus is False


def test_stage_thread_error_is_reported(tmp_path):
    """Any non-Abort error ended the stage thread while the console kept showing 'idle'."""
    import types as _t
    logs = []
    fake = _t.SimpleNamespace(collection_mode='robot', stage=0, stage_error=None,
                              recorder=_t.SimpleNamespace(abort=lambda: None),
                              get_logger=lambda: _t.SimpleNamespace(error=logs.append))

    def boom():
        raise OSError('No space left on device')
    fake._wait_presession = boom
    EpisodeManager._run_stage(fake)
    assert 'No space left' in fake.stage_error


def test_start_refused_below_the_disk_floor(tmp_path):
    fake = _fake_manager(tmp_path, {})
    fake.sim = True
    fake.scfg['session']['min_free_disk_gb'] = 10**9
    assert EpisodeManager._presession_ready(fake) is False and 'GB free' in fake.presession_problem


def _post_fake(tmp_path):
    import types as _t
    from usrig_episode import episode_status as es
    calls = []
    f = _t.SimpleNamespace(sdir=tmp_path, finishing=False, scfg={}, rig={}, keymap={}, last_ep=None, counts=None,
                           _finished_episodes=lambda: [], calls=calls,
                           get_logger=lambda: _t.SimpleNamespace(info=calls.append, warn=calls.append))
    f._post_process = lambda ep: EpisodeManager._post_process(f, ep)
    assert es.last_summary                                          # used by _post_process
    return f


def test_discarded_episodes_skip_qc(tmp_path, monkeypatch):
    """QC on a discard (never exported) was pure operator dead time."""
    import json as _j
    import usrig_qc.episode_qc as eq
    monkeypatch.setattr(eq, 'run_episode_qc', lambda *a, **k: (_ for _ in ()).throw(AssertionError('QC ran')))
    (tmp_path / 'ep_0001').mkdir()
    fake = _post_fake(tmp_path)
    ep = EpisodeManager._post_process(fake, {'episode_dir': 'ep_0001', 'episode_id': 'e1', 'outcome': 'discard',
                                             'notes': ''})
    assert ep['qc'] == {'pass': False, 'skipped': 'discarded episode'}
    assert _j.loads((tmp_path / 'ep_0001' / 'episode.json').read_text())['qc']['skipped']


def test_restart_runs_qc_that_never_ran(tmp_path, monkeypatch):
    """A crash between the episode.json write and QC left qc: null for good; it is now run on restart."""
    import json as _j
    import usrig_qc.episode_qc as eq
    import usrig_qc.auto_annotate as aa
    monkeypatch.setattr(eq, 'run_episode_qc', lambda d, ep, s, r: {'pass': True, 'checks': {}})
    monkeypatch.setattr(aa, 'auto_annotate', lambda d, ep, km: None)
    d = tmp_path / 'ep_0002'
    d.mkdir()
    (d / 'episode.json').write_text(_j.dumps({'episode_dir': 'ep_0002', 'episode_id': 'e2', 'outcome': 'success',
                                              'qc': None, 'notes': '', 'duration_s': 25.0}))
    fake = _post_fake(tmp_path)
    EpisodeManager._recover_interrupted(fake)
    assert _j.loads((d / 'episode.json').read_text())['qc']['pass'] is True


def test_session_hand_eye_calibration_is_not_overwritten(tmp_path):
    """Every manager start overwrote session.json wrist_cam_cali_mtx with rig.yaml's (null or older)."""
    import json as _j
    import types as _t
    M = [[1, 0, 0, 0.01], [0, 1, 0, 0], [0, 0, 1, 0.05], [0, 0, 0, 1]]
    (tmp_path / 'session.json').write_text(_j.dumps({'wrist_cam_cali_mtx': M}))
    fake = _t.SimpleNamespace(sdir=tmp_path, rig={'calibration': {'wrist_cam_cali_mtx': None}})
    EpisodeManager._snapshot_calibration(fake)
    assert _j.loads((tmp_path / 'session.json').read_text())['wrist_cam_cali_mtx'] == M
    (tmp_path / 'session.json').write_text('{}')
    fake.rig = {'calibration': {'wrist_cam_cali_mtx': M}}
    EpisodeManager._snapshot_calibration(fake)
    assert _j.loads((tmp_path / 'session.json').read_text())['wrist_cam_cali_mtx'] == M


def test_onset_keys_do_not_overwrite_the_disturbance_stamp(tmp_path):
    """An operator onset key overwrote onset_t but kept onset_source='disturbance stamp', and repeated keys
    kept the last press while the annotation g_t uses the first."""
    import json as _j
    import queue as _q
    import threading as _th
    import types as _t
    from std_msgs.msg import String
    events = []
    ep = {'t_start_ns': 1_000_000_000, 'onset_t': 2.0, 'onset_source': 'disturbance stamp', 'notes': '',
          'view_reached_ns': []}
    fake = _t.SimpleNamespace(ep_lock=_th.Lock(), ep=ep, cmds=_q.Queue(), pause_log=None,
                              event=lambda k, v, *a: events.append((k, v)))
    EpisodeManager._on_cmd(fake, String(data=_j.dumps({'cmd': 'onset'})))
    assert ep['onset_t'] == 2.0 and ep['onset_source'] == 'disturbance stamp' and ep['onset_t_operator'] > 0
    ep2 = {'t_start_ns': 1_000_000_000, 'onset_t': None, 'onset_source': None, 'notes': '', 'view_reached_ns': [],
           'recovery_start_t': None}
    fake.ep = ep2
    EpisodeManager._on_cmd(fake, String(data=_j.dumps({'cmd': 'recovery_start'})))
    first = ep2['recovery_start_t']
    EpisodeManager._on_cmd(fake, String(data=_j.dumps({'cmd': 'recovery_start'})))
    assert ep2['recovery_start_t'] == first                          # first press counts, like the annotation
