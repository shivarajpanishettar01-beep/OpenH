import json
import subprocess
import sys
from pathlib import Path

import pytest

from usrig_common.config import set_thread_rt_priority
from usrig_common.session import check_sim_flag, data_root, update_session_json

REPO = Path(__file__).resolve().parents[3]
SCFG = {'session': {'root': '/data/real', 'sim_root': '/data/sim'}}


def test_sim_and_real_roots_differ():
    assert data_root(SCFG, False) == Path('/data/real')
    assert data_root(SCFG, True) == Path('/data/sim')


def test_session_keeps_its_sim_flag(tmp_path):
    update_session_json(tmp_path, sim=True)
    check_sim_flag(tmp_path, True)
    with pytest.raises(RuntimeError, match='sim=True'):
        check_sim_flag(tmp_path, False)


def test_stage_bags_refuses_sim_sessions(tmp_path):
    s = tmp_path / 'S1'
    (s / 'ep_0001').mkdir(parents=True)
    (s / 'session.json').write_text(json.dumps({'sim': True}))
    (s / 'ep_0001' / 'episode.json').write_text(json.dumps({'sim': True, 'outcome': 'success', 'qc': {'pass': True}}))
    r = subprocess.run([sys.executable, str(REPO / 'export/stage_bags.py'), '--sessions', str(s),
                        '-o', str(tmp_path / 'staging')], capture_output=True, text=True)
    assert r.returncode != 0 and 'never exported' in r.stderr


def test_rt_priority_never_raises():
    ok, msg = set_thread_rt_priority(0)
    assert not ok
    ok, msg = set_thread_rt_priority(1)   # may or may not be permitted on this machine
    assert isinstance(ok, bool) and msg


def _session(tmp_path, name, synthetic):
    s = tmp_path / name
    (s / 'ep_0001').mkdir(parents=True)
    (s / 'session.json').write_text(json.dumps({'sim': False}))
    (s / 'ep_0001' / 'episode.json').write_text(json.dumps({
        'episode_id': 'ep_0001', 'session_id': name, 'stage': 2, 'type': 'success', 'outcome': 'success',
        'qc': {'pass': True}, 'synthetic_pipeline_check': synthetic}))
    return s


@pytest.mark.parametrize('synthetic,flag,ok', [(True, False, False), (False, True, False), (True, True, True),
                                               (False, False, True)])
def test_synthetic_pipeline_check_episodes_never_mix(tmp_path, synthetic, flag, ok):
    s = _session(tmp_path, 'S1', synthetic)
    cmd = [sys.executable, str(REPO / 'export/stage_bags.py'), '--sessions', str(s), '-o', str(tmp_path / 'st')]
    r = subprocess.run(cmd + (['--pipeline-check'] if flag else []), capture_output=True, text=True)
    assert (r.returncode == 0) == ok, r.stderr


def test_data_layout_same_folders_real_and_sim(tmp_path):
    from usrig_common.session import LAYOUT, data_dir, episode_files, session_dir, session_is_sim
    for kind in LAYOUT:
        assert data_dir(SCFG, kind, False) == Path('/data/real') / kind
        assert data_dir(SCFG, kind, True) == Path('/data/sim') / kind
    with pytest.raises(ValueError):
        data_dir(SCFG, 'somewhere', False)
    assert session_dir('/data/real', 'S1') == Path('/data/real/sessions/S1')
    s = session_dir(tmp_path, 'S1')
    (s / 'ep_0001').mkdir(parents=True)
    (s / 'ep_0001' / 'episode.json').write_text('{}')
    (tmp_path / 'waypoints').mkdir()                     # asset folders are never taken for sessions
    (tmp_path / 'waypoints' / 'x.yaml').write_text('{}')
    assert episode_files(tmp_path) == [s / 'ep_0001' / 'episode.json']
    assert not session_is_sim(s)
    update_session_json(s, sim=True)
    assert session_is_sim(s)
