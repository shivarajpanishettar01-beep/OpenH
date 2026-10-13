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
