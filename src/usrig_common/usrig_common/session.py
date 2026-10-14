"""Data folder layout and session.json read/merge/write.

Everything the rig writes lives under ONE root: session.root (real) or session.sim_root (simulation),
with the same folders in both, so simulated data can never land in a real folder:

  <root>/sessions/<session_id>/      session.json, plan_progress.json, ep_NNNN/ (bag, episode.json, qc.json,
                                     annotation.json, task.txt)
  <root>/micrus/<session_id>/        micrus --out: lossless HDF5 recordings, snapshots
  <root>/waypoints/<phantom_id>.yaml collection waypoints (phantom frame)
  <root>/paths/<phantom_id>.yaml     Stage 1 paths
  <root>/plans/                      Stage 2 plans (csv)
  <root>/presets/base_presets.yaml   test / transit poses (base frame), preset_check
  <root>/phantoms/<phantom_id>.yaml  fiducials for registration
  <root>/calibration/                raw captures: hand-eye images, pivot poses
  <root>/checks/                     pipeline checks (synthetic data)
Exported datasets are derived data and go elsewhere (e.g. ~/datasets).
"""
import json
import os
import tempfile
from pathlib import Path


LAYOUT = ('sessions', 'micrus', 'waypoints', 'paths', 'plans', 'presets', 'phantoms', 'calibration', 'checks')


def data_root(scfg, sim=False) -> Path:
    """Real sessions under session.root; simulated runs (sim:=true) under session.sim_root, so
    simulated data can never end up in a real dataset."""
    key = 'sim_root' if sim else 'root'
    root = scfg['session'].get(key)
    if sim and not root:
        raise RuntimeError('session.sim_root is not set in session.yaml')
    return Path(os.path.expandvars(str(root))).expanduser()


def check_sim_flag(session_path, sim):
    """Refuse to mix: an existing session keeps the sim flag it was created with."""
    data = read_json(Path(session_path) / 'session.json')
    if 'sim' in data and bool(data['sim']) != bool(sim):
        raise RuntimeError(f'{session_path} was created with sim={data["sim"]}; refusing to add sim={sim} data')


def data_dir(scfg, kind, sim=False) -> Path:
    """<root>/<kind> for real data, <sim_root>/<kind> for simulation (kind in LAYOUT)."""
    if kind not in LAYOUT:
        raise ValueError(f'unknown data folder {kind!r} (one of {LAYOUT})')
    return data_root(scfg, sim) / kind


def session_dir(data_root_path, session_id) -> Path:
    return Path(os.path.expandvars(str(data_root_path))).expanduser() / 'sessions' / session_id


def episode_files(data_root_path):
    """Every <root>/sessions/*/ep_*/episode.json, sorted."""
    return sorted((Path(os.path.expandvars(str(data_root_path))).expanduser() / 'sessions').glob('*/ep_*/episode.json'))


def session_is_sim(session_path) -> bool:
    """The sim flag a session was created with (False if it has no session.json yet)."""
    return bool(read_json(Path(session_path).expanduser() / 'session.json').get('sim', False))


def episode_dir(session_root, session_id, index: int) -> Path:
    return session_dir(session_root, session_id) / f'ep_{index:04d}'


def write_json_atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix='.' + path.name)
    with os.fdopen(fd, 'w') as f:
        json.dump(data, f, indent=2, sort_keys=False, default=str)
        f.write('\n')
    os.replace(tmp, path)


def read_json(path, default=None):
    path = Path(path)
    if not path.exists():
        return {} if default is None else default
    with open(path) as f:
        return json.load(f)


def update_session_json(session_path, **fields):
    p = Path(session_path) / 'session.json'
    data = read_json(p)
    for k, v in fields.items():
        if isinstance(v, dict) and isinstance(data.get(k), dict):
            data[k].update(v)
        else:
            data[k] = v
    write_json_atomic(p, data)
    return data
