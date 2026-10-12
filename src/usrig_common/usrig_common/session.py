"""Session folder layout and session.json read/merge/write."""
import json
import os
import tempfile
from pathlib import Path


def session_dir(session_root, session_id) -> Path:
    return Path(os.path.expandvars(str(session_root))).expanduser() / session_id


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
