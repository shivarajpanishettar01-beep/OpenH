"""Pure helpers for the Open-H V2 export (testable without LeRobot or ROS).

Follows the submitted proposal (IIT Mandi, Open-H Embodiment V2):
  * action a_t = [dx, dy, dz, drx, dry, drz]: incremental TCP pose, rotation as a rotation vector
  * per-step status flag g_t in {nominal, failure, recovery}
  * wrist depth kept lossless (16-bit PNG per step, millimetres)
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src' / 'usrig_common'))
from usrig_common import annotation as A  # noqa: E402

ACTION_NAMES = ['dx', 'dy', 'dz', 'drx', 'dry', 'drz']


def delta_actions(tcp_xyzq, frame='base'):
    """tcp_xyzq [n, 7] (x, y, z, qx, qy, qz, qw) -> a [n, 6]: motion from step t to t+1.

    frame 'base': dp in the base frame, rotation vector of R_{t+1} R_t^T (base frame).
    frame 'tool': dp and rotation expressed in the TCP frame at step t (R_t^T dp, R_t^T R_{t+1}).
    The last step has no successor: its action is zero.
    """
    P = np.asarray(tcp_xyzq, float)
    n = len(P)
    a = np.zeros((n, 6))
    if n < 2:
        return a
    R = Rotation.from_quat(P[:, 3:7])
    dp = P[1:, :3] - P[:-1, :3]
    if frame == 'base':
        a[:-1, :3] = dp
        a[:-1, 3:] = (R[1:] * R[:-1].inv()).as_rotvec()
    elif frame == 'tool':
        a[:-1, :3] = R[:-1].inv().apply(dp)
        a[:-1, 3:] = (R[:-1].inv() * R[1:]).as_rotvec()
    else:
        raise ValueError("frame must be 'base' or 'tool'")
    return a


def step_labels(ann, t_steps_ns):
    """Per-step (g_t, phase, gesture) strings from an annotation dict, through the label resolver
    (QA-rejected entries dropped; expert > model_assisted > trainee > automated where they overlap)."""
    r = A.resolve_steps(ann, [int(x) for x in t_steps_ns])
    return r['status'][0], r['phase'][0], r['gesture'][0]


def step_label_sources(ann, t_steps_ns):
    """Per-step provenance of the label that won ('' where no label covers the step)."""
    return {k: v[1] for k, v in A.resolve_steps(ann, [int(x) for x in t_steps_ns]).items()}


def failure_class(epj, ann, catalog=None):
    """(class, provenance) of a failure / recovery episode: the resolved label (merged annotation), else the
    class of the plan's FM id (automated). ('', '') for other episode types or when nothing gives one."""
    if epj.get('type') not in A.FAILURE_TYPES:
        return '', ''
    lab = ann.get('failure_class') or {}
    if lab.get('value') and lab.get('qa_status') != 'rejected':
        return lab['value'], lab.get('provenance') or 'automated'
    fc = A.failure_class_for(epj.get('failure_mode'), catalog)
    return (fc, 'automated') if fc else ('', '')


def failure_class_problems(episodes, catalog=None):
    """[(episode.json, merged annotation)] -> one message per failure / recovery episode without a valid class."""
    out = []
    for epj, ann in episodes:
        if epj.get('type') not in A.FAILURE_TYPES:
            continue
        fc, _ = failure_class(epj, ann, catalog)
        if fc not in A.FAILURE_CLASSES:
            out.append(f'{epj.get("episode_id")}: type {epj.get("type")}, failure_mode {epj.get("failure_mode")}, '
                       f'failure class {fc!r} (needs one of {A.FAILURE_CLASSES}: label it in the annotator '
                       'or map the FM id)')
    return out


def sample_hold(stamps_ns, t_steps_ns):
    """Index of the latest sample stamped at or before each step (-1 if none)."""
    st = np.asarray(stamps_ns, np.int64)
    order = np.argsort(st)
    idx = np.searchsorted(st[order], np.asarray(t_steps_ns, np.int64), side='right') - 1
    return np.where(idx >= 0, order[np.clip(idx, 0, None)], -1)


def png16(depth_u16):
    import cv2
    ok, buf = cv2.imencode('.png', np.ascontiguousarray(depth_u16, dtype=np.uint16))
    if not ok:
        raise RuntimeError('PNG encode failed')
    return buf.tobytes()


def unpng16(b):
    import cv2
    return cv2.imdecode(np.frombuffer(b, np.uint8), cv2.IMREAD_UNCHANGED)


def camera_info_dict(m):
    return {'width': int(m.width), 'height': int(m.height), 'distortion_model': m.distortion_model,
            'K': [float(x) for x in m.k], 'D': [float(x) for x in m.d], 'P': [float(x) for x in m.p],
            'frame_id': m.header.frame_id}


def write_json(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, indent=2) + '\n')
