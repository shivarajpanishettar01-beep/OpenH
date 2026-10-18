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


def step_flags(t_steps_ns, t_start_ns, t_end_ns, pauses_ns):
    """(in_episode, paused, action_valid) bool arrays. The converter keeps the whole bag: steps before the start
    event (readiness, free-air hold, descent) and after the end event are outside the episode. paused: inside an
    operator pause (episode.json pauses_ns). action_valid: the action from t to t+1 joins two in-episode,
    unpaused steps (the last step has no successor: its zero action is never valid)."""
    t = np.asarray(t_steps_ns, np.int64)
    inside = (t >= int(t_start_ns)) & (t < int(t_end_ns))
    paused = np.zeros(len(t), bool)
    for t0, t1 in pauses_ns or []:
        paused |= (t >= int(t0)) & (t < int(t1))
    ok = inside & ~paused
    valid = np.zeros(len(t), bool)
    valid[:-1] = ok[:-1] & ok[1:]
    return inside, paused, valid


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


GENERIC_TASKS = {'scan the phantom', 'scan', 'find the view', 'find the view.', 'shakedown', ''}


def task_problems(episodes):
    """[episode.json] -> one message per episode whose task text is missing or generic, or has no target.
    Open-H: per-episode task text states task and target, never a generic label. A staged ambiguous-instruction
    failure (FM-12) is generic on purpose."""
    out = []
    for e in episodes:
        instr = (e.get('instruction') or '').strip()
        why = []
        if e.get('failure_mode') != 'FM-12' and instr.lower() in GENERIC_TASKS:
            why.append(f'task text {instr!r} is missing or generic')
        if not (e.get('target') or '').strip():
            why.append('no target')
        if why:
            out.append(f'{e.get("episode_id")}: ' + ', '.join(why) + ' (set the task in the console before Start)')
    return out


# exported name -> (rig.yaml / session snapshot key, exact frame definition)
CALIBRATIONS = {
    'wrist_cali_mtx': ('wrist_cam_cali_mtx',
                       'T_flange_camera: 4x4 pose of the wrist D415 colour optical frame in the UR flange (tool0) '
                       'frame, metres (eye-in-hand hand-eye, usrig_episode handeye_calib). Not the TCP / probe '
                       'tip: compose with robot_tcp_setting (provenance.jsonl) for the TCP frame.'),
    'prob_cali_mtx': ('prob_cali_mtx',
                      'ultrasound image to probe calibration as given in calibration.prob_cali_mtx; source and '
                      'target frames as defined by the lab calibration file (not re-derived here)'),
}


def session_calibration(sess, rig_cal):
    """{exported name: (matrix, source)} for one session: the session's own snapshot wins (session.json
    `calibration.<key>`, else a top-level `<key>` written at session start); the exporting PC's rig.yaml is only a
    labelled fallback. (None, None) when neither has it."""
    snap = sess.get('calibration') or {}
    out = {}
    for name, (key, _) in CALIBRATIONS.items():
        if snap.get(key) is not None:
            out[name] = (snap[key], f'session.json:calibration.{key}')
        elif sess.get(key) is not None:
            out[name] = (sess[key], f'session.json:{key}')
        elif (rig_cal or {}).get(key) is not None:
            out[name] = (rig_cal[key], 'rig.yaml of the exporting PC (no session snapshot)')
        else:
            out[name] = (None, None)
    return out


def calibration_meta(per_session, name):
    """meta/calibration/<name>.json: the exact frame, every session's matrix and source, and the matrix itself
    at the top level only when all sessions agree (else None: use per_session)."""
    rows = {sid: c[name] for sid, c in sorted(per_session.items())}
    distinct = {json.dumps(m) for m, _ in rows.values()}
    top = next(iter(rows.values()))[0] if len(distinct) == 1 and rows else None
    return {name: top, 'frame': CALIBRATIONS[name][1],
            'per_session': {sid: {'matrix': m, 'source': src} for sid, (m, src) in rows.items()}}


def index_problems(order_episodes, sources, parquet_indices):
    """The v2.1 dataset must hold exactly the staged episodes, in staging order (episode_index == stage index):
    the splits and every per-episode label are keyed on it. Returns one message per mismatch."""
    out = []
    by_index = {int(s['episode_index']): str(Path(s['bag']).resolve()) for s in sources}
    for o in order_episodes:
        bag = by_index.get(int(o['index']))
        if bag is None:
            out.append(f'staged episode {o["index"]} ({o["bag"]}) is not in the v2.1 dataset')
        elif bag != str(Path(o['bag']).resolve()):
            out.append(f'v2.1 episode {o["index"]} is {bag}, staged as {o["bag"]}')
    staged = {int(o['index']) for o in order_episodes}
    out += [f'v2.1 episode {i} ({by_index[i]}) was not staged' for i in sorted(set(by_index) - staged)]
    if sorted(int(i) for i in parquet_indices) != sorted(by_index):
        out.append(f'{len(parquet_indices)} episode parquet files for {len(by_index)} episodes in micrus_sources.jsonl')
    return out


def converter_offsets(lr):
    """({'ultrasound', 'camera'}: s) the converter (bag_to_lerobot, lerobot_rig.yaml) aligned the videos with,
    and problems (the frame info must use the ultrasound image's offset)."""
    im = lr.get('images') or {}
    used = {'ultrasound': float((im.get('observation.images.ultrasound') or {}).get('time_offset_s') or 0.0),
            'camera': float((im.get('observation.images.wrist_camera') or {}).get('time_offset_s') or 0.0)}
    fi = lr.get('micrus_frame_info')
    probs = []
    if fi is not None and abs(float(fi.get('time_offset_s') or 0.0) - used['ultrasound']) > 1e-6:
        probs.append(f'lerobot_rig.yaml micrus_frame_info time_offset_s {fi.get("time_offset_s")} != ultrasound '
                     f'image {used["ultrasound"]}')
    return used, probs


def offset_problems(epj, used, tol_s=0.001):
    """An episode whose capture PC measured offsets (episode.json time_offsets_s) differ from the ones its videos
    were aligned with: its video frames and host stamps would be off by the difference."""
    out = []
    for name, meas in (epj.get('time_offsets_s') or {}).items():
        if name in used and meas is not None and abs(float(meas) - used[name]) > tol_s:
            out.append(f'{epj.get("episode_id")}: {name} offset measured on {epj.get("capture_host")} is {meas} s, '
                       f'the videos were aligned with {used[name]} s (lerobot_rig.yaml)')
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
