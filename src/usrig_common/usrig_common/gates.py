"""Stage gates (section 2.5). Every gate value is null in session.yaml until set by the user;
a gate with a null value never passes. Results are written to session.json.
"""
import json
from pathlib import Path

from usrig_common.config import get_path


def load_episodes(session_root):
    """All episode.json files under the session root, in name order."""
    out = []
    for p in sorted(Path(session_root).expanduser().glob('*/ep_*/episode.json')):
        try:
            out.append(json.loads(p.read_text()))
        except ValueError:
            continue
    return out


def _qc_pass(ep):
    qc = ep.get('qc') or {}
    return qc.get('pass') is True


def gate01(session_cfg, episodes):
    need = get_path(session_cfg, 'gate01.min_consecutive_qc_pass')
    speeds = get_path(session_cfg, 'gate01.key_speeds_chosen')
    band = get_path(session_cfg, 'gate01.force_band_chosen')
    nulls = [k for k, v in (('gate01.min_consecutive_qc_pass', need),
                            ('gate01.key_speeds_chosen', speeds),
                            ('gate01.force_band_chosen', band)) if v is None]
    if nulls:
        return {'passed': False, 'reason': f'null in session.yaml: {nulls}'}
    run = best = 0
    for ep in episodes:
        if ep.get('stage') != 0 or ep.get('outcome') == 'discard':
            continue
        run = run + 1 if _qc_pass(ep) else 0
        best = max(best, run)
    ok = best >= int(need) and bool(speeds) and bool(band)
    return {'passed': ok, 'consecutive_qc_pass': best, 'required': need,
            'key_speeds_chosen': speeds, 'force_band_chosen': band}


def gate12(session_cfg, episodes):
    max_err = get_path(session_cfg, 'gate12.max_speed_error_pct')
    min_paths = get_path(session_cfg, 'gate12.min_paths')
    if max_err is None or min_paths is None:
        return {'passed': False, 'reason': 'gate12.max_speed_error_pct / min_paths null in session.yaml'}
    st1 = [e for e in episodes if e.get('stage') == 1 and e.get('outcome') != 'discard']
    good = 0
    gaps_ok = True
    for e in st1:
        tr = e.get('speed_tracking') or {}
        if tr.get('max_error_pct') is not None and tr['max_error_pct'] <= max_err:
            good += 1
        checks = (e.get('qc') or {}).get('checks', {})
        for name in ('micrus_frame_gaps', 'wrist_rate', 'robot_rate', 'robot_sample_gaps'):
            if checks.get(name, {}).get('status') == 'fail':
                gaps_ok = False
    durations = [e.get('duration_s') or 0 for e in st1]
    eph = (len(st1) / (sum(durations) / 3600.0)) if sum(durations) > 0 else None
    return {'passed': good >= int(min_paths) and gaps_ok, 'paths_within_error': good,
            'required_paths': min_paths, 'zero_frame_gaps': gaps_ok,
            'episodes_per_recorded_hour': eph}


def check_stage(stage, session_cfg, session_root):
    eps = load_episodes(session_root)
    if stage == 0:
        return {'passed': True, 'gate': None}
    if stage == 1:
        return {'gate': '0->1', **gate01(session_cfg, eps)}
    if stage == 2:
        g01 = gate01(session_cfg, eps)
        g12 = gate12(session_cfg, eps)
        return {'gate': '1->2', 'passed': g01['passed'] and g12['passed'], 'gate01': g01, 'gate12': g12}
    raise ValueError(f'unknown stage {stage}')
