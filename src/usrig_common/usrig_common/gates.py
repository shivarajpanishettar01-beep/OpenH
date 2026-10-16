"""Stage gates (section 2.5). Every gate value is null in session.yaml until set by the user;
a gate with a null value never passes. Results are written to session.json.
"""
import json

from usrig_common.config import get_path


def load_episodes(session_root):
    """All episode.json files under <root>/sessions, in name order."""
    from .session import episode_files
    out = []
    for p in episode_files(session_root):
        try:
            out.append(json.loads(p.read_text()))
        except ValueError:
            continue
    return out


def _qc_pass(ep):
    qc = ep.get('qc') or {}
    return qc.get('pass') is True


def plan_rows_done(session_root, plan_sha):
    """Rows of the plan with this SHA-256 that have a kept, QC-passed episode in ANY session under the root
    (a plan spans many sessions; progress kept per session restarted it at row 0 every day)."""
    return {int(e['plan_row']) for e in load_episodes(session_root)
            if e.get('plan_sha256') == plan_sha and e.get('plan_row') is not None
            and e.get('outcome') != 'discard' and _qc_pass(e)}


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
        # teleop shakedown only: freedrive episodes are forced to stage 0 but are not teleop
        if (ep.get('stage') != 0 or ep.get('outcome') == 'discard' or ep.get('gate_excluded')
                or ep.get('collection_mode') == 'freedrive'):
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
    # gate_excluded: {'by', 'reason'} written by a reviewer (e.g. a cable pulled): not counted either way
    st1 = sorted((e for e in episodes if e.get('stage') == 1 and e.get('outcome') != 'discard'
                  and not e.get('gate_excluded')), key=lambda e: e.get('t_start_ns') or 0)
    # distinct paths (one path repeated is not min_paths paths)
    good = {e.get('target') for e in st1
            if (e.get('speed_tracking') or {}).get('max_error_pct') is not None
            and e['speed_tracking']['max_error_pct'] <= max_err}
    # stream gaps judged on the latest min_paths episodes: an old fixed problem must not block Stage 2 forever
    recent = st1[-int(min_paths):] if int(min_paths) > 0 else []
    gaps_ok = bool(recent) and not any(
        ((e.get('qc') or {}).get('checks', {}).get(name) or {}).get('status') == 'fail'
        for e in recent for name in ('micrus_frame_gaps', 'wrist_rate', 'robot_rate', 'robot_sample_gaps',
                                     'stream_coverage'))
    durations = [e.get('duration_s') or 0 for e in st1]
    eph = (len(st1) / (sum(durations) / 3600.0)) if sum(durations) > 0 else None
    return {'passed': len(good) >= int(min_paths) and gaps_ok, 'paths_within_error': len(good),
            'required_paths': min_paths, 'zero_frame_gaps_recent': gaps_ok, 'recent_judged': len(recent),
            'episodes_per_recorded_hour': eph}


def presession_ok(session_json, stage, control, now_ns, max_age_h, camera=None):
    """(ok, reason): a passing pre-session check for this stage, run with --control if the robot will
    be commanded, younger than max_age_h. Written by presession_check into session.json."""
    r = (session_json.get('presession') or {}).get(f'stage{stage}')
    if not r:
        return False, f'no pre-session check for stage {stage} in session.json (run presession_check)'
    if not r.get('pass'):
        failed = [x['check'] for x in r.get('rows', []) if x.get('result') not in ('PASS', 'N/A')]
        return False, f'pre-session check for stage {stage} did not pass: {failed}'
    if control and not r.get('control'):
        return False, 'pre-session check was run without --control but robot_control will command the robot'
    if camera and r.get('camera') is False:
        return False, 'pre-session check was run with --no-camera but the camera is recorded: run it again with the camera'
    age_h = (now_ns - int(r.get('checked_ns', 0))) / 3.6e12
    if age_h > float(max_age_h):
        return False, f'pre-session check is {age_h:.1f} h old (max {max_age_h} h): run it again'
    return True, 'ok'


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
