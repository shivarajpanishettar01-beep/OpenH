"""Per-episode QC from the bag's real acquisition stamps (header stamps), never from an exported
dataset (whose timestamp column is frame_index / fps by definition).

  ros2 run usrig_qc episode_qc <session>/ep_0001 [--config-dir DIR]

Writes <ep>/qc.json and returns {'pass': bool, 'checks': {...}}. Status per check:
pass | fail | n/a (does not apply to this stage / episode) | unset (threshold null in config).
An episode passes only if every check is pass or n/a.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from usrig_common import config

ROBOT_SOURCES = {0: 'teleop', 1: 'scripted_task', 2: 'transit', 3: 'perturbation', 4: 'none', 5: 'kinesthetic'}
EXPERT_SOURCES = (0, 1, 5)          # teleop, scripted task, kinesthetic (freedrive by hand)
PLANNED_LIFT = {'FM-01', 'FM-02'}
UNSTAMPED = {'/micrus/frame_info', '/qc/status', '/operator/command'}   # std_msgs/String: no header


def _typestore():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS2_HUMBLE)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('usrig_interfaces')) / 'msg'
    except Exception:
        share = Path(__file__).resolve().parents[2] / 'usrig_interfaces' / 'msg'
    add = {}
    for f in share.glob('*.msg'):
        add.update(get_types_from_msg(f.read_text(), f'usrig_interfaces/msg/{f.stem}'))
    # realsense2_camera_msgs/Metadata (frame metadata as JSON, used for frame-number gaps)
    add.update(get_types_from_msg('std_msgs/Header header\nstring json_data', 'realsense2_camera_msgs/msg/Metadata'))
    ts.register(add)
    return ts


def stamp_ns(msg):
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def read_bag(bag_dir, topics, decode_every=None, window=None, window_stats=None):
    """{topic: [(log_ns, msg), ...]}; decode_every={topic: n} keeps every n-th message only.
    window=(t0, t1) with a window_stats dict: for every stamped topic, {topic: [n, first, last]} of the
    messages whose header stamp (read from the raw CDR, no decoding) is inside the window."""
    from rosbags.highlevel import AnyReader

    from .stream_check import stamp_from_cdr
    out = {t: [] for t in topics}
    counts = {t: 0 for t in topics}
    with AnyReader([Path(bag_dir)], default_typestore=_typestore()) as r:
        conns = [c for c in r.connections if c.topic in topics]
        for c, t, raw in r.messages(connections=conns):
            counts[c.topic] += 1
            if window_stats is not None and window and c.topic not in UNSTAMPED:
                st = stamp_from_cdr(raw)
                if window[0] <= st <= window[1]:
                    w = window_stats.setdefault(c.topic, [0, st, st])
                    w[0], w[1], w[2] = w[0] + 1, min(w[1], st), max(w[2], st)
            n = (decode_every or {}).get(c.topic)
            if n and (counts[c.topic] - 1) % n:
                continue
            out[c.topic].append((t, r.deserialize(raw, c.msgtype)))
    return out, counts


def _status(ok, **kw):
    return {'status': 'pass' if ok else 'fail', **kw}


def run_episode_qc(ep_dir, ep, scfg, rig, write=True):
    ep_dir = Path(ep_dir)
    cam = scfg['wrist_camera']
    ct = cam['topics']
    stage = ep.get('stage', 0)
    topics = ['/micrus/bmode', '/micrus/frame_info', '/robot/state', '/episode/events', '/qc/status']
    use_cam = bool(scfg['recording'].get('include_wrist_camera') and cam.get('enabled'))
    if use_cam:
        topics += [ct['color'], ct['depth']] + ([ct['metadata']] if ct.get('metadata') else [])
    t0, t1 = ep.get('t_start_ns'), ep.get('t_end_ns')
    win = {}
    data, counts = read_bag(ep_dir, topics, decode_every={ct.get('depth'): 5} if use_cam else None,
                            window=(t0, t1) if t0 is not None and t1 is not None else None, window_stats=win)
    checks = {}

    def in_win(ns):
        return t0 is not None and t1 is not None and t0 <= ns <= t1

    # ---- micrus
    fi = [json.loads(m.data) for _, m in data['/micrus/frame_info']]
    idx = np.array([f['frame_index'] for f in fi if 'frame_index' in f])
    gaps = int(np.sum(np.diff(idx) != 1)) if len(idx) > 1 else None
    checks['micrus_frame_gaps'] = _status(gaps == 0, gaps=gaps, frames=int(len(idx)))
    bm_st = np.array([stamp_ns(m) for _, m in data['/micrus/bmode']])
    bm_win = bm_st[[in_win(s) for s in bm_st]] if len(bm_st) else bm_st
    lo, hi = scfg['micrus']['expected_fps']
    fps = (len(bm_win) - 1) / ((bm_win[-1] - bm_win[0]) * 1e-9) if len(bm_win) > 1 else 0.0
    checks['micrus_fps'] = _status(lo <= fps <= hi, fps=fps, band=[lo, hi])
    settings = [json.dumps(f.get('settings'), sort_keys=True) for f in fi]
    checks['settings_constant'] = _status(len(set(settings)) <= 1, distinct=len(set(settings)))

    # ---- robot
    rs = data['/robot/state']
    r_st = np.array([stamp_ns(m) for _, m in rs])
    r_win = r_st[[in_win(s) for s in r_st]] if len(r_st) else r_st
    r_hz = (len(r_win) - 1) / ((r_win[-1] - r_win[0]) * 1e-9) if len(r_win) > 1 else 0.0
    # Samples lost anywhere between the controller and the bag: controller time steps > 1.5 periods.
    rate = float(config.get_path(rig, 'robot.state_rate_hz') or 500.0)
    period = 1.0 / rate
    ctrl = np.array([m.controller_time_s for _, m in rs if in_win(stamp_ns(m))])
    # rate on the controller's clock (a simulator slower than real time is not a data problem); PC clock as info
    hz_ctrl = (len(ctrl) - 1) / (ctrl[-1] - ctrl[0]) if len(ctrl) > 1 and ctrl[-1] > ctrl[0] else 0.0
    checks['robot_rate'] = _status(hz_ctrl >= 0.98 * rate, hz=hz_ctrl, hz_pc_clock=r_hz)
    if len(ctrl) > 1:
        dc = np.diff(ctrl)
        lost = int(np.sum(np.round(dc[dc > 1.5 * period] / period) - 1))
        checks['robot_sample_gaps'] = _status(lost == 0, gaps=int(np.sum(dc > 1.5 * period)), samples_lost=lost,
                                              repeated=int(np.sum(dc <= 0)))
    else:
        checks['robot_sample_gaps'] = _status(False, reason='no robot samples in window')
    max_gap_ms = float(config.get_path(rig, 'qc.ultrasound_robot_max_gap_ms') or 10)
    if len(bm_win) and len(r_st):
        r_sorted = np.sort(r_st)
        k = np.searchsorted(r_sorted, bm_win)
        both = (k > 0) & (k < len(r_sorted))
        before = np.where(k > 0, bm_win - r_sorted[np.clip(k - 1, 0, None)], np.inf)
        after = np.where(k < len(r_sorted), r_sorted[np.clip(k, None, len(r_sorted) - 1)] - bm_win, np.inf)
        worst = float(np.max(np.maximum(before, after)) * 1e-6)
        checks['us_robot_stamp_gap'] = _status(bool(both.all()) and worst < max_gap_ms,
                                               max_gap_ms=worst, frames_without_both_sides=int((~both).sum()))
    else:
        checks['us_robot_stamp_gap'] = _status(False, reason='no frames or robot samples in window')

    # ---- every stream covers the whole window (a stall / late start used to pass: rates were measured
    # between the samples present). Allowed at each end: 3 periods, at least 0.1 s.
    if t0 is not None and t1 is not None:
        streams = {'ultrasound': ('/micrus/bmode', float(np.mean(scfg['micrus']['expected_fps']))),
                   'robot': ('/robot/state', rate)}
        if use_cam:
            streams.update({'wrist.color': (ct['color'], float(cam['fps'])), 'wrist.depth': (ct['depth'], float(cam['fps']))})
        short, ends = [], {}
        for name, (topic, hz) in streams.items():
            n, first, last = win.get(topic, (0, None, None))
            tol = max(3.0 / hz, 0.1)
            if n == 0:
                short.append(name)
                ends[name] = None
                continue
            ends[name] = [round((first - t0) * 1e-9, 3), round((t1 - last) * 1e-9, 3)]
            if (first - t0) * 1e-9 > tol or (t1 - last) * 1e-9 > tol:
                short.append(name)
        checks['stream_coverage'] = _status(not short, short=short, start_end_gap_s=ends)
    else:
        checks['stream_coverage'] = _status(False, reason='no start / end event')
    rc = ep.get('bag_returncode')
    checks['bag_closed'] = ({'status': 'n/a', 'reason': 'no bag_returncode (offline run)'} if rc is None
                            else _status(rc == 0, returncode=rc))

    # ---- coupling
    floor = config.get_path(rig, 'qc.coupling_floor')
    if ep.get('failure_mode') in PLANNED_LIFT:
        checks['coupling'] = {'status': 'n/a', 'reason': 'planned lift / coupling loss'}
    elif floor is None:
        checks['coupling'] = {'status': 'unset', 'reason': 'rig.yaml qc.coupling_floor is null'}
    else:
        means = [float(np.frombuffer(bytes(m.data), np.uint8).mean()) for t, m in data['/micrus/bmode']
                 if in_win(stamp_ns(m))]
        frac = float(np.mean(np.array(means) > floor)) if means else 0.0
        checks['coupling'] = _status(frac >= 0.90, frac_above_floor=frac, floor=floor)

    # ---- force band (probe-axis force while in contact, after the contact event)
    events = [(stamp_ns(m), m.kind, json.loads(m.value or '{}')) for _, m in data['/episode/events']]
    kinds = {k for _, k, _ in events}
    contact_ns = next((s for s, k, v in events if k == 'phase' and v.get('phase') == 'contact'), None)
    band = config.get_path(rig, 'limits.force_band_n')
    if ep.get('type') == 'failure' or ep.get('failure_mode') in PLANNED_LIFT:
        checks['force_band'] = {'status': 'n/a', 'reason': 'planned'}
    elif ep.get('force_label_valid') is False:
        checks['force_band'] = {'status': 'n/a', 'reason': f'grip {ep.get("grip")}: hand force is in the F/T reading'}
    elif band is None:
        checks['force_band'] = {'status': 'unset', 'reason': 'rig.yaml limits.force_band_n is null'}
    elif contact_ns is None:
        checks['force_band'] = {'status': 'n/a', 'reason': 'no contact phase (robot_control off?)'}
    else:
        from scipy.spatial.transform import Rotation
        f = []
        for _, m in rs:
            s = stamp_ns(m)
            if in_win(s) and s >= contact_ns:
                o = m.tcp_base.orientation
                z = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()[:, 2]
                f.append(float(z @ np.asarray(m.wrench[:3])))
        frac = float(np.mean([(band[0] <= abs(x) <= band[1]) for x in f])) if f else 0.0
        checks['force_band'] = _status(frac >= 0.95, frac_in_band=frac, band=band)

    # ---- free air, expert flag, events
    fa = [s for s, k, v in events if k == 'phase' and v.get('phase') == 'free_air']
    fa_end = [s for s, k, v in events if k == 'phase' and v.get('phase') == 'free_air_end']
    # F/T offset of this episode (probe in free air): nothing zeroes the sensor in freedrive, and the export
    # carries it so the force can be referenced to free air (open decision 18); raw values stay as recorded
    w_fa = (np.array([m.wrench for _, m in rs if fa[0] <= stamp_ns(m) <= fa_end[0]], float)
            if fa and fa_end else np.zeros((0, 6)))
    checks['free_air_window'] = _status(bool(fa) and bool(np.any(r_st >= fa[0])), samples=int(len(w_fa)),
                                        wrench_mean=w_fa.mean(axis=0).tolist() if len(w_fa) else None,
                                        wrench_std=w_fa.std(axis=0).tolist() if len(w_fa) else None)
    bad_expert = sum(1 for _, m in rs if m.action_is_expert and m.action_source not in EXPERT_SOURCES)
    checks['expert_flag'] = _status(bad_expert == 0, samples_flagged_wrongly=bad_expert)
    need = {'start', 'end', 'outcome'}
    checks['events'] = _status(need <= kinds, missing=sorted(need - kinds))

    # ---- freedrive (kinesthetic) episodes: the pedal was used, and the hand did not exceed the speed cap
    if ep.get('collection_mode') == 'freedrive':
        win = [m for _, m in rs if in_win(stamp_ns(m))]
        n_fd = sum(1 for m in win if m.action_source == 5)
        checks['freedrive_engaged'] = _status(n_fd > 0, samples=n_fd, share=n_fd / len(win) if win else 0.0)
        vmax = config.get_path(rig, 'limits.max_linear_speed_mm_s')
        if len(win) > 10:
            p = np.array([[m.tcp_base.position.x, m.tcp_base.position.y, m.tcp_base.position.z] for m in win])
            t = np.array([stamp_ns(m) for m in win]) * 1e-9
            step = 10                                        # 20 ms at 500 Hz: smooths quantisation noise
            v = np.linalg.norm(p[step:] - p[:-step], axis=1) / np.maximum(t[step:] - t[:-step], 1e-6) * 1000.0
            vpk = float(v.max()) if len(v) else 0.0
            if vmax is None:
                checks['freedrive_speed'] = {'status': 'unset', 'reason': 'limits.max_linear_speed_mm_s is null',
                                             'max_mm_s': vpk}
            else:
                checks['freedrive_speed'] = _status(vpk <= float(vmax), max_mm_s=vpk, limit_mm_s=float(vmax))
        else:
            checks['freedrive_speed'] = {'status': 'n/a', 'reason': 'too few robot samples'}

    # ---- control-loop timing (R8): robot_control reports tick intervals every 10 s while recording
    timing = [v for s, k, v in events if k == 'control_timing' and in_win(s)]
    max_late = config.get_path(rig, 'qc.max_late_tick_frac')
    if not timing:
        checks['control_timing'] = {'status': 'n/a', 'reason': 'no control_timing events (robot_control off or < 10 s)'}
    else:
        n = sum(v.get('ticks', 0) for v in timing)
        late = sum(v.get('late', 0) for v in timing)
        info = {'ticks': n, 'late': late, 'late_frac': late / n if n else 0.0,
                'max_ms': max(v.get('max_ms', 0.0) for v in timing), 'p99_ms_worst': max(v.get('p99_ms', 0.0) for v in timing)}
        if max_late is None:
            checks['control_timing'] = {'status': 'unset', 'reason': 'rig.yaml qc.max_late_tick_frac is null', **info}
        else:
            checks['control_timing'] = _status(info['late_frac'] <= float(max_late), **info)

    # ---- failure labels (Stage 2 only)
    if stage != 2:
        checks['failure_labels'] = {'status': 'n/a', 'reason': f'stage {stage}'}
    elif ep.get('type') in ('recovery-start', 'recovery-mid', 'failure'):
        req = ['failure_mode', 'staging_method', 'onset_t', 'onset_source']
        if ep.get('type') != 'failure':
            req += ['recovery_start_t', 'recovery_end_t']
        miss = [k for k in req if ep.get(k) in (None, '')]
        checks['failure_labels'] = _status(not miss, missing=miss)
    else:
        checks['failure_labels'] = {'status': 'n/a', 'reason': ep.get('type')}

    # ---- episode length vs the proposal's target (information: the outcome decides when an attempt ends)
    lo_s, hi_s = (scfg.get('targets') or {}).get('episode_s') or [None, None]
    rec_s = ((t1 - t0) * 1e-9 - float(ep.get('paused_s') or 0.0)) if t0 is not None and t1 is not None else None
    checks['episode_length'] = {'status': 'n/a', 'recorded_s': rec_s, 'target_s': [lo_s, hi_s],
                                'within_target': (rec_s is not None and lo_s is not None
                                                  and lo_s <= rec_s <= hi_s)}

    # ---- disk
    from shutil import disk_usage
    free = disk_usage(ep_dir).free / 1e9
    min_free = scfg['session'].get('min_free_disk_gb', 50)
    # information only: the recorded episode is not worse for a full disk; episode_manager refuses to START
    # an episode below the floor instead (failing here discarded good episodes)
    checks['free_disk'] = {'status': 'n/a', 'free_gb': free, 'floor_gb': min_free,
                           'reason': 'checked before each Start (episode_manager)'}

    # ---- wrist camera
    if not use_cam:
        for k in ('wrist_rate', 'wrist_stamp_offset', 'wrist_usb', 'valid_depth'):
            checks[k] = {'status': 'n/a', 'reason': 'camera not recorded'}
    else:
        fps_cfg = float(cam['fps'])
        res = {}
        for name in ('color', 'depth'):
            # messages stamped inside [start, end] only: the bag also holds the readiness tail, the free-air
            # hold and the descent before start and the stop after end (depth is decoded every 5th only)
            n, first, last = win.get(ct[name], (0, 0, 0))
            res[name] = (n - 1) / ((last - first) * 1e-9) if n > 1 and last > first else 0.0
        ok = all(abs(h - fps_cfg) <= 0.05 * fps_cfg for h in res.values())
        fn_gaps = None
        if ct.get('metadata') and data.get(ct['metadata']):
            fns = []
            for _, m in data[ct['metadata']]:
                try:
                    fns.append(int(json.loads(m.json_data).get('frame_number')))
                except (ValueError, TypeError):
                    pass
            if len(fns) > 1:
                d = np.diff(np.array(fns))
                fn_gaps = int(np.sum(d[d > 1] - 1))
        checks['wrist_rate'] = _status(ok and (fn_gaps in (None, 0)), hz=res, configured=fps_cfg,
                                       frames_lost_by_frame_number=fn_gaps if fn_gaps is not None
                                       else 'n/a (no metadata in bag)')
        off = np.array([(t - stamp_ns(m)) * 1e-6 for t, m in data[ct['color']]])
        bound = config.get_path(rig, 'qc.camera_stamp_offset_bound_ms')
        if bound is None:
            checks['wrist_stamp_offset'] = {'status': 'unset', 'median_ms': float(np.median(off)) if len(off) else None}
        else:
            checks['wrist_stamp_offset'] = _status(len(off) > 0 and float(np.percentile(np.abs(off), 99)) <= bound,
                                                   median_ms=float(np.median(off)) if len(off) else None,
                                                   p99_ms=float(np.percentile(off, 99)) if len(off) else None,
                                                   note='bag receive time minus header stamp')
        usb = [json.loads(m.data).get('wrist_usb') for t, m in data['/qc/status'] if in_win(t)]
        usb = [u for u in usb if u]
        if not usb:
            checks['wrist_usb'] = {'status': 'fail', 'reason': 'no qc_monitor USB reports in the bag'}
        else:
            speeds = {u.get('speed_mbps') for u in usb}
            reconnects = max(u.get('reconnects', 0) for u in usb) - min(u.get('reconnects', 0) for u in usb)
            checks['wrist_usb'] = _status(all((s or 0) >= 5000 for s in speeds) and reconnects == 0,
                                          speeds_mbps=sorted(x for x in speeds if x), reconnects=reconnects)
        min_frac = config.get_path(rig, 'qc.min_valid_depth_frac')
        fr = [float(np.mean(np.frombuffer(bytes(m.data), np.uint16) > 0)) for _, m in data[ct['depth']]]
        if min_frac is None:
            checks['valid_depth'] = {'status': 'unset', 'mean_valid_frac': float(np.mean(fr)) if fr else None}
        else:
            checks['valid_depth'] = _status(bool(fr) and float(np.mean(fr)) >= min_frac,
                                            mean_valid_frac=float(np.mean(fr)) if fr else None)

    # ---- speed tracking (Stage 1)
    if stage != 1:
        checks['speed_tracking'] = {'status': 'n/a'}
    else:
        lim = config.get_path(scfg, 'gate12.max_speed_error_pct')
        tr = (ep.get('speed_tracking') or {}).get('max_error_pct')
        if lim is None:
            checks['speed_tracking'] = {'status': 'unset', 'max_error_pct': tr}
        else:
            checks['speed_tracking'] = _status(tr is not None and tr <= lim, max_error_pct=tr, limit=lim)

    ok = all(c['status'] in ('pass', 'n/a') for c in checks.values())
    result = {'pass': ok, 'failed': [k for k, c in checks.items() if c['status'] not in ('pass', 'n/a')],
              'checks': checks}
    if write:
        (ep_dir / 'qc.json').write_text(json.dumps(result, indent=2, default=float) + '\n')
    return result


def main():
    """episode_qc <episode_dir | session_dir> [--update]: one episode, or every ep_* of a session (discards skipped).
    --update writes the result into episode.json (the previous one kept as qc_previous): progress and the export
    read episode.json, so episodes recorded before the QC thresholds were set count once they pass. The recorded
    data is never touched."""
    ap = argparse.ArgumentParser(description=__doc__ + main.__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('path', help='episode folder (ep_NNNN) or session folder')
    ap.add_argument('--config-dir')
    ap.add_argument('--update', action='store_true', help='write the result into episode.json')
    a, _ = ap.parse_known_args()
    p = Path(a.path).expanduser()
    eps = [p] if (p / 'episode.json').exists() and p.name.startswith('ep_') else sorted(p.glob('ep_*'))
    scfg, rig = config.load_config('session', a.config_dir), config.load_config('rig', a.config_dir)
    n_pass = n_run = 0
    for d in eps:
        f = d / 'episode.json'
        if not f.exists():
            continue
        ep = json.loads(f.read_text())
        if len(eps) > 1 and ep.get('outcome') == 'discard':
            continue
        r = run_episode_qc(d, ep, scfg, rig)
        n_run += 1
        n_pass += bool(r['pass'])
        if len(eps) == 1:
            for k, c in r['checks'].items():
                extra = {x: y for x, y in c.items() if x != 'status'}
                print(f'{c["status"].upper():6s} {k:22s} {json.dumps(extra, default=float)}')
        print(f'{d.name}: EPISODE QC {"PASS" if r["pass"] else "FAIL " + str(r.get("failed"))}')
        if a.update:
            ep['qc_previous'] = ep.get('qc')
            ep['qc'] = r
            tmp = f.with_suffix('.json.tmp')
            tmp.write_text(json.dumps(ep, indent=2, default=float) + '\n')
            tmp.replace(f)
    if len(eps) > 1:
        print(f'{n_pass} of {n_run} episodes pass' + ('' if a.update else ' (not written: add --update)'))
    sys.exit(0 if n_run and n_pass == n_run else 1)


if __name__ == '__main__':
    main()
