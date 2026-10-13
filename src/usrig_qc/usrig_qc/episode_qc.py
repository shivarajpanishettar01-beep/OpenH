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

ROBOT_SOURCES = {0: 'teleop', 1: 'scripted_task', 2: 'transit', 3: 'perturbation', 4: 'none'}
PLANNED_LIFT = {'FM-01', 'FM-02'}


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


def read_bag(bag_dir, topics, decode_every=None):
    """{topic: [(log_ns, msg), ...]}; decode_every={topic: n} keeps every n-th message only."""
    from rosbags.highlevel import AnyReader
    out = {t: [] for t in topics}
    counts = {t: 0 for t in topics}
    with AnyReader([Path(bag_dir)], default_typestore=_typestore()) as r:
        conns = [c for c in r.connections if c.topic in topics]
        for c, t, raw in r.messages(connections=conns):
            counts[c.topic] += 1
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
    data, counts = read_bag(ep_dir, topics, decode_every={ct.get('depth'): 5} if use_cam else None)
    checks = {}
    t0, t1 = ep.get('t_start_ns'), ep.get('t_end_ns')

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
    checks['robot_rate'] = _status(r_hz >= 490.0, hz=r_hz)
    # Samples lost anywhere between the controller and the bag: controller time steps > 1.5 periods.
    period = 1.0 / float(config.get_path(rig, 'robot.state_rate_hz') or 500.0)
    ctrl = np.array([m.controller_time_s for _, m in rs if in_win(stamp_ns(m))])
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
    checks['free_air_window'] = _status(bool(fa) and bool(np.any(r_st >= fa[0])))
    bad_expert = sum(1 for _, m in rs if m.action_is_expert and m.action_source not in (0, 1))
    checks['expert_flag'] = _status(bad_expert == 0, samples_flagged_wrongly=bad_expert)
    need = {'start', 'end', 'outcome'}
    checks['events'] = _status(need <= kinds, missing=sorted(need - kinds))

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

    # ---- disk
    from shutil import disk_usage
    free = disk_usage(ep_dir).free / 1e9
    min_free = scfg['session'].get('min_free_disk_gb', 50)
    checks['free_disk'] = _status(free > min_free, free_gb=free)

    # ---- wrist camera
    if not use_cam:
        for k in ('wrist_rate', 'wrist_stamp_offset', 'wrist_usb', 'valid_depth'):
            checks[k] = {'status': 'n/a', 'reason': 'camera not recorded'}
    else:
        fps_cfg = float(cam['fps'])
        res = {}
        for name in ('color', 'depth'):
            st = np.array([stamp_ns(m) for _, m in data[ct[name]] if in_win(stamp_ns(m))])
            n_all = counts[ct[name]]
            if name == 'depth':
                # decoded every 5th: use the count of all messages over the episode span
                hz = (n_all - 1) / ((t1 - t0) * 1e-9) if t0 and t1 and n_all > 1 else 0.0
            else:
                hz = (len(st) - 1) / ((st[-1] - st[0]) * 1e-9) if len(st) > 1 else 0.0
            res[name] = hz
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
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('episode_dir')
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    ep = json.loads((Path(a.episode_dir) / 'episode.json').read_text())
    r = run_episode_qc(a.episode_dir, ep, config.load_config('session', a.config_dir),
                       config.load_config('rig', a.config_dir))
    for k, c in r['checks'].items():
        extra = {x: y for x, y in c.items() if x != 'status'}
        print(f'{c["status"].upper():6s} {k:22s} {json.dumps(extra, default=float)}')
    print('EPISODE QC', 'PASS' if r['pass'] else 'FAIL')
    sys.exit(0 if r['pass'] else 1)


if __name__ == '__main__':
    main()
