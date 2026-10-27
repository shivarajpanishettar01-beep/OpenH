"""bag_check: is a recorded test bag complete and consistent? (staged tests, docs/TESTING_STEPS.md)

  ros2 run usrig_qc bag_check <bag_dir> --expect robot [micrus] [camera] [tpv] [--json out.json]

Reads header stamps from the raw messages (images are not decoded) and reports per topic: messages,
rate, largest gap, stamps going backwards, and header stamp vs bag time. Then the checks for the parts
under test (PASS / FAIL):
  robot   /robot/state at state_rate_hz (+-2 %), 0 robot samples lost (controller time), /joint_states,
          /robot/tcp_pose, /robot/wrench present with the same count as /robot/state, poses in metres;
          the data is READ, not only present: every value finite, joint angles not all zero, TCP not at
          the base origin, the F/T reading live (varies: a real sensor always has noise; all zero or
          constant = actual_TCP_force not arriving), and /joint_states, /robot/tcp_pose, /robot/wrench
          carry the same sample as /robot/state (matched by stamp)
  micrus  /micrus/bmode in micrus.expected_fps, 0 frame_index gaps, settings constant, every frame has a
          robot sample within qc.ultrasound_robot_max_gap_ms on both sides (with robot)
  camera  colour and depth at wrist_camera.fps (+-5 %), camera_info present, depth 16UC1
  tpv     third-person view colour at tpv_camera.fps (+-5 %), camera_info present
Prints what the robot data shows: how far the TCP moved and turned, the force / torque range, and the
samples labelled freedrive (pedal held), so you can check them against what you did during the recording.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

from usrig_common import config

from .episode_qc import _typestore
from .stream_check import stamp_from_cdr

NO_HEADER = {'/micrus/frame_info', '/operator/command', '/qc/status'}


def read(bag, decode, first_only=()):
    """{topic: {'stamps': [ns], 'log': [ns], 'msgs': [decoded, only for topics in decode]}};
    topics in first_only: only their first message is decoded (e.g. one depth image for its encoding)."""
    from rosbags.highlevel import AnyReader
    out = {}
    with AnyReader([Path(bag)], default_typestore=_typestore()) as r:
        types = {c.topic: c.msgtype for c in r.connections}
        for c, t, raw in r.messages():
            d = out.setdefault(c.topic, {'stamps': [], 'log': [], 'msgs': [], 'type': c.msgtype})
            d['log'].append(t)
            if c.topic not in NO_HEADER and c.topic != '/tf_static':
                try:
                    d['stamps'].append(stamp_from_cdr(raw))
                except Exception:  # noqa: BLE001
                    pass
            if c.topic in decode or (c.topic in first_only and not d['msgs']):
                d['msgs'].append(r.deserialize(raw, c.msgtype))
    return out, types


def topic_row(name, d):
    st = np.asarray(d['stamps'] or d['log'], np.int64)
    n = len(d['log'])
    if len(st) < 2:
        return {'topic': name, 'n': n, 'hz': 0.0, 'max_gap_ms': None, 'backwards': 0, 'stamp_minus_log_ms': None}
    dt = np.diff(st)
    lag = (np.asarray(d['log'][:len(st)], np.int64) - st) * 1e-6 if d['stamps'] else np.zeros(1)
    return {'topic': name, 'n': n, 'hz': float((len(st) - 1) / ((st[-1] - st[0]) * 1e-9)) if st[-1] > st[0] else 0.0,
            'max_gap_ms': float(dt.max() * 1e-6), 'backwards': int(np.sum(dt <= 0)),
            'stamp_minus_log_ms': float(np.median(-lag)) if d['stamps'] else None}


def check(bag, expect, scfg, rig):
    ct = scfg['wrist_camera']['topics']
    decode = {'/robot/state', '/micrus/frame_info'}
    if 'robot' in expect:
        decode |= {'/joint_states', '/robot/tcp_pose', '/robot/wrench'}
    data, types = read(bag, decode, first_only={ct['depth']} if 'camera' in expect else ())
    rows = [topic_row(t, d) for t, d in sorted(data.items())]
    checks = []

    def add(name, ok, detail):
        checks.append((name, bool(ok), detail))

    rate = float(config.get_path(rig, 'robot.state_rate_hz') or 500.0)
    rs = data.get('/robot/state', {'msgs': [], 'stamps': [], 'log': []})
    summary = None
    if 'robot' in expect:
        r = topic_row('/robot/state', rs) if rs['log'] else {'hz': 0.0}
        ctrl = np.array([m.controller_time_s for m in rs['msgs']])
        # judged on the controller's clock (URSim runs slower than real time; the PC-clock rate is info)
        hz_ctrl = (len(ctrl) - 1) / (ctrl[-1] - ctrl[0]) if len(ctrl) > 1 and ctrl[-1] > ctrl[0] else 0.0
        add('robot: /robot/state rate (controller clock)', abs(hz_ctrl - rate) <= 0.02 * rate,
            f'{hz_ctrl:.1f} Hz (expected {rate:g}); PC clock {r["hz"]:.1f} Hz'
            + ('  (simulator slower than real time)' if hz_ctrl and r['hz'] < 0.98 * hz_ctrl else ''))
        if len(ctrl) > 1:
            dc = np.diff(ctrl)
            p = 1.0 / rate
            lost = int(np.sum(np.round(dc[dc > 1.5 * p] / p) - 1))
            add('robot: samples lost (controller time)', lost == 0 and not np.any(dc <= 0),
                f'{lost} lost, {int(np.sum(dc <= 0))} repeated')
        else:
            add('robot: samples lost (controller time)', False, 'no /robot/state')
        n = len(rs['log'])
        for t in ('/joint_states', '/robot/tcp_pose', '/robot/wrench'):
            m = len(data.get(t, {'log': []})['log'])
            # topics are subscribed a few ms apart: a handful of messages differ at the bag start / end
            add(f'robot: {t}', n > 0 and abs(m - n) <= max(5, 0.001 * n), f'{m} messages (robot state {n})')
        if rs['msgs']:
            summary = robot_content(rs['msgs'], data, add)
    if 'micrus' in expect:
        bm = data.get('/micrus/bmode', {'log': [], 'stamps': []})
        r = topic_row('/micrus/bmode', bm) if bm['log'] else {'hz': 0.0}
        lo, hi = scfg['micrus']['expected_fps']
        add('micrus: /micrus/bmode rate', lo <= r['hz'] <= hi, f'{r["hz"]:.2f} fps (band {lo}-{hi})')
        fi = [json.loads(m.data) for m in data.get('/micrus/frame_info', {'msgs': []})['msgs']]
        idx = np.array([f['frame_index'] for f in fi if 'frame_index' in f])
        gaps = int(np.sum(np.diff(idx) != 1)) if len(idx) > 1 else None
        add('micrus: frame_index gaps', gaps == 0, f'{gaps} gaps over {len(idx)} frames')
        settings = {json.dumps(f.get('settings'), sort_keys=True) for f in fi}
        add('micrus: settings constant', len(settings) <= 1, f'{len(settings)} distinct settings')
        if 'robot' in expect and bm['stamps'] and rs['stamps']:
            rst = np.sort(np.asarray(rs['stamps'], np.int64))
            us = np.asarray(bm['stamps'], np.int64)
            us = us[(us > rst[0]) & (us < rst[-1])]
            k = np.searchsorted(rst, us)
            worst = float(np.max(np.maximum(us - rst[k - 1], rst[k] - us)) * 1e-6) if len(us) else float('inf')
            lim = float(config.get_path(rig, 'qc.ultrasound_robot_max_gap_ms') or 10)
            add('micrus+robot: robot sample around every frame', worst < lim, f'worst {worst:.2f} ms (limit {lim:g})')
    if 'camera' in expect:
        fps = float(scfg['wrist_camera']['fps'])
        for k in ('color', 'depth'):
            d = data.get(ct[k], {'log': [], 'stamps': []})
            r = topic_row(ct[k], d) if d['log'] else {'hz': 0.0}
            add(f'camera: {k} rate', abs(r['hz'] - fps) <= 0.05 * fps, f'{r["hz"]:.2f} Hz (expected {fps:g})')
        for k in ('color_info', 'depth_info'):
            add(f'camera: {k}', len(data.get(ct[k], {'log': []})['log']) > 0, ct[k])
        dm = data.get(ct['depth'], {'msgs': []})['msgs']
        enc = dm[0].encoding if dm else None
        add('camera: depth is 16-bit (16UC1, mm)', enc in ('16UC1', 'mono16'), f'encoding {enc}')
    if 'tpv' in expect:
        tpv = scfg['tpv_camera']
        fps = float(tpv['fps'])
        d = data.get(tpv['topics']['color'], {'log': [], 'stamps': []})
        r = topic_row(tpv['topics']['color'], d) if d['log'] else {'hz': 0.0}
        add('tpv: color rate', abs(r['hz'] - fps) <= 0.05 * fps, f'{r["hz"]:.2f} Hz (expected {fps:g})')
        add('tpv: color_info', len(data.get(tpv['topics']['color_info'], {'log': []})['log']) > 0,
            tpv['topics']['color_info'])
    for r in rows:
        if r['backwards']:
            add(f'stamps in order: {r["topic"]}', False, f'{r["backwards"]} stamps not increasing')
    return rows, checks, summary


def _stamp(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def robot_content(msgs, data, add):
    """Checks that the robot values were read (not zeros / constants) and that the derived topics carry the
    same sample as /robot/state; returns a summary of what the data shows."""
    q = np.array([m.q for m in msgs], float)
    qd = np.array([m.qd for m in msgs], float)
    w = np.array([m.wrench for m in msgs], float)
    pos = np.array([[m.tcp_base.position.x, m.tcp_base.position.y, m.tcp_base.position.z] for m in msgs])
    quat = np.array([[m.tcp_base.orientation.x, m.tcp_base.orientation.y, m.tcp_base.orientation.z,
                      m.tcp_base.orientation.w] for m in msgs])
    bad = int(np.sum(~np.isfinite(np.hstack([q, qd, w, pos, quat])).all(axis=1)))
    add('robot: values finite (q, qd, TCP pose, wrench)', bad == 0, f'{bad} of {len(msgs)} samples with NaN / inf')
    ok_rows = np.isfinite(np.hstack([q, w, pos, quat])).all(axis=1)
    q, w, pos, quat = q[ok_rows], w[ok_rows], pos[ok_rows], quat[ok_rows]
    if not len(q):
        return None
    add('robot: joint angles read', bool(np.any(q != 0)),
        f'first {np.round(np.degrees(q[0]), 1).tolist()} deg' if np.any(q != 0) else 'every joint angle is 0 in every sample')
    r = np.linalg.norm(pos, axis=1)
    add('robot: TCP pose plausible (metres, within 2 m)', float(np.max(np.abs(pos))) < 2.0,
        f'first {np.round(pos[0] * 1000, 1).tolist()} mm, last {np.round(pos[-1] * 1000, 1).tolist()} mm')
    n_qbad = int(np.sum(np.abs(np.linalg.norm(quat, axis=1) - 1.0) > 1e-3))
    add('robot: TCP pose read (not at the base origin)', float(np.min(r)) > 0.01 and n_qbad == 0,
        f'closest to the base origin {np.min(r) * 1000:.1f} mm; {n_qbad} orientations not unit quaternions')
    spread = np.ptp(w, axis=0)
    f = np.linalg.norm(w[:, :3], axis=1)
    t = np.linalg.norm(w[:, 3:], axis=1)
    live = bool(np.any(w != 0) and np.any(spread > 0))
    add('robot: force / torque read (F/T live)', live,
        f'|F| {f.min():.2f} .. {f.max():.2f} N, |T| up to {t.max():.3f} Nm, spread per axis '
        f'{np.round(spread, 3).tolist()}' if live else
        ('every wrench value is 0: actual_TCP_force is not arriving (RTDE output / F/T sensor); '
         'URSim always reports 0 (no F/T sensor), so this fails in simulation' if not np.any(w != 0)
         else f'wrench constant {np.round(w[0], 3).tolist()}: a live F/T sensor always has noise'))

    by_stamp = {_stamp(m): m for m in msgs}

    def compare(topic, values, ref, tol):
        got = data.get(topic, {'msgs': []})['msgs']
        n, worst = 0, 0.0
        for m in got:
            s = by_stamp.get(_stamp(m))
            if s is None:
                continue
            n += 1
            d = np.abs(np.asarray(values(m), float) - np.asarray(ref(s), float))
            worst = max(worst, float(np.nanmax(d)) if np.all(np.isfinite(d)) else float('inf'))
        add(f'robot: {topic} = /robot/state', n > 0 and n >= 0.99 * len(got) and worst <= tol,
            f'{n} of {len(got)} messages matched by stamp, largest difference {worst:.3g}')

    compare('/joint_states', lambda m: m.position, lambda s: s.q, 1e-9)
    compare('/robot/tcp_pose', lambda m: [m.pose.position.x, m.pose.position.y, m.pose.position.z,
                                          m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z,
                                          m.pose.orientation.w],
            lambda s: [s.tcp_base.position.x, s.tcp_base.position.y, s.tcp_base.position.z,
                       s.tcp_base.orientation.x, s.tcp_base.orientation.y, s.tcp_base.orientation.z,
                       s.tcp_base.orientation.w], 1e-9)
    compare('/robot/wrench', lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                                        m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z],
            lambda s: s.wrench, 1e-9)

    qn = quat / np.linalg.norm(quat, axis=1, keepdims=True)
    turned = np.degrees(2 * np.arccos(np.clip(np.abs(qn @ qn[0]), 0.0, 1.0)))   # angle from the first orientation
    modes = np.array([m.mode for m in msgs])
    return {'first_mm': (pos[0] * 1000).tolist(), 'last_mm': (pos[-1] * 1000).tolist(),
            'moved_mm': float(np.max(np.linalg.norm(pos - pos[0], axis=1)) * 1000),
            'path_mm': float(np.sum(np.linalg.norm(np.diff(pos, axis=0), axis=1)) * 1000),
            'turned_deg': float(np.max(turned)),
            'force_n': {'min': float(f.min()), 'median': float(np.median(f)), 'max': float(f.max())},
            'torque_nm_max': float(t.max()),
            'freedrive_samples': int(np.sum(modes == 7)), 'samples': len(msgs)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--expect', nargs='+', choices=['robot', 'micrus', 'camera', 'tpv'], default=['robot'])
    ap.add_argument('--json')
    ap.add_argument('--config-dir')
    a = ap.parse_args(argv)
    scfg = config.load_config('session', a.config_dir)
    rig = config.load_config('rig', a.config_dir)
    rows, checks, summary = check(a.bag, a.expect, scfg, rig)
    print(f'\n{a.bag}\n{"TOPIC":45s} {"MSGS":>7s} {"HZ":>8s} {"MAX GAP ms":>11s} {"STAMP-LOG ms":>13s}')
    for r in rows:
        g = '-' if r['max_gap_ms'] is None else f'{r["max_gap_ms"]:.1f}'
        s = '-' if r['stamp_minus_log_ms'] is None else f'{r["stamp_minus_log_ms"]:.2f}'
        print(f'{r["topic"]:45s} {r["n"]:7d} {r["hz"]:8.2f} {g:>11s} {s:>13s}')
    size_mb = sum(f.stat().st_size for f in Path(a.bag).rglob('*') if f.is_file()) / 1e6
    from rosbags.highlevel import AnyReader
    from usrig_common.bag_io import tolerate_crlf_msgdefs
    tolerate_crlf_msgdefs()
    with AnyReader([Path(a.bag)]) as rd:
        dur = (rd.end_time - rd.start_time) * 1e-9
    if dur > 0:
        print(f'\n{size_mb:.1f} MB in {dur:.1f} s = {size_mb / dur:.2f} MB/s (bag only; micrus HDF5 not included)')
    if summary:
        s = summary
        print(f'\nrobot data ({s["samples"]} samples): compare with what you did during the recording')
        print(f'  TCP moved up to {s["moved_mm"]:.1f} mm from the start (path {s["path_mm"]:.1f} mm), '
              f'turned up to {s["turned_deg"]:.1f} deg')
        print(f'  first {np.round(s["first_mm"], 1).tolist()} mm   last {np.round(s["last_mm"], 1).tolist()} mm (base frame)')
        print(f'  force |F| {s["force_n"]["min"]:.2f} / {s["force_n"]["median"]:.2f} / {s["force_n"]["max"]:.2f} N '
              f'(min / median / max), torque up to {s["torque_nm_max"]:.3f} Nm')
        print(f'  freedrive (pedal held): {s["freedrive_samples"]} samples '
              f'({s["freedrive_samples"] / max(1, s["samples"]):.0%})')
    from usrig_common.motion_check import print_table
    ok = print_table(f'bag_check ({", ".join(a.expect)})', checks)
    if a.json:
        Path(a.json).write_text(json.dumps({'topics': rows, 'checks': [
            {'check': c, 'pass': p, 'detail': d} for c, p, d in checks], 'robot_data': summary, 'pass': ok}, indent=2))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
