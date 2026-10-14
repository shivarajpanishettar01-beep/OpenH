"""bag_check: is a recorded test bag complete and consistent? (staged tests, docs/TESTING_STEPS.md)

  ros2 run usrig_qc bag_check <bag_dir> --expect robot [micrus] [camera] [--json out.json]

Reads header stamps from the raw messages (images are not decoded) and reports per topic: messages,
rate, largest gap, stamps going backwards, and header stamp vs bag time. Then the checks for the parts
under test (PASS / FAIL):
  robot   /robot/state at state_rate_hz (+-2 %), 0 robot samples lost (controller time), /joint_states,
          /robot/tcp_pose, /robot/wrench present with the same count as /robot/state, poses in metres
  micrus  /micrus/bmode in micrus.expected_fps, 0 frame_index gaps, settings constant, every frame has a
          robot sample within qc.ultrasound_robot_max_gap_ms on both sides (with robot)
  camera  colour and depth at wrist_camera.fps (+-5 %), camera_info present, depth 16UC1
Prints the first and last TCP pose of the recording, so you can see the arm's motion in the data.
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
    data, types = read(bag, decode, first_only={ct['depth']} if 'camera' in expect else ())
    rows = [topic_row(t, d) for t, d in sorted(data.items())]
    checks = []

    def add(name, ok, detail):
        checks.append((name, bool(ok), detail))

    rate = float(config.get_path(rig, 'robot.state_rate_hz') or 500.0)
    rs = data.get('/robot/state', {'msgs': [], 'stamps': [], 'log': []})
    poses = None
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
            p0, p1 = rs['msgs'][0].tcp_base.position, rs['msgs'][-1].tcp_base.position
            poses = ([p0.x, p0.y, p0.z], [p1.x, p1.y, p1.z])
            add('robot: TCP pose plausible (metres, within 2 m)', max(map(abs, poses[0] + poses[1])) < 2.0,
                f'first {np.round(np.array(poses[0]) * 1000, 1).tolist()} mm, last {np.round(np.array(poses[1]) * 1000, 1).tolist()} mm')
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
    for r in rows:
        if r['backwards']:
            add(f'stamps in order: {r["topic"]}', False, f'{r["backwards"]} stamps not increasing')
    return rows, checks, poses


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--expect', nargs='+', choices=['robot', 'micrus', 'camera'], default=['robot'])
    ap.add_argument('--json')
    ap.add_argument('--config-dir')
    a = ap.parse_args(argv)
    scfg = config.load_config('session', a.config_dir)
    rig = config.load_config('rig', a.config_dir)
    rows, checks, poses = check(a.bag, a.expect, scfg, rig)
    print(f'\n{a.bag}\n{"TOPIC":45s} {"MSGS":>7s} {"HZ":>8s} {"MAX GAP ms":>11s} {"STAMP-LOG ms":>13s}')
    for r in rows:
        g = '-' if r['max_gap_ms'] is None else f'{r["max_gap_ms"]:.1f}'
        s = '-' if r['stamp_minus_log_ms'] is None else f'{r["stamp_minus_log_ms"]:.2f}'
        print(f'{r["topic"]:45s} {r["n"]:7d} {r["hz"]:8.2f} {g:>11s} {s:>13s}')
    size_mb = sum(f.stat().st_size for f in Path(a.bag).rglob('*') if f.is_file()) / 1e6
    from rosbags.highlevel import AnyReader
    with AnyReader([Path(a.bag)]) as rd:
        dur = (rd.end_time - rd.start_time) * 1e-9
    if dur > 0:
        print(f'\n{size_mb:.1f} MB in {dur:.1f} s = {size_mb / dur:.2f} MB/s (bag only; micrus HDF5 not included)')
    if poses:
        d = np.array(poses[1]) - np.array(poses[0])
        print(f'\nTCP moved {np.linalg.norm(d) * 1000:.1f} mm between the first and the last sample '
              f'(d {np.round(d * 1000, 1).tolist()} mm)')
    from usrig_common.motion_check import print_table
    ok = print_table(f'bag_check ({", ".join(a.expect)})', checks)
    if a.json:
        Path(a.json).write_text(json.dumps({'topics': rows, 'checks': [
            {'check': c, 'pass': p, 'detail': d} for c, p, d in checks], 'pass': ok}, indent=2))
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
