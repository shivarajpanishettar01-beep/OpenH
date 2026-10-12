#!/usr/bin/env python3
"""Temporal calibration from a sinusoidal compress test on the phantom (P9c).

The robot moves the probe sinusoidally along its axis in contact. Ultrasound: the depth of a
bright horizontal line (e.g. the phantom bottom or a wire) moves with robot z. D415: the median
depth of the phantom surface in an ROI moves with robot z. Cross-correlating each signal with the
robot's probe-axis position gives the latency of the sensor's header stamps relative to RTDE.

  python temporal_calib.py <bag> --us-rows 150 380 [--us-cols 100 300] \
      [--depth-roi 200 140 440 340] [--max-lag-ms 200]

Prints the lag (ms) and the time_offset_s to use in export/lerobot_rig.yaml (offset = -lag).
Real numbers only: nothing here is assumed.
"""
import argparse
from pathlib import Path

import numpy as np


def stamp(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def xcorr_lag(t_ref, x_ref, t_sig, x_sig, max_lag_s, dt=0.001):
    """Lag (s) that best aligns x_sig to x_ref: x_sig(t) ~ x_ref(t - lag)."""
    t0 = max(t_ref[0], t_sig[0]) + max_lag_s
    t1 = min(t_ref[-1], t_sig[-1]) - max_lag_s
    if t1 <= t0:
        raise SystemExit('signals do not overlap long enough')
    tg = np.arange(t0, t1, dt)
    r = np.interp(tg, t_ref, x_ref)
    r = (r - r.mean()) / (r.std() + 1e-12)
    best = (None, -np.inf)
    for lag in np.arange(-max_lag_s, max_lag_s + dt, dt):
        s = np.interp(tg + lag, t_sig, x_sig)
        s = (s - s.mean()) / (s.std() + 1e-12)
        c = float(abs((r * s).mean()))
        if c > best[1]:
            best = (lag, c)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--us-rows', type=int, nargs=2, required=True, help='row band containing the line')
    ap.add_argument('--us-cols', type=int, nargs=2)
    ap.add_argument('--depth-topic', default='/wrist/d415/aligned_depth_to_color/image_raw')
    ap.add_argument('--depth-roi', type=int, nargs=4)
    ap.add_argument('--max-lag-ms', type=float, default=200.0)
    a = ap.parse_args()
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    msg_dir = Path(__file__).resolve().parents[1] / 'src/usrig_interfaces/msg'
    for f in msg_dir.glob('*.msg'):
        ts.register(get_types_from_msg(f.read_text(), f'usrig_interfaces/msg/{f.stem}'))

    rz_t, rz, us_t, us, d_t, d = [], [], [], [], [], []
    with AnyReader([Path(a.bag)], default_typestore=ts) as r:
        topics = {'/robot/state', '/micrus/bmode', a.depth_topic}
        for c, _, raw in r.messages(connections=[c for c in r.connections if c.topic in topics]):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == '/robot/state':
                o = m.tcp_base.orientation
                x, y, z, w = o.x, o.y, o.z, o.w
                axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
                p = m.tcp_base.position
                rz_t.append(stamp(m) * 1e-9)
                rz.append(float(axis @ [p.x, p.y, p.z]))
            elif c.topic == '/micrus/bmode':
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width].astype(float)
                band = img[a.us_rows[0]:a.us_rows[1]]
                if a.us_cols:
                    band = band[:, a.us_cols[0]:a.us_cols[1]]
                prof = band.mean(1)
                w8 = np.clip(prof - np.percentile(prof, 50), 0, None)
                us_t.append(stamp(m) * 1e-9)
                us.append(float((w8 * np.arange(len(prof))).sum() / (w8.sum() + 1e-9)))
            elif a.depth_roi:
                img = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
                x0, y0, x1, y1 = a.depth_roi
                v = img[y0:y1, x0:x1]
                v = v[v > 0]
                if v.size:
                    d_t.append(stamp(m) * 1e-9)
                    d.append(float(np.median(v)))
    rz_t, rz = np.array(rz_t), np.array(rz)
    o = np.argsort(rz_t)
    rz_t, rz = rz_t[o], rz[o]
    lag, c = xcorr_lag(rz_t, rz, np.array(us_t), np.array(us), a.max_lag_ms / 1000)
    print(f'ultrasound: lag {lag * 1000:.1f} ms (|corr| {c:.3f}, {len(us)} frames) '
          f'-> time_offset_s {-lag:.4f}')
    if d:
        lag, c = xcorr_lag(rz_t, rz, np.array(d_t), np.array(d), a.max_lag_ms / 1000)
        print(f'D415 depth: lag {lag * 1000:.1f} ms (|corr| {c:.3f}, {len(d)} frames) '
              f'-> time_offset_s {-lag:.4f}')


if __name__ == '__main__':
    main()
