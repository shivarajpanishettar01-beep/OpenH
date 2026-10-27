#!/usr/bin/env python3
"""Temporal calibration from a sinusoidal compress test on the phantom (proposal section 9).

The robot compresses the phantom periodically along the probe axis. Robot reference: the probe-axis
position of the TCP (RTDE, 500 Hz). Sensor signals:
  ultrasound   depth of a tracked bright structure (row centroid in a band), B-mode
  wrist camera (a) --aruco: distance to a fixed ArUco marker (solvePnP on the colour image), or
               (b) --depth-roi: median depth of the phantom surface in an ROI
  TPV camera   --camera tpv_camera --aruco: the camera is fixed, so the marker goes on the probe holder
               (it moves with the arm); colour only, no --depth-roi
Each signal is compared with the robot reference two ways: cross-correlation, and fitting a
sinusoid at the robot's dominant frequency (phase difference -> lag). Both are reported.

  python temporal_calib.py BAG --us-rows 150 380 [--us-cols 100 300]
      [--aruco DICT_4X4_50 --marker-mm 40 | --depth-roi 200 140 440 340]
  python temporal_calib.py START_BAG END_BAG ...   # start and end of a session: reports the drift
  add --session <session_dir> to store the result in session.json (temporal_calibration)

time_offset_s for the exports = -lag. Real numbers only; nothing here is assumed.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]


def stamp(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


def xcorr_lag(t_ref, x_ref, t_sig, x_sig, max_lag_s, dt=0.001):
    """Lag (s) such that x_sig(t) ~ x_ref(t - lag)."""
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
            best = (float(lag), c)
    return best


def dominant_freq(t, x):
    dt = np.median(np.diff(t))
    tg = np.arange(t[0], t[-1], dt)
    xg = np.interp(tg, t, x) - np.mean(x)
    spec = np.abs(np.fft.rfft(xg * np.hanning(len(xg))))
    f = np.fft.rfftfreq(len(xg), dt)
    k = 1 + int(np.argmax(spec[1:]))
    return float(f[k])


def fit_phase(t, x, f):
    """Least-squares x = a sin(wt) + b cos(wt) + c -> (phase rad, amplitude, residual rms / amplitude)."""
    w = 2 * np.pi * f
    M = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    (a, b, c), *_ = np.linalg.lstsq(M, x, rcond=None)
    amp = float(np.hypot(a, b))
    res = x - M @ np.array([a, b, c])
    return float(np.arctan2(b, a)), amp, float(np.sqrt(np.mean(res ** 2)) / (amp + 1e-12))


def sine_lag(t_ref, x_ref, t_sig, x_sig, f):
    """Lag from the phase difference of sinusoids fitted at the robot frequency (same sign as xcorr)."""
    t0 = min(t_ref[0], t_sig[0])
    p_ref, _, q_ref = fit_phase(t_ref - t0, x_ref, f)
    p_sig, _, q_sig = fit_phase(t_sig - t0, x_sig, f)
    # x_sig(t) = x_ref(t - lag)  ->  phase_sig = phase_ref - w*lag  (sign of x_sig may be inverted)
    best = None
    for sgn, dp in ((1, p_ref - p_sig), (-1, p_ref - p_sig + np.pi)):
        dp = (dp + np.pi) % (2 * np.pi) - np.pi
        lag = dp / (2 * np.pi * f)
        if best is None or abs(lag) < abs(best[0]):
            best = (float(lag), sgn)
    return best[0], max(q_ref, q_sig)


def read(bag, a):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    for f in (REPO / 'src/usrig_interfaces/msg').glob('*.msg'):
        ts.register(get_types_from_msg(f.read_text(), f'usrig_interfaces/msg/{f.stem}'))
    if a.aruco:
        import cv2
        aruco = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, a.aruco)))
        half = a.marker_mm / 2000.0
        obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)
    rz_t, rz, us_t, us, c_t, c, K, D = [], [], [], [], [], [], None, None
    # ultrasound only with --us-rows (a camera-only recording, e.g. test 3, has none)
    topics = {'/robot/state', a.color_topic, a.color_info_topic, a.depth_topic} | ({'/micrus/bmode'} if a.us_rows else set())
    with AnyReader([Path(bag)], default_typestore=ts) as r:
        for con, _, raw in r.messages(connections=[x for x in r.connections if x.topic in topics]):
            m = r.deserialize(raw, con.msgtype)
            if con.topic == '/robot/state':
                o = m.tcp_base.orientation
                x, y, z, w = o.x, o.y, o.z, o.w
                axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
                p = m.tcp_base.position
                rz_t.append(stamp(m) * 1e-9)
                rz.append(float(axis @ [p.x, p.y, p.z]))
            elif con.topic == '/micrus/bmode':
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width].astype(float)
                band = img[a.us_rows[0]:a.us_rows[1]]
                if a.us_cols:
                    band = band[:, a.us_cols[0]:a.us_cols[1]]
                prof = band.mean(1)
                w8 = np.clip(prof - np.percentile(prof, 50), 0, None)
                us_t.append(stamp(m) * 1e-9)
                us.append(float((w8 * np.arange(len(prof))).sum() / (w8.sum() + 1e-9)))
            elif con.topic == a.color_info_topic and K is None:
                K, D = np.array(m.k).reshape(3, 3), np.array(m.d)
            elif con.topic == a.color_topic and a.aruco and K is not None:
                img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
                gray = img.reshape(m.height, m.width, 3).mean(2).astype(np.uint8)
                corners, ids, _ = aruco.detectMarkers(gray)
                if ids is not None and len(ids):
                    ok, _, tvec = cv2.solvePnP(obj, corners[0].reshape(4, 2).astype(np.float32), K, D,
                                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        c_t.append(stamp(m) * 1e-9)
                        c.append(float(np.linalg.norm(tvec)))
            elif con.topic == a.depth_topic and a.depth_roi and not a.aruco:
                img = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
                x0, y0, x1, y1 = a.depth_roi
                v = img[y0:y1, x0:x1]
                v = v[v > 0]
                if v.size:
                    c_t.append(stamp(m) * 1e-9)
                    c.append(float(np.median(v)))
    o = np.argsort(rz_t)
    return (np.array(rz_t)[o], np.array(rz)[o], np.array(us_t), np.array(us), np.array(c_t), np.array(c))


def calibrate(bag, a):
    rz_t, rz, us_t, us, c_t, c = read(bag, a)
    f = dominant_freq(rz_t, rz)
    res = {'bag': str(bag), 'robot_freq_hz': f}
    cam = a.camera
    for name, t, x in (('ultrasound', us_t, us), (cam, c_t, c)):
        if len(x) < 10:
            continue
        lag_x, corr = xcorr_lag(rz_t, rz, t, x, a.max_lag_ms / 1000)
        lag_s, rel_res = sine_lag(rz_t, rz, t, x, f)
        res[name] = {'lag_xcorr_ms': lag_x * 1000, 'xcorr_abs': corr, 'lag_sine_ms': lag_s * 1000,
                     'sine_fit_rel_residual': rel_res, 'frames': int(len(x)),
                     'time_offset_s': -lag_x, 'method': 'aruco' if (name == cam and a.aruco) else
                     ('depth_roi' if name == cam else 'bright_structure')}
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', help='one bag, or start and end bags of a session')
    ap.add_argument('--us-rows', type=int, nargs=2, help='image row band of a moving bright structure (ultrasound)')
    ap.add_argument('--us-cols', type=int, nargs=2)
    ap.add_argument('--aruco', help='e.g. DICT_4X4_50 (marker fixed on the phantom / table)')
    ap.add_argument('--marker-mm', type=float, default=40.0)
    ap.add_argument('--depth-roi', type=int, nargs=4)
    ap.add_argument('--camera', choices=['wrist_camera', 'tpv_camera'], default='wrist_camera',
                    help='which camera the colour / depth topics are from (result key, default topics)')
    ap.add_argument('--color-topic')
    ap.add_argument('--color-info-topic')
    ap.add_argument('--depth-topic')
    ap.add_argument('--max-lag-ms', type=float, default=200.0)
    ap.add_argument('--session', help='write the result into <session>/session.json')
    a = ap.parse_args()
    ns = {'wrist_camera': '/wrist/d415', 'tpv_camera': '/tpv/camera'}[a.camera]   # session.yaml <camera>.topics
    a.color_topic = a.color_topic or f'{ns}/color/image_raw'
    a.color_info_topic = a.color_info_topic or f'{ns}/color/camera_info'
    a.depth_topic = a.depth_topic or (f'{ns}/aligned_depth_to_color/image_raw' if a.camera == 'wrist_camera' else '')
    if a.camera == 'tpv_camera' and a.depth_roi:
        sys.exit('--depth-roi needs depth: the TPV camera records colour only (use --aruco)')
    if not a.us_rows and not a.aruco and not a.depth_roi:
        sys.exit('nothing to calibrate: give --us-rows (ultrasound) and / or --aruco / --depth-roi (camera)')
    if len(a.bags) > 2:
        sys.exit('give one bag, or start and end bags')
    results = [calibrate(b, a) for b in a.bags]
    for r in results:
        print(f'{r["bag"]}: robot sinusoid {r["robot_freq_hz"]:.3f} Hz')
        for name in ('ultrasound', a.camera):
            if name in r:
                x = r[name]
                print(f'  {name:13s} lag xcorr {x["lag_xcorr_ms"]:7.1f} ms (|corr| {x["xcorr_abs"]:.3f}), '
                      f'sine fit {x["lag_sine_ms"]:7.1f} ms (rel. residual {x["sine_fit_rel_residual"]:.2f}), '
                      f'{x["frames"]} frames -> time_offset_s {x["time_offset_s"]:.4f}')
    out = {'start': results[0]}
    if len(results) == 2:
        out['end'] = results[1]
        out['drift_ms'] = {n: results[1][n]['lag_xcorr_ms'] - results[0][n]['lag_xcorr_ms']
                           for n in ('ultrasound', a.camera) if n in results[0] and n in results[1]}
        print(f'drift start -> end: {json.dumps({k: round(v, 1) for k, v in out["drift_ms"].items()})} ms')
    if a.session:
        sys.path.insert(0, str(REPO / 'src/usrig_common'))
        from usrig_common.session import update_session_json
        update_session_json(a.session, temporal_calibration=out)
        print(f'stored in {Path(a.session) / "session.json"}')


if __name__ == '__main__':
    main()
