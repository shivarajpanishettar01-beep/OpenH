"""Measure live streams for the P1 / P1b / P2 checks. Reports real numbers only.

  ros2 run usrig_qc stream_check --duration 300 --micrus            # P1: rate, frame_index gaps
  ros2 run usrig_qc stream_check --duration 600 --camera            # P1b: rates, stamp - clock, jitter
  ros2 run usrig_qc stream_check --duration 10 --camera --depth-roi 200 140 440 340   # depth at contact
  ros2 run usrig_qc stream_check --duration 60 --robot              # P2: 500 Hz, ctrl time, jitter
  add --json out.json to save the result.

stamp - clock: header stamp minus the system clock when the message arrived here (the
subscriber adds its own latency, so this is an upper bound on how late stamps are).
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from usrig_common import config
from usrig_common.timeutil import now_ns, ns_from_stamp


def stamp_from_cdr(b):
    """header.stamp of a serialized message whose first field is std_msgs/Header (CDR)."""
    little = b[1] == 1                   # encapsulation: 0x0001 = CDR little endian
    sec = int.from_bytes(b[4:8], 'little' if little else 'big', signed=True)
    nsec = int.from_bytes(b[8:12], 'little' if little else 'big')
    return sec * 1_000_000_000 + nsec


def summarize(stamps, recv):
    s = np.asarray(stamps, np.int64)
    r = np.asarray(recv, np.int64)
    if len(s) < 2:
        return {'n': int(len(s))}
    dt = np.diff(s) * 1e-6
    off = (s - r) * 1e-6
    return {'n': int(len(s)), 'hz': float((len(s) - 1) / ((s[-1] - s[0]) * 1e-9)),
            'interval_ms_median': float(np.median(dt)), 'interval_ms_max': float(dt.max()),
            'jitter_ms_std': float(dt.std()), 'non_increasing_stamps': int(np.sum(dt <= 0)),
            'stamp_minus_clock_ms': {'median': float(np.median(off)), 'p99': float(np.percentile(off, 99)),
                                     'p1': float(np.percentile(off, 1))}}


class Checker(Node):

    def __init__(self, a, scfg):
        super().__init__('stream_check')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.data = {}
        self.extra = {}
        self.a = a
        if a.micrus:
            self._track('/micrus/bmode', Image, qos)
            self.extra['frame_index'] = []
            self.extra['settings'] = set()
            self.create_subscription(String, '/micrus/frame_info', self._fi, qos)
        if a.camera:
            ct = scfg['wrist_camera']['topics']
            self._track(ct['color'], Image, qos)
            self._track(ct['depth'], Image, qos, cb=self._depth)
            self.extra['depth_valid'] = []
            self.extra['depth_median_mm'] = []
        if a.robot:
            from usrig_interfaces.msg import RobotState
            self._track('/robot/state', RobotState, qos, cb=self._robot)
            self.extra['ctrl'] = []

    def _track(self, topic, typ, qos, cb=None):
        self.data[topic] = ([], [])
        if cb is None:
            # Stamp only: take raw CDR bytes, never deserialize the payload. Decoding large
            # images in Python makes this checker fall behind and lose frames itself.
            def f_raw(b, t=topic):
                self.data[t][1].append(now_ns())
                self.data[t][0].append(stamp_from_cdr(b))
            self.create_subscription(typ, topic, f_raw, qos, raw=True)
            return

        def f(m, t=topic):
            self.data[t][0].append(ns_from_stamp(m.header.stamp))
            self.data[t][1].append(now_ns())
            cb(m)
        self.create_subscription(typ, topic, f, qos)

    def _fi(self, m):
        fi = json.loads(m.data)
        self.extra['frame_index'].append(fi.get('frame_index'))
        self.extra['settings'].add(json.dumps(fi.get('settings'), sort_keys=True))

    def _depth(self, m):
        if len(self.extra['depth_valid']) and len(self.data[self.a_depth_topic()][0]) % 5:
            return
        d = np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
        if self.a.depth_roi:
            x0, y0, x1, y1 = self.a.depth_roi
            d = d[y0:y1, x0:x1]
        v = d[d > 0]
        self.extra['depth_valid'].append(float(np.mean(d > 0)))
        self.extra['depth_median_mm'].append(float(np.median(v)) if v.size else None)

    def a_depth_topic(self):
        return [t for t in self.data if 'depth' in t][0]

    def _robot(self, m):
        self.extra['ctrl'].append(m.controller_time_s)

    def result(self):
        out = {t: summarize(*v) for t, v in self.data.items()}
        if 'frame_index' in self.extra:
            idx = np.array([i for i in self.extra['frame_index'] if i is not None])
            out['micrus_frame_index'] = {'frames': int(len(idx)),
                                         'gaps': int(np.sum(np.diff(idx) != 1)) if len(idx) > 1 else None,
                                         'distinct_settings': len(self.extra['settings'])}
        if 'ctrl' in self.extra and self.extra['ctrl']:
            c = np.array(self.extra['ctrl'])
            st = np.array(self.data['/robot/state'][0]) * 1e-9
            d = (st - c)
            d = (d - np.median(d)) * 1e3
            out['robot_controller_time'] = {
                'repeated': int(np.sum(np.diff(c) <= 0)),
                'stamp_minus_ctrl_jitter_ms': {'median_abs': float(np.median(np.abs(d))),
                                               'p99_abs': float(np.percentile(np.abs(d), 99))}}
        if self.extra.get('depth_valid'):
            med = [x for x in self.extra['depth_median_mm'] if x is not None]
            out['wrist_depth'] = {'valid_frac_mean': float(np.mean(self.extra['depth_valid'])),
                                  'median_depth_mm': float(np.median(med)) if med else None,
                                  'roi': self.a.depth_roi}
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--micrus', action='store_true')
    ap.add_argument('--camera', action='store_true')
    ap.add_argument('--robot', action='store_true')
    ap.add_argument('--depth-roi', type=int, nargs=4, metavar=('X0', 'Y0', 'X1', 'Y1'))
    ap.add_argument('--config-dir')
    ap.add_argument('--json')
    a, _ = ap.parse_known_args()
    rclpy.init()
    n = Checker(a, config.load_config('session', a.config_dir))
    t_end = time.monotonic() + a.duration
    try:
        while time.monotonic() < t_end:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    r = n.result()
    print(json.dumps(r, indent=2))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(r, f, indent=2)
    n.destroy_node()
    rclpy.shutdown()
