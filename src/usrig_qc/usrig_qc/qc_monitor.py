"""qc_monitor: live data health on /qc/status (std_msgs/String JSON, 2 Hz).

Per topic: rate over the last window, age of the newest header stamp vs system clock, gaps.
Plus micrus frame_index gaps, coupling (mean B-mode intensity), probe-axis force vs band,
free disk, and the wrist camera's USB speed and reconnect count (sysfs).
All subscriptions are best effort: QC can never slow down a publisher. Messages arrive raw
(serialized): rates and stamps come from the CDR header without decoding, and only a subsample is
decoded for content checks (robot 1 in 25, B-mode 1 in 3, depth 1 in 10), which keeps this node's
CPU use low on a loaded capture PC (it used ~50 % of a core decoding every robot sample).
"""
import json
import shutil
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from usrig_common import config
from usrig_common.timeutil import now_ns, ns_from_stamp
from usrig_interfaces.msg import RobotState

from .stream_check import stamp_from_cdr
from .usb import realsense_usb

DECODE_EVERY = {'/robot/state': 25, '/micrus/bmode': 3, 'depth': 10}

WINDOW_S = 2.0


class TopicStats:

    def __init__(self):
        self.stamps = deque()
        self.recv = deque()
        self.last_stamp = None
        self.max_gap_ms = 0.0

    def add(self, stamp_ns):
        r = now_ns()
        if self.last_stamp is not None:
            self.max_gap_ms = max(self.max_gap_ms, (stamp_ns - self.last_stamp) * 1e-6)
        self.last_stamp = stamp_ns
        self.stamps.append(stamp_ns)
        self.recv.append(r)

    def report(self):
        t = now_ns()
        while self.recv and t - self.recv[0] > WINDOW_S * 1e9:
            self.recv.popleft()
            self.stamps.popleft()
        n = len(self.stamps)
        hz = (n - 1) / ((self.stamps[-1] - self.stamps[0]) * 1e-9) if n > 1 and self.stamps[-1] > self.stamps[0] else 0.0
        out = {'hz': round(hz, 2), 'stamp_age_ms': round((t - self.last_stamp) * 1e-6, 2) if self.last_stamp else None,
               'max_gap_ms': round(self.max_gap_ms, 2)}
        self.max_gap_ms = 0.0
        return out


class QcMonitor(Node):

    def __init__(self):
        super().__init__('qc_monitor')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('session_id', '')
        self.declare_parameter('sim', False)
        cfg_dir = self.get_parameter('config_dir').value or None
        self.scfg = config.load_config('session', cfg_dir)
        self.rig = config.load_config('rig', cfg_dir)
        cam = self.scfg['wrist_camera']
        self.cam_on = bool(cam.get('enabled'))
        self.serial = cam.get('serial')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.lock = threading.Lock()
        self.stats = {}
        self.frame_gaps = 0
        self.last_frame_index = None
        self.coupling = deque(maxlen=60)
        self.force = None
        self.valid_depth = None
        self._depth_n = 0

        self.depth_topic = cam['topics']['depth']
        self._n = {}
        self._raw(Image, '/micrus/bmode', self._on_bmode, DECODE_EVERY['/micrus/bmode'], qos)
        self.stats['/micrus/frame_info'] = TopicStats()
        self.create_subscription(String, '/micrus/frame_info', self._on_frame_info, qos)
        self._raw(RobotState, '/robot/state', self._on_robot, DECODE_EVERY['/robot/state'], qos)
        if self.cam_on:
            self._raw(Image, cam['topics']['color'], None, 0, qos)
            self._raw(Image, self.depth_topic, self._on_depth, DECODE_EVERY['depth'], qos)

        self.usb_speed = None
        self.usb_reconnects = 0
        self._usb_present = None
        self.pub = self.create_publisher(String, '/qc/status', 10)
        self.create_timer(0.5, self._publish)
        self.create_timer(1.0, self._poll_usb)

    def _raw(self, typ, topic, cb, every, qos):
        """Raw subscription: stamp from the CDR header for every message, decode 1 in `every`."""
        from rclpy.serialization import deserialize_message
        self.stats[topic] = TopicStats()
        self._n[topic] = 0

        def f(b, topic=topic):
            with self.lock:
                self.stats[topic].add(stamp_from_cdr(b))
            self._n[topic] += 1
            if cb is not None and every and self._n[topic] % every == 0:
                cb(deserialize_message(b, typ))
        self.create_subscription(typ, topic, f, qos, raw=True)

    def _on_bmode(self, m):
        mean = float(np.frombuffer(bytes(m.data), np.uint8).mean())
        with self.lock:
            self.coupling.append(mean)

    def _on_frame_info(self, m):
        with self.lock:
            self.stats['/micrus/frame_info'].add(now_ns())
        try:
            fi = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            idx = fi.get('frame_index')
            if self.last_frame_index is not None and idx is not None and idx != self.last_frame_index + 1:
                self.frame_gaps += 1
            self.last_frame_index = idx

    def _on_robot(self, m):
        o = m.tcp_base.orientation
        x, y, z, w = o.x, o.y, o.z, o.w
        zaxis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        with self.lock:
            self.force = float(zaxis @ np.asarray(m.wrench[:3]))
            self.mode = int(m.mode)
            self.flags = int(m.safety_flags)

    def _on_depth(self, m):
        frac = float(np.mean(np.frombuffer(bytes(m.data), np.uint16) > 0))
        with self.lock:
            self.valid_depth = frac

    def _poll_usb(self):
        if not self.cam_on:
            return
        devs = realsense_usb(self.serial)
        present = bool(devs)
        if self._usb_present and not present:
            self.usb_reconnects += 1
        self._usb_present = present
        self.usb_speed = devs[0]['speed_mbps'] if devs else None

    def _publish(self):
        from usrig_common.session import data_root
        root = data_root(self.scfg, self.get_parameter('sim').value)
        while not root.exists():
            root = root.parent
        with self.lock:
            topics = {t: s.report() for t, s in self.stats.items()}
            band = config.get_path(self.rig, 'limits.force_band_n')
            floor = config.get_path(self.rig, 'qc.coupling_floor')
            st = {
                'stamp_ns': now_ns(),
                'topics': topics,
                'micrus_frame_gaps': self.frame_gaps,
                'micrus_frame_index': self.last_frame_index,
                'coupling_mean': float(np.mean(self.coupling)) if self.coupling else None,
                'coupling_ok': (float(np.mean(self.coupling)) > floor) if (self.coupling and floor is not None) else None,
                'force_axis_n': self.force,
                'force_in_band': (band[0] <= abs(self.force) <= band[1]) if (band and self.force is not None) else None,
                'free_disk_gb': round(shutil.disk_usage(root).free / 1e9, 1),
                'wrist_valid_depth_frac': self.valid_depth,
                'wrist_usb': {'speed_mbps': self.usb_speed, 'reconnects': self.usb_reconnects} if self.cam_on else None,
            }
        m = String()
        m.data = json.dumps(st)
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    n = QcMonitor()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
