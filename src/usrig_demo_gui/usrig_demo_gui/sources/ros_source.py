"""Read-only ROS 2 adapter (DRAFT, not yet run against the rig).

Subscribes to the topics the OpenH repo publishes and fills a DashboardState. It never publishes,
never calls a service and never starts or stops anything; killing the GUI cannot affect
recording or control (same rule as usrig_dashboard).

What comes from where:
  /robot/state        usrig_interfaces/RobotState  -> joints, TCP pose, mode, safety flags, wrench
  /micrus/bmode       sensor_msgs/Image (mono8)    -> B-mode image
  /micrus/frame_info  std_msgs/String (JSON)       -> frame index, pixel spacing, scan settings
  wrist color/depth   sensor_msgs/Image            -> wrist camera views (topics from demo_gui.yaml)
  /qc/status          std_msgs/String (JSON)       -> topic rates, coupling, frame gaps, disk, USB
  /robot/control_status usrig_interfaces/ControlStatus -> arrival rate only (counted raw, never decoded)
  /episode/events     usrig_interfaces/EpisodeEvent-> phase, outcome, recording, event log
  ROS graph           node names                   -> node up/down
  /episode/status     std_msgs/String (JSON)       -> demo only: is a real collection session running?

Demo only: while episode_manager runs a real (non-sim) session, every subscription above except
/episode/status is destroyed and the window shows a paused screen (collection_guard.py), so the
GUI costs nothing during dataset collection. It resumes by itself when that session ends.

Items marked NOT VERIFIED in docs/INTEGRATION.md must be checked on the capture PC.
"""
from __future__ import annotations

import json
import threading
import time
from collections import deque

import numpy as np

from .. import kinematics as kin
from .. import metrics as M
from ..collection_guard import collection_lock
from ..frames import ToolCalibration, pose_from_pq, tool_frames
from ..models import (CameraFrame, ContactState, DashboardState, ForceTorque, Health, Metric, NodeHealth,
                      PlaneState, RecordingState, RobotState, ScanMetrics, StreamStatus, SystemState, TaskState,
                      UltrasoundFrame)
from .base import DataSource, expected_text, rate_health
from .episode_tracker import EpisodeTracker

MODES = ['IDLE', 'MOVING', 'DESCENT', 'TELEOP', 'PERTURB', 'RETRACT', 'FAULT', 'FREEDRIVE']   # RobotState.msg
FLAGS = {1: 'envelope', 2: 'tilt', 4: 'force_max', 8: 'deadman', 16: 'speed'}
STALE_S = 1.0
DISCOVERY_S = 2.0         # graph not trusted this soon after the node starts (DDS discovery)
GUARD_PERIOD_S = 1.0


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def episode_paths(ep_id, status, data_root='~/data'):
    """episode_id '<session>_ep_NNNN' -> (session_id, 'ep_NNNN', output path).

    The path is session.session_dir from /episode/status (episode_manager's own folder, so a sim
    session shows its ~/data_sim path); without it, <data_root>/sessions/<session>/ep_NNNN/ as in
    usrig_common.session.session_dir.
    """
    if not ep_id or '_ep_' not in ep_id:
        return None, None, None
    sid, num = ep_id.rsplit('_ep_', 1)
    ep = f'ep_{num}'
    session = (status or {}).get('session') or {}
    sdir = session.get('session_dir') if session.get('session_id') == sid else None
    if sdir:
        return sid, ep, f'{str(sdir).rstrip("/")}/{ep}/'
    root = (data_root or '~/data').rstrip('/')
    return sid, ep, f'{root}/sessions/{sid}/{ep}/'


def image_to_array(m):
    """sensor_msgs/Image -> numpy (same encodings the existing dashboard handles)."""
    if m.encoding in ('mono8', '8UC1'):
        return np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width]
    if m.encoding in ('16UC1', 'mono16'):
        return np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
    if m.encoding in ('rgb8', 'bgr8'):
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
        a = a.reshape(m.height, m.width, 3)
        return a[:, :, ::-1] if m.encoding == 'bgr8' else a
    return None


class RosSource(DataSource):
    label = 'ros'
    is_mock = False

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.latest = {}
        self.stamp = {}                                   # monotonic arrival time per stream
        self.arrivals = {k: deque(maxlen=60) for k in ('bmode', 'color', 'depth', 'robot', 'qc', 'control')}
        n = int(float(cfg.get('force_window_s', 10)) * 125)
        self.ft = deque(maxlen=n)                        # (t_s, wrench_tool[6]), decimated to ~125 Hz
        self.tracker = EpisodeTracker(cfg.get('phase_map'), cfg.get('outcome_map'))
        self.nodes_up = set()
        self.node = None
        self.executor = None
        self.spin_thread = None
        self._robot_n = 0
        self.rig = {}                                    # rig.yaml via usrig_common.config (tcp_setting, probe, hand-eye)
        self.T_flange_camera_tf = None                   # flange -> wrist_camera from /tf_static
        self.subs = []                                   # display subscriptions (dropped during collection)
        self.ep_status = None                            # last /episode/status payload and its arrival time
        self.ep_status_t = None
        self.paused_reason = None                        # set by start() / _guard_tick(); None = live
        self.t_node = None

    # ------------------------------------------------------------ ROS side
    def start(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        if not rclpy.ok():
            rclpy.init()
        self.node = Node('demo_gui')
        self.t_node = time.monotonic()
        self.paused_reason = collection_lock(None, None, None)    # paused until the graph has been read
        # the only subscription that always exists: episode_manager's latched 2 Hz status (session.sim)
        self.node.create_subscription(
            String, self.cfg['topics'].get('episode_status', '/episode/status'), self._on_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        try:                                             # same loader as the other usrig nodes
            from usrig_common import config as ucfg
            self.rig = ucfg.load_config('rig', self.cfg.get('rig_config_dir') or None) or {}
        except Exception as e:
            self.node.get_logger().warn(f'rig.yaml not loaded ({e}); flange / tip / camera paths unavailable')
        # runs in the executor thread, so subscriptions are created / destroyed there, never mid-callback
        self.node.create_timer(GUARD_PERIOD_S, self._guard_tick)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def _subscribe_display(self):
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from usrig_interfaces.msg import ControlStatus, EpisodeEvent, RobotState as RobotStateMsg

        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        t = self.cfg['topics']
        sub = self.node.create_subscription
        self.subs = [
            sub(RobotStateMsg, t['robot_state'], self._on_robot,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)),
            sub(Image, t['bmode'], lambda m: self._img('bmode', m), best),
            sub(String, t['frame_info'], lambda m: self._json('frame_info', m.data), best),
            sub(Image, t['color'], lambda m: self._img('color', m), best),
            sub(Image, t['depth'], lambda m: self._img('depth', m), best),
            sub(String, t['qc_status'], lambda m: self._json('qc', m.data), best),
            # 125 Hz: only the arrival rate is needed, so the message is never deserialized (as qc_monitor)
            sub(ControlStatus, t.get('control_status', '/robot/control_status'), self._on_control, best, raw=True),
            sub(EpisodeEvent, t['episode_events'], self._on_event,
                QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)),
            # trajectory map: controller TCP pose + static calibrations
            sub(PoseStamped, t.get('tcp_pose', '/robot/tcp_pose'), self._on_tcp_pose, best),
            sub(TFMessage, t.get('tf_static', '/tf_static'), self._on_tf_static,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]

    def _unsubscribe_display(self):
        for s_ in self.subs:
            self.node.destroy_subscription(s_)
        self.subs = []
        with self.lock:                                  # nothing from before the pause is shown afterwards
            self.latest.clear()
            self.stamp.clear()
            for dq in self.arrivals.values():
                dq.clear()
            self.ft.clear()
            self.tracker = EpisodeTracker(self.cfg.get('phase_map'), self.cfg.get('outcome_map'))

    def _on_status(self, m):
        try:
            v = json.loads(m.data)
        except ValueError:
            return
        with self.lock:
            self.ep_status, self.ep_status_t = v, time.monotonic()

    def _guard_tick(self):
        """Read the ROS graph, decide paused / live, and add or drop the display subscriptions."""
        now = time.monotonic()
        nodes = None
        if now - self.t_node >= DISCOVERY_S:
            try:
                nodes = {n for n, _ in self.node.get_node_names_and_namespaces()}
            except Exception:
                nodes = None
        with self.lock:
            status = self.ep_status
            age = now - self.ep_status_t if self.ep_status_t is not None else None
        reason = collection_lock(nodes, status, age)
        if reason and self.subs:
            self._unsubscribe_display()
            self.node.get_logger().info(f'demo GUI paused: {reason}')
        elif reason is None and not self.subs:
            self._subscribe_display()
            self.node.get_logger().info('demo GUI live (no real collection session running)')
        with self.lock:
            self.nodes_up = nodes or set()
            self.paused_reason = reason

    def stop(self):
        try:
            if self.executor:
                self.executor.shutdown()
            # spin() must have returned before the node and context go away; otherwise the process
            # aborts on exit ("terminate called without an active exception")
            if self.spin_thread is not None:
                self.spin_thread.join(timeout=2.0)
            if self.node:
                self.node.destroy_node()
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def _img(self, k, m):
        now = time.monotonic()
        with self.lock:
            self.latest[k] = m
            self.stamp[k] = now
            self.arrivals[k].append(now)

    def _json(self, k, s):
        try:
            v = json.loads(s)
        except ValueError:
            return
        now = time.monotonic()
        with self.lock:
            self.latest[k] = v
            self.stamp[k] = now
            if k in self.arrivals:
                self.arrivals[k].append(now)

    def _on_control(self, _raw):
        now = time.monotonic()
        with self.lock:
            self.stamp['control'] = now
            self.arrivals['control'].append(now)

    def _on_robot(self, m):
        now = time.monotonic()
        self._robot_n += 1
        with self.lock:
            self.latest['robot'] = m
            self.stamp['robot'] = now
            self.arrivals['robot'].append(now)
            if self._robot_n % 4 == 0:                    # 500 Hz -> 125 Hz for the chart
                o = m.tcp_base.orientation
                R = quat_to_R(o.x, o.y, o.z, o.w)
                w = np.asarray(m.wrench, float)
                # RobotState.wrench is in the base frame (see RobotState.msg); rotate into the tool frame.
                tool = np.concatenate([R.T @ w[:3], R.T @ w[3:]])
                self.ft.append((now, tool))

    def _on_tcp_pose(self, m):
        p, o = m.pose.position, m.pose.orientation
        T = pose_from_pq([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
        with self.lock:
            self.latest['tcp_pose'] = T
            self.stamp['tcp_pose'] = time.monotonic()

    def _on_tf_static(self, m):
        fr = self.cfg.get('frames') or {}
        parent, child = fr.get('flange', 'flange'), fr.get('wrist_camera', 'wrist_camera')
        for t in m.transforms:
            if t.header.frame_id.lstrip('/') == parent and t.child_frame_id.lstrip('/') == child:
                tr, q = t.transform.translation, t.transform.rotation
                with self.lock:
                    self.T_flange_camera_tf = pose_from_pq([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])

    def _on_event(self, m):
        from usrig_common.timeutil import ns_from_stamp   # repo helper, same as the dashboard
        with self.lock:
            self.tracker.handle(m.kind, m.value, ns_from_stamp(m.header.stamp), m.episode_id)

    # ------------------------------------------------------------ snapshot
    def _fresh(self, k, now):
        stale = 3.0 if k == 'qc' else STALE_S                 # /qc/status is 2 Hz
        return k in self.stamp and now - self.stamp[k] < stale

    def snapshot(self) -> DashboardState:
        now = time.monotonic()
        now_ns = time.time_ns()
        with self.lock:
            if self.paused_reason:
                return DashboardState(source_label='ROS 2 (read-only)', is_mock=False, system=SystemState.IDLE,
                                      paused_reason=self.paused_reason)
            L = dict(self.latest)
            fresh = {k: self._fresh(k, now) for k in ('robot', 'bmode', 'color', 'depth', 'qc', 'frame_info',
                                                      'tcp_pose', 'control')}
            cam_tf = self.T_flange_camera_tf
            rates = {k: (self._rate(v) if fresh[k] else None) for k, v in self.arrivals.items()}
            ft = list(self.ft)
            tr = self.tracker
            task = TaskState(phase=tr.phase, outcome=tr.outcome, events=list(tr.events[-4:]),
                             detail=(tr.info or {}).get('instruction') or '')
            elapsed = tr.elapsed_s(now_ns)
            ep_id = tr.episode_id
            nodes_up = set(self.nodes_up)
            recorder_up = ('rosbag2_recorder' in nodes_up) if nodes_up else None
            rec_status = tr.recording_status(now_ns, recorder_up)
            ep_status = self.ep_status

        d = DashboardState(source_label='ROS 2 (read-only)', is_mock=False)
        qc = L.get('qc') if fresh['qc'] else None

        # robot
        m = L.get('robot') if fresh['robot'] else None
        fz_axis = None
        if m is not None:
            p, o = m.tcp_base.position, m.tcp_base.orientation
            R = quat_to_R(o.x, o.y, o.z, o.w)
            # trajectory map: /robot/tcp_pose (same sample as tcp_base), else tcp_base from /robot/state
            T_tcp = L['tcp_pose'] if fresh['tcp_pose'] else pose_from_pq([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])
            cal = ToolCalibration.from_rig(self.rig, cam_tf)
            fr = tool_frames(T_tcp, cal)
            d.robot = RobotState(connected=True, joints_rad=np.asarray(m.q, float),
                                 tcp_pos_m=np.array([p.x, p.y, p.z]), tcp_rotvec=kin.rotvec_from_matrix(R),
                                 mode=MODES[m.mode] if m.mode < len(MODES) else str(m.mode),
                                 safety_flags=[n for b, n in FLAGS.items() if m.safety_flags & b],
                                 T_flange=fr.flange, T_tip=fr.tip, T_camera=fr.camera,
                                 frames_missing=cal.missing())
            fz_axis = float(R[:, 2] @ np.asarray(m.wrench[:3], float))     # same as qc_monitor / dashboard
        if ft:
            t = np.array([x[0] for x in ft])
            d.ft = ForceTorque(t=t - now, values=np.array([x[1] for x in ft]))

        # contact / plane
        thr = self.cfg.get('contact_force_threshold_n')
        if fz_axis is None or thr is None:
            d.contact = ContactState.UNKNOWN
        else:
            d.contact = ContactState.IN_CONTACT if abs(fz_axis) > float(thr) else ContactState.NO_CONTACT
        d.plane = PlaneState.UNKNOWN          # no plane estimator in the repo yet

        # ultrasound
        fi = L.get('frame_info') or {}
        img = None
        if 'bmode' not in L:
            us_status = StreamStatus.LOADING if self.node else StreamStatus.DISCONNECTED
        elif not fresh['bmode']:
            us_status = StreamStatus.DISCONNECTED
        else:
            img = image_to_array(L['bmode'])
            us_status = StreamStatus.LIVE if img is not None else StreamStatus.NO_IMAGE
        ps = (fi.get('image') or {}).get('pixel_spacing_mm')
        if isinstance(ps, (int, float)):
            ps = (ps, ps)
        # micrus frame_info keys (checked with micrus --simulate): settings.depth_mm, settings.frequency_hz
        settings = fi.get('settings') or {}
        depth_mm = settings.get('depth_mm') or settings.get('depth') or \
            (img.shape[0] * ps[1] if (img is not None and ps) else None)
        f_hz = settings.get('frequency_hz')
        freq_mhz = f_hz / 1e6 if isinstance(f_hz, (int, float)) else (settings.get('freq') or settings.get('frequency'))
        d.ultrasound = UltrasoundFrame(status=us_status, image=img, pixel_mm=tuple(ps[:2]) if ps else (1.0, 1.0),
                                       fps=rates['bmode'], frame_index=fi.get('frame_index'),
                                       gain=settings.get('gain'), depth_mm=depth_mm,
                                       freq_mhz=freq_mhz)

        # wrist cameras
        d.rgb = self._camera(L, 'color', fresh['color'], rates['color'])
        d.depth = self._camera(L, 'depth', fresh['depth'], rates['depth'])

        # metrics
        items = [Metric('Contact force', abs(fz_axis) if fz_axis is not None else None, 'N', '{:.2f}',
                        'robot wrench', Health.UNKNOWN, 'Force along the probe axis (tool z), from /robot/state.'),
                 Metric('Probe-tissue angle', None, 'deg', note='Needs phantom registration (tcp_phantom).'),
                 Metric('In-plane score', None, note='No plane estimator in the repo yet.')]
        snr = None
        if img is not None:
            h, w = img.shape
            snr = M.speckle_snr(img, (int(h * 0.75), int(h * 0.9), int(w * 0.25), int(w * 0.75)))
        items.append(Metric('Speckle SNR', snr, '', '{:.2f}', 'image ROI', Health.UNKNOWN,
                            'mean/std of displayed intensity in a fixed deep ROI. Relative indicator only.'))
        items.append(Metric('CNR (vessel)', None, note='Needs a vessel ROI or segmentation.'))
        band = self.cfg.get('force_band_n')
        if band and fz_axis is not None and d.contact == ContactState.IN_CONTACT:
            items[0].level = Health.OK if band[0] <= abs(fz_axis) <= band[1] else Health.WARN
        q, ql = None, Health.UNKNOWN
        ok = qc.get('coupling_ok') if qc else None
        if ok is None and img is not None and self.cfg.get('coupling_floor') is not None:
            ok = M.coupling_mean(img) > float(self.cfg['coupling_floor'])
        if ok is not None:
            q, ql = ('Good', Health.OK) if ok else ('Low coupling', Health.WARN)
        d.metrics = ScanMetrics(items=items, image_quality=q, image_quality_level=ql)

        # task / recording
        cap = self.cfg.get('episode_cap_s')
        task.progress = min(elapsed / float(cap), 1.0) if (elapsed is not None and cap) else None
        d.task = task
        sid, ep, out = episode_paths(ep_id, ep_status, self.cfg.get('data_root'))
        d.recording = RecordingState(status=rec_status, session_id=sid, episode_id=ep,
                                     elapsed_s=elapsed if rec_status.name in ('RECORDING', 'FINALIZING') else None,
                                     total_s=float(cap) if cap else None,
                                     output_path=out, warnings=self._warnings(qc))

        # nodes
        topic_hz = (qc or {}).get('topics', {})
        tp = self.cfg['topics']
        own = {tp['bmode']: rates['bmode'], tp['color']: rates['color'], tp['depth']: rates['depth'],
               tp['robot_state']: rates['robot'], tp['qc_status']: rates['qc'],
               tp.get('control_status', '/robot/control_status'): rates['control']}
        for n in self.cfg.get('nodes', []):
            topic = n.get('topic', '')
            rate = (topic_hz.get(topic) or {}).get('hz') if topic in topic_hz else own.get(topic)
            up = (n.get('node') in nodes_up) if nodes_up else None
            exp = n.get('expected_hz')
            if n.get('node') == 'rosbag2_recorder':
                h = Health.OK if up else Health.UNKNOWN
                d.nodes.append(NodeHealth(n['label'], topic, None, 'recording' if up else 'idle', h))
                continue
            h = Health.UNKNOWN if up is None else rate_health(rate, exp, up)
            d.nodes.append(NodeHealth(n['label'], topic, rate, expected_text(exp), h))

        # system
        if not fresh['robot'] and not fresh['bmode']:
            d.system = SystemState.DISCONNECTED
        elif (m is not None and (d.robot.mode == 'FAULT' or d.robot.safety_flags)) or \
                any(x.health in (Health.WARN, Health.DOWN) for x in d.nodes):
            d.system = SystemState.DEGRADED
        elif (m is not None and d.robot.mode not in ('IDLE', None)) or rec_status.name == 'RECORDING':
            d.system = SystemState.RUNNING
        else:
            d.system = SystemState.IDLE
        return d

    def _camera(self, L, k, fresh, rate):
        if k not in L:
            return CameraFrame(status=StreamStatus.LOADING if self.node else StreamStatus.DISCONNECTED)
        if not fresh:
            return CameraFrame(status=StreamStatus.DISCONNECTED)
        a = image_to_array(L[k])
        if a is None:
            return CameraFrame(status=StreamStatus.NO_IMAGE, fps=rate)
        if k == 'depth':
            a = a.astype(np.float32) / 1000.0            # D415 aligned depth: uint16 millimetres
            return CameraFrame(status=StreamStatus.LIVE, image=a, fps=rate, depth_range_m=(0.0, 0.5))
        return CameraFrame(status=StreamStatus.LIVE, image=a, fps=rate)

    def _warnings(self, qc):
        if not qc:
            return []
        w = []
        if qc.get('micrus_frame_gaps'):
            w.append(f'micrus frame gaps: {qc["micrus_frame_gaps"]} (QC)')
        v = qc.get('wrist_valid_depth_frac')
        if v is not None and v < 0.8:
            w.append(f'wrist depth: {v:.0%} valid pixels')
        usb = qc.get('wrist_usb') or {}
        if usb.get('reconnects'):
            w.append(f'wrist camera USB reconnects: {usb["reconnects"]}')
        if usb.get('speed_mbps') is not None and usb['speed_mbps'] < 5000:
            w.append(f'wrist camera on USB {usb["speed_mbps"]} Mb/s (not USB 3)')
        if qc.get('force_in_band') is False:
            w.append('contact force outside the band')
        disk = qc.get('free_disk_gb')
        if disk is not None and disk < 50:
            w.append(f'free disk {disk} GB')
        return w

    @staticmethod
    def _rate(dq):
        if len(dq) < 2:
            return None
        return (len(dq) - 1) / max(dq[-1] - dq[0], 1e-6)

