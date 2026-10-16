"""Per-episode recorder: one MCAP bag per episode + the micrus HDF5.

Order (section P4):
  1. start ``ros2 bag record -s mcap -o <session>/<ep_NNNN>`` (zstd chunk compression)
  2. readiness gate: the bag has subscribed to every topic and each ready topic has produced a
     message after that -> t_bag_ready_ns (the caller then publishes the ``start`` event)
  3. /micrus/record true  -> HDF5 path from response.message
  stop: (caller publishes ``end``) -> SIGINT the bag and wait -> /micrus/record false
  The caller writes task.txt and episode.json only after the bag has stopped.

Crash safety:
  * the bag process gets PR_SET_PDEATHSIG=SIGINT, so it closes cleanly if the manager dies;
  * a detached record_guard watches the manager PID and stops a micrus recording left on;
  * atexit / signal handlers in episode_manager call ``abort()``.
"""
import ctypes
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import yaml
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

from usrig_common.timeutil import now_ns

BAG_NODE_NAMES = ('rosbag2_recorder',)
PR_SET_PDEATHSIG = 1


def _pdeathsig():
    libc = ctypes.CDLL('libc.so.6', use_errno=True)
    libc.prctl(PR_SET_PDEATHSIG, signal.SIGINT)


class RecorderError(RuntimeError):
    pass


class EpisodeRecorder:

    def __init__(self, node, session_cfg, lock_dir):
        self.node = node
        self.cfg = session_cfg
        rec = session_cfg['recording']
        cam = session_cfg['wrist_camera']
        self.topics = list(rec['topics'])
        if rec.get('record_beamformed'):
            self.topics.append('/micrus/beamformed')
        cam_topics = {}
        if rec.get('include_wrist_camera') and cam.get('enabled'):
            cam_topics = {k: v for k, v in cam['topics'].items() if v}
            self.topics += list(cam_topics.values())
        alias = {'wrist.color': cam['topics'].get('color'), 'wrist.depth': cam['topics'].get('depth')}
        self.ready_topics = []
        for t in rec['ready_topics']:
            t = alias.get(t, t)
            if t and t in self.topics:
                self.ready_topics.append(t)
        self.ready_timeout = float(rec.get('ready_timeout_s', 20.0))

        self.lock_dir = Path(lock_dir)
        self.lock_dir.mkdir(parents=True, exist_ok=True)
        self.lock_file = self.lock_dir / 'micrus_recording.lock'
        self.proc = None
        self.recording_micrus = False
        self.hdf5_path = ''
        self.micrus_stop_error = None
        self._seen = {}
        self._seen_lock = threading.Lock()
        self._cfg_files = self._write_bag_configs(rec)

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self._subs = []
        from rosidl_runtime_py.utilities import get_message
        self._get_message = get_message
        self._qos = qos
        self._typed = set()
        self.micrus_record = node.create_client(SetBool, '/micrus/record')
        self._start_guard()

    # ------------------------------------------------------------------ setup
    def _write_bag_configs(self, rec):
        d = Path(tempfile.mkdtemp(prefix='usrig_bag_'))
        storage = d / 'mcap_writer.yaml'
        storage.write_text(yaml.safe_dump(rec.get('mcap_storage_config') or {}))
        qos = d / 'qos_overrides.yaml'
        # micrus publishes reliable / keep last / depth 2; record it reliably.
        overrides = {t: {'reliability': 'reliable', 'history': 'keep_last', 'depth': 100}
                     for t in self.topics if t.startswith('/micrus/')}
        qos.write_text(yaml.safe_dump(overrides))
        return {'storage': storage, 'qos': qos}

    def _start_guard(self):
        subprocess.Popen([sys.executable, '-m', 'usrig_episode.record_guard',
                          '--parent-pid', str(os.getpid()), '--lock', str(self.lock_file)],
                         start_new_session=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _watch(self, topic):
        """Subscribe (best effort) to see fresh messages on a ready topic."""
        if topic in self._typed:
            return True
        types = dict(self.node.get_topic_names_and_types())
        if topic not in types:
            return False
        msg_type = self._get_message(types[topic][0])

        def cb(_msg, t=topic):
            with self._seen_lock:
                self._seen[t] = now_ns()
        # raw: never deserialize (images at 27 Hz, robot at 500 Hz); removed again once the bag is ready
        self._subs.append(self.node.create_subscription(msg_type, topic, cb, self._qos, raw=True))
        self._typed.add(topic)
        return True

    # ------------------------------------------------------------------ micrus
    def _call_micrus(self, on: bool, timeout=10.0):
        if not self.micrus_record.wait_for_service(timeout_sec=timeout):
            raise RecorderError('/micrus/record service not available')
        fut = self.micrus_record.call_async(SetBool.Request(data=on))
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise RecorderError(f'/micrus/record {on} timed out')
            time.sleep(0.01)
        r = fut.result()
        if not r.success:
            raise RecorderError(f'/micrus/record {on} failed: {r.message}')
        return r.message

    def ensure_micrus_stopped(self):
        """Called at manager start-up: stop any recording a previous crash left on."""
        try:
            self._call_micrus(False, timeout=5.0)
        except RecorderError:
            pass
        self.lock_file.unlink(missing_ok=True)

    # ------------------------------------------------------------------ episode
    def start(self, bag_dir: Path) -> int:
        bag_dir = Path(bag_dir)
        if bag_dir.exists():
            raise RecorderError(f'{bag_dir} exists (ros2 bag record -o refuses an existing folder)')
        bag_dir.parent.mkdir(parents=True, exist_ok=True)
        for t in self.ready_topics:
            if not self._watch(t):
                raise RecorderError(f'ready topic {t} is not being published')
        cmd = self._rt_prefix() + ['ros2', 'bag', 'record', '-s', self.cfg['recording'].get('storage', 'mcap'),
               '--storage-config-file', str(self._cfg_files['storage']),
               '--qos-profile-overrides-path', str(self._cfg_files['qos']),
               '-o', str(bag_dir), *self.topics]
        self.bag_log = open(bag_dir.parent / f'.{bag_dir.name}.record.log', 'w')
        self.proc = subprocess.Popen(cmd, preexec_fn=_pdeathsig, start_new_session=True,
                                     stdout=self.bag_log, stderr=subprocess.STDOUT)
        t_ready = self._wait_ready()
        self.lock_file.write_text(str(os.getpid()))
        try:
            self.hdf5_path = self._call_micrus(True)
            self.recording_micrus = True
        except RecorderError:
            self._stop_bag()
            # a timed-out start may still have started micrus: stop it; keep the lock if that fails too
            try:
                self._call_micrus(False, timeout=5.0)
                self.lock_file.unlink(missing_ok=True)
            except RecorderError:
                pass
            raise
        return t_ready

    def _rt_prefix(self):
        """`chrt -f N` for the recorder when recording.rt_priority is set and allowed on this PC."""
        prio = int(self.cfg['recording'].get('rt_priority') or 0)
        if not prio:
            return []
        ok = subprocess.run(['chrt', '-f', str(prio), 'true'], capture_output=True).returncode == 0
        if not ok:
            self.node.get_logger().warn(
                f'recorder: SCHED_FIFO {prio} not permitted (rtprio limit); recording at normal priority. '
                'See docs/capture_pc_setup.md')
            return []
        return ['chrt', '-f', str(prio)]

    def _unwatch(self):
        """Drop the readiness subscriptions: they must not load this process during the episode."""
        for sub in self._subs:
            self.node.destroy_subscription(sub)
        self._subs.clear()
        self._typed.clear()
        with self._seen_lock:
            self._seen.clear()

    def _bag_subscribed(self, topic):
        for info in self.node.get_subscriptions_info_by_topic(topic):
            if info.node_name in BAG_NODE_NAMES:
                return True
        return False

    def _wait_ready(self):
        t0 = time.monotonic()
        while True:
            if self.proc.poll() is not None:
                raise RecorderError(f'ros2 bag record exited with {self.proc.returncode} (see {self.bag_log.name})')
            missing = [t for t in self.topics if t in self.ready_topics and not self._bag_subscribed(t)]
            if not missing:
                break
            if time.monotonic() - t0 > self.ready_timeout:
                self._stop_bag()
                self._unwatch()
                raise RecorderError(f'bag did not subscribe to {missing} within {self.ready_timeout} s')
            time.sleep(0.05)
        t_sub = now_ns()
        while True:
            with self._seen_lock:
                stale = [t for t in self.ready_topics if self._seen.get(t, 0) <= t_sub]
            if not stale:
                self._unwatch()
                return now_ns()
            if time.monotonic() - t0 > self.ready_timeout:
                self._stop_bag()
                self._unwatch()
                raise RecorderError(f'no fresh message on {stale} after the bag subscribed')
            time.sleep(0.01)

    def _stop_bag(self, timeout=60.0):
        if self.proc is None:
            return None
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGTERM)
                self.proc.wait(timeout=10)
        rc = self.proc.returncode
        self.proc = None
        self.bag_log.close()
        return rc

    def stop(self):
        """End the bag cleanly, then stop micrus. Returns (bag_returncode, hdf5_path). A failed micrus stop is
        recorded in micrus_stop_error, not raised: the episode is complete and must still be written. The lock
        file then stays, so record_guard stops micrus when this process ends."""
        rc = self._stop_bag()
        self.micrus_stop_error = None
        if self.recording_micrus:
            self.recording_micrus = False
            try:
                self._call_micrus(False)
                self.lock_file.unlink(missing_ok=True)
            except RecorderError as e:
                self.micrus_stop_error = str(e)
        return rc, self.hdf5_path

    def abort(self):
        """Best effort, for signal / atexit paths: never leave micrus recording."""
        try:
            self._stop_bag(timeout=20.0)
        except Exception:
            pass
        if self.recording_micrus or self.lock_file.exists():
            try:
                self._call_micrus(False, timeout=5.0)
            except Exception:
                from .record_guard import stop_micrus_recording
                try:
                    stop_micrus_recording()
                except Exception:
                    pass
            self.recording_micrus = False
            self.lock_file.unlink(missing_ok=True)
