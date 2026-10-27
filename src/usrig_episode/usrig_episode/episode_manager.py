"""episode_manager: runs the stage, starts and stops recordings, writes labels and events.

  ros2 run usrig_episode episode_manager --ros-args -p session_id:=<id> [-p stage:=0|1|2]
      [-p robot_control:=true] [-p adhoc_json:=fields.json]

Stage 0  operator starts / stops (B key or dashboard); safety cap stage0.max_duration_s.
Stage 1  per path: approach -> recording -> free-air hold -> zero_ft -> descend -> start ->
         run_path -> end -> retract -> QC. Esc aborts, P holds.
Stage 2  per plan row (Mode B): move_to_pose(approach + start_offset) -> recording -> free-air
         hold -> zero_ft -> descend -> start -> teleop (+ planned perturbation) -> outcome key ->
         end/outcome -> retract -> QC (pass: next row, fail: redo). Restart resumes at the first
         incomplete row.
Mode A   any stage: extra fields from adhoc_json are merged into episode.json.
Freedrive  -p collection_mode:=freedrive (robot_control off): the operator guides the arm by hand with
         the foot pedal (UR "Freedrive" input). B start -> recording -> free-air hold -> start ->
         demonstration -> outcome keys -> B stop. Nothing is commanded; episodes are type
         'kinesthetic', stage 0, with the grip from session.yaml freedrive.grip.

A stage whose gate has not passed is refused; the gate result goes to session.json.
"""
import atexit
import json
import queue
import signal
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from usrig_common import config, geometry
from usrig_common.session import (check_sim_flag, data_root, read_json, session_dir, update_session_json,
                                  write_json_atomic)
from usrig_common.timeutil import now_ns, ns_from_stamp, stamp_from_ns
from usrig_interfaces.msg import EpisodeEvent
from usrig_interfaces.srv import DescendToContact, InjectPerturbation, MoveToPose, RunPath

from usrig_common import gates
from usrig_common.rosutil import ignore_sigint

from . import episode_status as es
from . import provenance
from .recorder import EpisodeRecorder, RecorderError
from usrig_presets.path_editor import resolve as resolve_path
from usrig_presets.plan_generator import read_csv, sha256
from usrig_presets.waypoints import executable_problem


class Abort(Exception):
    pass


class EpisodeManager(Node):

    def __init__(self):
        super().__init__('episode_manager')
        for name, default in (('config_dir', ''), ('session_id', ''), ('session_json', ''),
                              ('stage', -1), ('robot_control', False), ('adhoc_json', ''),
                              ('operator_id', ''), ('phantom_id', ''), ('sim', False),
                              ('collection_mode', 'robot'), ('camera', True), ('tpv', True)):
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        cfg_dir = p('config_dir') or None
        self.scfg = config.load_config('session', cfg_dir)
        if not bool(self.get_parameter('camera').value):
            # camera:=false: not started, so not recorded, not waited for, not checked by QC
            self.scfg['recording']['include_wrist_camera'] = False
        self.camera_recorded = bool(self.scfg['recording'].get('include_wrist_camera')
                                    and self.scfg['wrist_camera'].get('enabled'))
        if not bool(self.get_parameter('tpv').value):
            self.scfg['recording']['include_tpv_camera'] = False     # tpv:=false: as camera:=false, for the TPV
        self.tpv_recorded = config.camera_recorded(self.scfg, 'tpv_camera')
        self.rig = config.load_config('rig', cfg_dir)
        self.session_id = p('session_id')
        if not self.session_id:
            raise RuntimeError('session_id parameter is required')
        self.stage = int(p('stage')) if int(p('stage')) >= 0 else int(self.scfg['session']['stage'])
        self.control = bool(p('robot_control'))
        self.collection_mode = p('collection_mode')
        if self.collection_mode not in ('robot', 'freedrive'):
            raise RuntimeError("collection_mode must be 'robot' or 'freedrive'")
        if self.collection_mode == 'freedrive':
            if self.control:
                raise RuntimeError('collection_mode freedrive: the robot is guided by hand; run with robot_control:=false')
            if config.get_path(self.rig, 'freedrive.input') is None:
                raise RuntimeError('collection_mode freedrive: rig.yaml freedrive.input (pedal input) is null')
            self.stage = 0                 # operator-paced, no stage gate; episodes are type 'kinesthetic'
        self.adhoc = read_json(p('adhoc_json')) if p('adhoc_json') else {}
        self.operator_id = p('operator_id') or self.scfg['session']['operators'][0]
        self.phantom_id = p('phantom_id') or self.scfg['session'].get('phantom_id')
        phantoms = self.scfg.get('phantoms') or {}
        if self.phantom_id not in phantoms:
            raise RuntimeError(f'phantom_id {self.phantom_id!r} is not one of the proposal phantoms '
                               f'{sorted(phantoms)} (session.yaml phantoms)')
        self.keymap = config.load_config('keymap', cfg_dir)
        self.sim = bool(p('sim'))
        self.root = data_root(self.scfg, self.sim)
        self.sdir = session_dir(self.root, self.session_id)
        check_sim_flag(self.sdir, self.sim)
        self.sdir.mkdir(parents=True, exist_ok=True)

        cb = ReentrantCallbackGroup()
        self.pub_evt = self.create_publisher(EpisodeEvent, '/episode/events', 50)
        self.pub_cmd = self.create_publisher(String, '/operator/command', 10)
        self.create_subscription(String, '/operator/command', self._on_cmd, 50, callback_group=cb)
        self.create_subscription(String, '/micrus/frame_info', self._on_frame_info, 10, callback_group=cb)
        self.create_subscription(EpisodeEvent, '/episode/events', self._on_event, 50, callback_group=cb)
        self.cli = {
            'zero_ft': self.create_client(Trigger, '/robot/zero_ft', callback_group=cb),
            'move': self.create_client(MoveToPose, '/robot/move_to_pose', callback_group=cb),
            'descend': self.create_client(DescendToContact, '/robot/descend_to_contact', callback_group=cb),
            'teleop': self.create_client(SetBool, '/robot/set_teleop', callback_group=cb),
            'perturb': self.create_client(InjectPerturbation, '/robot/inject_perturbation', callback_group=cb),
            'retract': self.create_client(Trigger, '/robot/retract', callback_group=cb),
            'run_path': self.create_client(RunPath, '/robot/run_path', callback_group=cb),
        }

        self.cmds = queue.Queue()
        self.ep = None
        self.ep_lock = threading.Lock()
        # operator console state (/episode/status): what we wait for, pause spans, last episode, counts
        self.pause_log = None
        self.waiting_for = set()
        self.task = None                 # operator task (target + instruction) for freedrive / teleop episodes
        self.finishing = False
        self.last_ep = None
        self.plan_info = None
        self.counts = es.summarize_episodes(self._finished_episodes())
        self.pub_status = self.create_publisher(
            String, '/episode/status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.recorder = EpisodeRecorder(self, self.scfg, self.sdir)
        self.recorder.ensure_micrus_stopped()
        self._recover_interrupted()

        update_session_json(self.sdir, session_id=self.session_id, stage=self.stage, sim=self.sim,
                            operators=self.scfg['session']['operators'], phantom_id=self.phantom_id,
                            phantom_placement=self.scfg['session'].get('phantom_placement'),
                            rig_git_commit=provenance.rig_git_commit())
        self._snapshot_calibration()
        g = gates.check_stage(self.stage, self.scfg, self.root)
        update_session_json(self.sdir, gates={f'stage{self.stage}': {**g, 'checked_ns': now_ns()}})
        if not g['passed']:
            raise RuntimeError(f'stage {self.stage} gate has not passed: {json.dumps(g)}')
        if self.control and self.stage > 0:
            config.require_limits(self.rig)
        # Pre-session gate at Start, not here: the check needs this launch's streams running, so refusing at
        # start-up meant launch -> exit -> check -> relaunch for every new session.
        self.presession_problem = None
        if not self._presession_ready():
            self.get_logger().warn(f'no episode starts until the pre-session check passes: {self.presession_problem}')

        self.create_timer(0.5, self._publish_status, callback_group=cb)
        self.worker = threading.Thread(target=self._run_stage, name='stage', daemon=True)
        self.worker.start()

    def _snapshot_calibration(self):
        """rig.yaml's hand-eye matrix into session.json, unless the session already has its own (handeye_calib
        --session writes it there): overwriting it on every start lost that session's calibration."""
        sess = read_json(self.sdir / 'session.json')
        if sess.get('wrist_cam_cali_mtx') is None:
            update_session_json(self.sdir, wrist_cam_cali_mtx=config.get_path(self.rig, 'calibration.wrist_cam_cali_mtx'))
        if sess.get('tpv_cam_cali_mtx') is None:
            update_session_json(self.sdir, tpv_cam_cali_mtx=config.get_path(self.rig, 'calibration.tpv_cam_cali_mtx'),
                                tpv_cam_cali=config.get_path(self.rig, 'calibration.tpv_cam_cali_info'))

    def _presession_ready(self):
        """A passing, recent pre-session check for this stage in session.json (re-read every time: the operator
        runs the check while this node is up). Sets presession_problem for the console."""
        sess = read_json(self.sdir / 'session.json')
        ok, why = (True, None) if self.sim else gates.presession_ok(
            sess, self.stage, self.control, now_ns(), self.scfg['session'].get('presession_max_age_h', 12),
            self.camera_recorded, self.tpv_recorded)
        if ok:
            from shutil import disk_usage
            free = disk_usage(self.sdir).free / 1e9
            floor = float(self.scfg['session'].get('min_free_disk_gb', 50))
            if free <= floor:
                ok, why = False, f'{free:.0f} GB free on the data disk (floor {floor:g} GB): free space first'
        T = (sess.get('phantom_registration') or {}).get('T_base_phantom')
        if ok and T is not None:
            # the recorded tcp_phantom comes from robot_state's own copy of the registration
            from usrig_common import registration_sync
            same, msg = registration_sync.matches(self, T)
            if same is False:
                ok, why = False, f'{msg}: run register_phantom again (it updates robot_state) or restart the launch'
        self.presession_problem = None if ok else why
        return ok

    def _wait_presession(self):
        """Block until the pre-session check passes; the operator runs it, then presses Start."""
        while rclpy.ok() and not self._presession_ready():
            self.get_logger().error(f'refusing to collect: {self.presession_problem}. Run the pre-session check, '
                                    'then press Start')
            self.wait_cmd({'start_stop'})

    # ------------------------------------------------------------------ operator console status
    def _finished_episodes(self):
        out = []
        for f in sorted(self.sdir.glob('ep_*/episode.json')):
            try:
                out.append(read_json(f))
            except (OSError, ValueError):
                continue
        return out

    def _publish_status(self):
        with self.ep_lock:
            payload = es.status_payload(
                session={'session_id': self.session_id, 'stage': self.stage, 'collection_mode': self.collection_mode,
                         'robot_control': self.control, 'operator_id': self.operator_id,
                         'phantom_id': self.phantom_id, 'sim': self.sim, 'session_dir': str(self.sdir),
                         'presession_problem': self.presession_problem,
                         'stage_error': getattr(self, 'stage_error', None),
                         'task': getattr(self, 'task', None)},
                ep=self.ep, pause_log=self.pause_log, waiting_for=self.waiting_for, finishing=self.finishing,
                last_ep=self.last_ep, counts=self.counts, plan=self.plan_info, now_ns=now_ns())
        self.pub_status.publish(String(data=json.dumps(payload, default=str)))

    def _set_paused(self, ep, pause, t):
        """Operator pause / resume (ep_lock held). Recording continues; the span is marked and the robot holds."""
        if self.pause_log is None or ep.get('t_start_ns') is None:
            return
        if pause and getattr(self, 'stage', None) == 1 and getattr(self, 'collection_mode', 'robot') == 'robot':
            self.get_logger().warn('pause is not used in Stage 1 (the path runs to a fixed time budget): use discard')
            return
        changed = self.pause_log.pause(t) if pause else self.pause_log.resume(t)
        if not changed:
            return
        self.event('pause', {'state': 'paused' if pause else 'resumed', 't_ns': t}, t)
        if self.control:
            self.pub_cmd.publish(String(data=json.dumps({'cmd': 'hold', 'value': bool(pause)})))

    # ------------------------------------------------------------------ inputs
    def _on_cmd(self, msg):
        try:
            c = json.loads(msg.data)
        except ValueError:
            return
        cmd = c.get('cmd')
        if cmd == 'key':
            return
        if cmd == 'set_task':
            # the operator's current task for the next freedrive / teleop episodes (Open-H: task type, target,
            # instruction and its paraphrases per episode); an incomplete one is ignored
            task = es.operator_task(c)
            if task:
                self.task = task
            return
        with self.ep_lock:
            ep = self.ep
            if ep is not None:
                t = now_ns()
                if cmd == 'view_reached':
                    ep['view_reached_ns'].append(t)
                    self.event('view_reached', {'t_ns': t})
                elif cmd == 'note':
                    ep['notes'] = (ep['notes'] + '\n' + c.get('value', '')).strip()
                    self.event('note', {'text': c.get('value', '')})
                elif cmd == 'gel':
                    ep['gel_state'] = c.get('value')
                    self.event('gel', {'state': c.get('value')})
                elif cmd in ('pause', 'resume', 'pause_toggle'):
                    want = (not (self.pause_log and self.pause_log.paused)) if cmd == 'pause_toggle' else cmd == 'pause'
                    self._set_paused(ep, want, t)
                elif cmd in ('onset', 'recovery_start', 'recovery_end'):
                    key = {'onset': 'onset_t', 'recovery_start': 'recovery_start_t',
                           'recovery_end': 'recovery_end_t'}[cmd]
                    rel = (t - ep['t_start_ns']) * 1e-9 if ep.get('t_start_ns') else None
                    if cmd == 'onset' and ep.get('onset_source') == 'disturbance stamp':
                        # the injected disturbance's own stamp stays the onset; the operator's press is kept beside it
                        ep.setdefault('onset_t_operator', rel)
                    elif ep.get(key) is None:
                        # first press counts (the annotation's g_t intervals use the first event too)
                        ep[key] = rel
                        if cmd == 'onset':
                            ep['onset_source'] = 'operator key'
                    self.event('phase', {'phase': cmd, 't_ns': t})
        self.cmds.put(c)

    def _on_frame_info(self, msg):
        with self.ep_lock:
            ep = self.ep
            if ep is None or ep.get('t_start_ns') is None:
                return
            try:
                settings = json.loads(msg.data).get('settings')
            except ValueError:
                return
            if ep['micrus_settings'] is None:
                ep['micrus_settings'] = settings
            elif settings != ep['micrus_settings']:
                ep['micrus_settings_constant'] = False

    def _on_event(self, msg):
        if msg.kind == 'policy':
            self._on_policy_event(msg)
            return
        if msg.kind != 'perturbation':
            return
        with self.ep_lock:
            ep = self.ep
            if ep is None:
                return
            v = json.loads(msg.value)
            ep['disturbance'] = {'kind': v.get('kind'), 'delta': v.get('delta'), 'onset_ns': v.get('onset_ns')}
            if ep.get('t_start_ns') and v.get('onset_ns'):
                ep['onset_t'] = (v['onset_ns'] - ep['t_start_ns']) * 1e-9
                ep['onset_source'] = 'disturbance stamp'

    def _on_policy_event(self, msg):
        """robot_control's policy runs: which policy drove this episode (model, checkpoint) and when, for the
        export's provenance. The per-step labels (source POLICY, not expert) are in /robot/state already."""
        with self.ep_lock:
            ep = self.ep
            if ep is None:
                return
            try:
                v = json.loads(msg.value)
            except ValueError:
                return
            t_ns = ns_from_stamp(msg.header.stamp)
            if v.get('enabled'):
                info = {k: x for k, x in v.items() if k != 'enabled'}
                if ep.get('policy') is None:
                    ep['policy'] = info
                elif ep['policy'] != info:
                    ep['policy_changed'] = True       # two policies in one episode: flagged for review
                ep.setdefault('policy_runs', []).append({'start_ns': t_ns, 'end_ns': None, 'reason': None})
            elif ep.get('policy_runs') and ep['policy_runs'][-1]['end_ns'] is None:
                ep['policy_runs'][-1].update(end_ns=t_ns, reason=v.get('reason'))

    # ------------------------------------------------------------------ helpers
    def event(self, kind, value, stamp_ns=None):
        e = EpisodeEvent()
        e.header.stamp = stamp_from_ns(stamp_ns or now_ns())
        e.episode_id = self.ep['episode_id'] if self.ep else ''
        e.kind = kind
        e.value = json.dumps(value)
        self.pub_evt.publish(e)
        return ns_from_stamp(e.header.stamp)

    def call(self, name, req, timeout=120.0):
        if not self.control:
            self.get_logger().info(f'robot_control off: skipping {name}')
            return None
        c = self.cli[name]
        if not c.wait_for_service(timeout_sec=5.0):
            raise Abort(f'service for {name} not available')
        fut = c.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if time.monotonic() - t0 > timeout:
                raise Abort(f'{name} timed out')
            time.sleep(0.01)
        r = fut.result()
        ok = getattr(r, 'success', True)
        if not ok:
            raise Abort(f'{name} failed: {getattr(r, "message", "")}')
        return r

    def wait_cmd(self, wanted, timeout=None):
        """Block until one of the wanted commands arrives; returns it (or None on timeout).
        ``timeout``: seconds, or a function returning the seconds left (re-evaluated while waiting)."""
        t_end = None if timeout is None or callable(timeout) else time.monotonic() + timeout
        self.waiting_for = set(wanted)
        try:
            while rclpy.ok():
                if callable(timeout):
                    left = timeout()
                else:
                    left = None if t_end is None else t_end - time.monotonic()
                if left is not None and left <= 0:
                    return None
                try:
                    c = self.cmds.get(timeout=0.1 if left is None else min(0.1, left))
                except queue.Empty:
                    continue
                if c.get('cmd') in wanted:
                    return c
            raise Abort('shutdown')
        finally:
            self.waiting_for = set()

    def _cap_left(self, cap_s, t_ref_ns):
        """Seconds left of an episode time cap counted from t_ref_ns, not counting paused time: a pause must
        not end the episode (Stage 2 cap_s equals the longest planned scan)."""
        now = now_ns()
        with self.ep_lock:
            paused = self.pause_log.total_s(now) if self.pause_log else 0.0
        return cap_s - ((now - t_ref_ns) * 1e-9 - paused)

    def _drain(self):
        while not self.cmds.empty():
            self.cmds.get_nowait()

    def _next_index(self):
        existing = [int(p.name[3:]) for p in self.sdir.glob('ep_*') if p.name[3:].isdigit()]
        return max(existing, default=0) + 1

    def _recover_interrupted(self):
        """Episodes whose manager died: keep the bag, write episode.json as an interrupted discard."""
        for partial in sorted(self.sdir.glob('.ep_*.partial.json')):
            ep = read_json(partial)
            d = self.sdir / ep['episode_dir']
            if d.exists() and not (d / 'episode.json').exists():
                ep.update(outcome='discard', success=False, interrupted=True,
                          notes=(ep.get('notes', '') + '\ninterrupted: manager stopped mid-episode').strip())
                write_json_atomic(d / 'episode.json', ep)
                self.get_logger().warn(f'{d.name}: interrupted episode marked discard')
            partial.unlink(missing_ok=True)
        for f in sorted(self.sdir.glob('ep_*/episode.json')):
            ep = read_json(f)
            if ep.get('qc') is None and ep.get('outcome') != 'discard':
                self.get_logger().info(f'{f.parent.name}: QC never ran (stopped during post-processing): running it')
                self._post_process(ep)

    def _wrist_info(self):
        cam = self.scfg['wrist_camera']
        return {'serial': cam.get('serial'), 'firmware': provenance.camera_firmware(cam.get('serial')),
                **provenance.realsense_versions(),
                'color_profile': f"{cam['color_width']}x{cam['color_height']}@{cam['fps']}",
                'depth_profile': f"{cam['depth_width']}x{cam['depth_height']}@{cam['fps']}",
                'exposure_us': cam.get('exposure_us'), 'white_balance_k': cam.get('white_balance_k'),
                'cali_ref': 'session.json:wrist_cam_cali_mtx'}

    def _tpv_info(self):
        cam = self.scfg['tpv_camera']
        return {'serial': cam.get('serial'), 'firmware': provenance.camera_firmware(cam.get('serial')),
                **provenance.realsense_versions(),
                'color_profile': f"{cam['color_width']}x{cam['color_height']}@{cam['fps']}",
                'exposure_us': cam.get('exposure_us'), 'white_balance_k': cam.get('white_balance_k'),
                'cali_ref': 'session.json:tpv_cam_cali_mtx'}

    def _task_fields(self, mode):
        """Episode fields from the operator's task. Freedrive episodes need one (None = refuse Start); Stage 0
        teleop with a task is collection ('teleop', exported), without one a shakedown test (not exported)."""
        task = getattr(self, 'task', None)
        if mode == 'freedrive':
            return None if not task else {'type': 'kinesthetic', **task}
        return {'type': 'teleop', **task} if task else {'type': 'shakedown'}

    def _new_episode(self, **fields):
        self.finishing = False
        idx = self._next_index()
        ep_id = f'{self.session_id}_ep_{idx:04d}'
        ep = {
            'episode_id': ep_id, 'episode_dir': f'ep_{idx:04d}', 'session_id': self.session_id, 'sim': self.sim,
            'stage': self.stage, 'plan_row': None, 'seed': None, 'collection_mode': self.collection_mode,
            'mode': 'B' if self.stage == 2 and not self.adhoc else 'A',
            'operator_id': self.operator_id, 'phantom_id': self.phantom_id,
            'phantom_placement': self.scfg['session'].get('phantom_placement'),
            'target': None, 'type': None, 'instruction': None, 'start_offset': None,
            'task_type': None, 'sub_task': None, 'cmd_sets': None,      # Open-H instruction.* (export)
            'disturbance': None, 'policy': None, 'policy_runs': [], 'force_setpoint_n': config.get_path(self.rig, 'limits.force_setpoint_default_n'),
            'failure_mode': None, 'staging_method': None, 'onset_t': None, 'recovery_start_t': None,
            'recovered': None, 'recovery_end_t': None, 'onset_source': None,
            'outcome': None, 'success': None, 'view_reached_ns': [], 'gel_state': None, 'notes': '',
            't_created_ns': now_ns(), 'pauses_ns': [], 'pauses': [], 'paused_s': 0.0,
            'micrus_hdf5_path': None, 'micrus_settings': None, 'micrus_settings_constant': True,
            'rig_git_commit': provenance.rig_git_commit(),
            'micrus_version': provenance.micrus_version(self.scfg['micrus']['command']),
            'robot_tcp_setting': config.get_path(self.rig, 'robot.tcp_setting'),
            # this PC's measured stream offsets (machine.yaml), carried with the data for the export
            'time_offsets_s': {'ultrasound': config.get_path(self.rig, 'calibration.ultrasound_time_offset_s'),
                               'camera': config.get_path(self.rig, 'calibration.camera_time_offset_s'),
                               'tpv_camera': config.get_path(self.rig, 'calibration.tpv_camera_time_offset_s')},
            'capture_host': __import__('socket').gethostname(),
            't_start_ns': None, 't_end_ns': None, 't_bag_ready_ns': None, 'duration_s': None, 'qc': None,
            'wrist_camera': self._wrist_info() if self.camera_recorded else None,
            'tpv_camera': self._tpv_info() if self.tpv_recorded else None,
        }
        ep.update(fields)
        ep.update(self.adhoc)
        return ep

    # ------------------------------------------------------------------ episode skeleton
    def _begin_recording(self, ep):
        with self.ep_lock:
            self.ep = ep
            self.pause_log = es.PauseLog()
        write_json_atomic(self.sdir / f'.{ep["episode_dir"]}.partial.json', ep)
        ep['t_bag_ready_ns'] = self.recorder.start(self.sdir / ep['episode_dir'])
        ep['micrus_hdf5_path'] = self.recorder.hdf5_path
        # refresh the crash record now that the HDF5 path is known (used if this process dies)
        write_json_atomic(self.sdir / f'.{ep["episode_dir"]}.partial.json', ep)
        self.event('phase', {'phase': 'free_air', 'hold_s': self.scfg['recording']['free_air_hold_s']})
        time.sleep(float(self.scfg['recording']['free_air_hold_s']))
        self.event('phase', {'phase': 'free_air_end'})

    def _close_unstarted(self, ep):
        """An episode whose bag never became ready (readiness gate, approach failure): stop whatever started and
        write its episode.json as a discard now, so no folder is left without one."""
        self.recorder.abort()
        with self.ep_lock:
            self.ep = None
            self.pause_log = None
        ep['outcome'] = 'discard'
        d = self.sdir / ep['episode_dir']
        if d.exists():
            write_json_atomic(d / 'episode.json', ep)
        (self.sdir / f'.{ep["episode_dir"]}.partial.json').unlink(missing_ok=True)

    def _start_event(self, ep):
        ep['t_start_ns'] = self.event('start', {k: ep.get(k) for k in
                                                ('stage', 'type', 'task_type', 'target', 'instruction', 'plan_row')})

    def _finish(self, ep, task_text=None, post=True):
        """end event -> bag stops -> micrus stops -> task.txt / episode.json [-> QC, auto-annotation].
        post=False: the caller retracts first and then calls _post_process (QC decodes every frame: seconds of
        dead time that used to pass with the probe still in contact)."""
        if ep.get('t_start_ns') is None:
            ep['t_start_ns'] = ep.get('t_bag_ready_ns')
        self.finishing = True
        with self.ep_lock:
            self._set_paused(ep, False, now_ns())         # a pause open at the end is closed (robot hold released)
            spans = self.pause_log.closed(now_ns()) if self.pause_log else []
            self.pause_log = None
        ep.update(es.episode_record(spans, ep['t_start_ns']))
        ep['t_end_ns'] = self.event('end', {'outcome': ep.get('outcome')})
        if ep.get('outcome'):
            self.event('outcome', {'outcome': ep['outcome']})
        time.sleep(0.2)   # let the end / outcome events reach the bag
        rc, _ = self.recorder.stop()
        if self.recorder.micrus_stop_error:
            ep['micrus_stop_error'] = self.recorder.micrus_stop_error
            ep['notes'] = (ep['notes'] + f'\nmicrus stop failed: {self.recorder.micrus_stop_error}').strip()
            self.get_logger().error(f'micrus did not confirm the stop ({self.recorder.micrus_stop_error}); '
                                    'episode kept, record_guard will stop micrus')
        with self.ep_lock:
            self.ep = None
        ep['duration_s'] = (ep['t_end_ns'] - ep['t_start_ns']) * 1e-9 if ep['t_start_ns'] else None
        ep['success'] = ep.get('outcome') in ('success', 'recovered')
        if ep.get('outcome') == 'recovered':
            ep['recovered'] = True
        ep['bag_returncode'] = rc
        d = self.sdir / ep['episode_dir']
        if task_text or ep.get('instruction'):
            (d / 'task.txt').write_text((task_text or ep['instruction']) + '\n')
        write_json_atomic(d / 'episode.json', ep)
        (self.sdir / f'.{ep["episode_dir"]}.partial.json').unlink(missing_ok=True)
        return self._post_process(ep) if post else ep

    def _post_process(self, ep):
        """QC and auto-annotation of a written episode, then the final episode.json. Discards are not checked
        (never exported): their QC time was pure operator dead time."""
        d = self.sdir / ep['episode_dir']
        self.finishing = True
        if ep.get('outcome') == 'discard':
            ep['qc'] = {'pass': False, 'skipped': 'discarded episode'}
            write_json_atomic(d / 'episode.json', ep)
            self.last_ep = es.last_summary(ep)
            self.counts = es.summarize_episodes(self._finished_episodes())
            self.finishing = False
            return ep
        try:
            from usrig_qc.episode_qc import run_episode_qc
            ep['qc'] = run_episode_qc(d, ep, self.scfg, self.rig)
        except Exception as e:  # noqa: BLE001
            ep['qc'] = {'pass': False, 'error': f'QC failed to run: {e}'}
        try:
            from usrig_qc.auto_annotate import auto_annotate
            auto_annotate(d, ep, self.keymap)
            ep['annotation_file'] = 'annotation.json'
        except Exception as e:  # noqa: BLE001
            ep['annotation_file'] = None
            ep['notes'] = (ep['notes'] + f'\nauto-annotation failed: {e}').strip()
        write_json_atomic(d / 'episode.json', ep)
        self.get_logger().info(f'{ep["episode_id"]}: outcome={ep.get("outcome")} '
                               f'duration={ep["duration_s"]} qc_pass={ep["qc"].get("pass")}')
        self.last_ep = es.last_summary(ep)
        self.counts = es.summarize_episodes(self._finished_episodes())
        self.finishing = False
        return ep

    def _safe_retract(self):
        try:
            self.call('teleop', SetBool.Request(data=False))
            self.call('retract', Trigger.Request())
        except Abort as e:
            self.get_logger().error(f'retract: {e}')

    def _zero_and_descend(self, fsp=None):
        self.call('zero_ft', Trigger.Request())
        req = DescendToContact.Request()
        req.force_setpoint_n = float(fsp or 0.0)
        self.call('descend', req)

    def _move(self, T, frame='phantom', cone_T=None):
        """Move to T. Phantom frame: then set the local tilt cone (D10) around the waypoint's probe axis,
        ``cone_T`` (the taught waypoint), not around a deliberately offset start pose or the first pose of
        a fan; without ``cone_T`` the target T itself."""
        req = MoveToPose.Request()
        req.target = PoseStamped()
        req.target.header.frame_id = frame
        geometry.fill_pose_msg(req.target.pose, T)
        self.call('move', req)
        if frame == 'phantom':
            self._set_local_tilt(T if cone_T is None else cone_T)

    @staticmethod
    def _taught_force(wp):
        """Descent setpoint for a waypoint: the probe-axis force taught with it (force_n), as a magnitude
        (the wrench sign convention is not verified yet); robot_control clamps it to the force band.
        None (not taught, or no waypoint) -> robot_control's default setpoint."""
        f = (wp or {}).get('force_n')
        return abs(float(f)) if f else None

    def _set_local_tilt(self, T_phantom):
        """D10: tilt cone around this waypoint's probe axis, sent to robot_control (base frame)."""
        reg = read_json(self.sdir / 'session.json').get('phantom_registration', {})
        if 'T_base_phantom' not in reg:
            return
        axis = (np.array(reg['T_base_phantom'])[:3, :3] @ np.asarray(T_phantom)[:3, 2]).tolist()
        self.pub_cmd.publish(String(data=json.dumps({'cmd': 'set_local_tilt_axis', 'axis_base': axis})))

    # ------------------------------------------------------------------ stages
    def _run_stage(self):
        self.stage_error = None
        try:
            self._wait_presession()
            if self.collection_mode == 'freedrive':
                self._freedrive()
                return
            {0: self._stage0, 1: self._stage1, 2: self._stage2}[self.stage]()
        except Abort as e:
            self.get_logger().error(f'stage stopped: {e}')
        except Exception as e:  # noqa: BLE001
            # shown in /episode/status: the node keeps running, so the console would otherwise show "idle"
            self.stage_error = repr(e)
            self.get_logger().error(f'stage error: {e!r} (no more episodes: restart the collection)')
        finally:
            self.recorder.abort()

    def _stage0(self):
        cap = float(self.scfg['stage0']['max_duration_s'])
        while rclpy.ok():
            self.get_logger().info('Stage 0: press B (start_stop) to start an episode')
            self._drain()
            self.wait_cmd({'start_stop'})
            if not self._presession_ready():
                self.get_logger().error(f'episode not started: {self.presession_problem}')
                continue
            ep = self._new_episode(**self._task_fields('robot'))
            try:
                wp_name = self.scfg['stage0'].get('approach_waypoint')
                wp = self._waypoint(wp_name) if wp_name and self.control else None
                if wp is not None:
                    self._move(self._approach_pose(wp['T']), cone_T=wp['T'])
                self._begin_recording(ep)
                if self.control:
                    self._zero_and_descend(self._taught_force(wp))
                    self.call('teleop', SetBool.Request(data=True))
                self._start_event(ep)
                left = lambda: self._cap_left(cap, ep['t_start_ns'])  # noqa: E731
                c = self.wait_cmd({'start_stop', 'discard', 'outcome'}, timeout=left)
                while c is not None and c['cmd'] == 'outcome':
                    ep['outcome'] = c['value']
                    c = self.wait_cmd({'start_stop', 'discard', 'outcome'}, timeout=left)
                if c is None:
                    ep['notes'] = (ep['notes'] + '\nstopped by stage0.max_duration_s').strip()
                elif c['cmd'] == 'discard':
                    ep['outcome'] = 'discard'
            except Exception as e:  # noqa: BLE001  (Abort, RecorderError, OSError, ...: this episode only)
                self.get_logger().error(f"{ep['episode_id']} aborted, discarded: {e}")
                ep['outcome'] = 'discard'
                ep['notes'] = (ep['notes'] + f'\naborted: {e}').strip()
            finally:
                written = bool(ep.get('t_bag_ready_ns'))
                if written:
                    ep = self._finish(ep, post=False)
                else:
                    self._close_unstarted(ep)
                if self.control:
                    self._safe_retract()
                if written:
                    self._post_process(ep)

    def _freedrive(self):
        """Kinesthetic episodes: the operator moves the arm with the freedrive pedal; nothing is commanded."""
        cap = float(self.scfg['stage0']['max_duration_s'])
        grip = config.get_path(self.scfg, 'freedrive.grip')
        while rclpy.ok():
            self.get_logger().info('Freedrive: probe in free air, press B to start; hold the pedal to guide '
                                   'the arm; B again to stop')
            self._drain()
            self.wait_cmd({'start_stop'})
            if not self._presession_ready():
                self.get_logger().error(f'episode not started: {self.presession_problem}')
                continue
            if self.count_publishers('/robot/control_status') > 0:
                self.get_logger().error('robot_control is running: stop it for freedrive collection (episode not started)')
                continue
            task = self._task_fields('freedrive')
            if task is None:
                self.get_logger().error('episode not started: set the task (type + target + instruction) in the console first')
                continue
            ep = self._new_episode(**task, collection_mode='freedrive', grip=grip,
                                   force_label_valid=grip == 'wrist_handle', force_setpoint_n=None)
            try:
                self._begin_recording(ep)          # includes the free-air hold (F/T offset window)
                self._start_event(ep)
                left = lambda: self._cap_left(cap, ep['t_start_ns'])  # noqa: E731
                c = self.wait_cmd({'start_stop', 'discard', 'outcome'}, timeout=left)
                while c is not None and c['cmd'] == 'outcome':
                    ep['outcome'] = c['value']
                    c = self.wait_cmd({'start_stop', 'discard', 'outcome'}, timeout=left)
                if c is None:
                    ep['notes'] = (ep['notes'] + '\nstopped by stage0.max_duration_s').strip()
                elif c['cmd'] == 'discard':
                    ep['outcome'] = 'discard'
            except Exception as e:  # noqa: BLE001  (Abort, RecorderError, OSError, ...: this episode only)
                self.get_logger().error(f"{ep['episode_id']} aborted, discarded: {e}")
                ep['outcome'] = 'discard'
                ep['notes'] = (ep['notes'] + f'\naborted: {e}').strip()
            finally:
                if ep.get('t_bag_ready_ns'):
                    self._finish(ep)
                else:
                    self._close_unstarted(ep)

    def _stage1(self):
        st1 = self.scfg['stage1']
        if st1.get('speed_mm_s') is None or not st1.get('path_file'):
            raise Abort('stage1.speed_mm_s / stage1.path_file null in session.yaml')
        paths = config.load_yaml(st1['path_file'])['paths']
        names = st1.get('paths') or list(paths)
        for name in names:
            spec = paths[name]
            named = [self._waypoint(e) for e in spec['waypoints'] if isinstance(e, str)]   # must be verified
            # fan / sweep paths are built around one taught waypoint (path_editor writes it as 'center'):
            # its axis centres the local tilt cone and its taught force is the descent setpoint
            center = self._waypoint(spec['center']) if spec.get('center') else (named[0] if named else None)
            wps = resolve_path(spec['waypoints'], config.load_yaml(self._waypoint_file())['waypoints'])
            ep = self._new_episode(type='scripted', target=name, instruction=spec.get('instruction'),
                                   path_waypoints=[w if isinstance(w, str) else w['name'] for w in spec['waypoints']], speed_mm_s=st1['speed_mm_s'],
                                   **es.path_task(name, spec))
            try:
                self._move(self._approach_pose(wps[0]), cone_T=center['T'] if center else None)
                self._begin_recording(ep)
                self._zero_and_descend(self._taught_force(center))
                self._start_event(ep)
                req = RunPath.Request()
                for T in wps:
                    req.waypoints.append(geometry.fill_pose_msg(PoseStamped().pose, T))
                req.speed_mm_s = float(st1['speed_mm_s'])
                result = {}
                th = threading.Thread(target=lambda: result.update(r=self._try_call('run_path', req)))
                th.start()
                while th.is_alive():
                    c = self.wait_cmd({'discard'}, timeout=0.2)
                    if c is not None:
                        ep['outcome'] = 'discard'
                        self.call('retract', Trigger.Request())
                        break
                th.join()
                r = result.get('r')
                if r is not None and ep.get('outcome') != 'discard':
                    ep['speed_tracking'] = {'mean_error_pct': r.speed_error_mean_pct,
                                            'max_error_pct': r.speed_error_max_pct}
                    ep['outcome'] = 'success' if r.success else 'fail'
                elif ep.get('outcome') != 'discard':
                    ep['outcome'] = 'fail'
            except Exception as e:  # noqa: BLE001  (Abort, RecorderError, OSError, ...: this episode only)
                self.get_logger().error(f"{ep['episode_id']} aborted, discarded: {e}")
                ep['outcome'] = 'discard'
                ep['notes'] = (ep['notes'] + f'\naborted: {e}').strip()
            finally:
                written = bool(ep.get('t_bag_ready_ns'))
                if written:
                    ep = self._finish(ep, task_text=ep.get('instruction') or f'Scan path {name}', post=False)
                else:
                    self._close_unstarted(ep)
                self._safe_retract()
                if written:
                    ep = self._post_process(ep)

    def _try_call(self, name, req):
        try:
            return self.call(name, req, timeout=600.0)
        except Abort as e:
            self.get_logger().error(str(e))
            return None

    def _stage2(self):
        s2 = self.scfg['stage2']
        plan_file = s2.get('plan_file')
        if not plan_file:
            raise Abort('stage2.plan_file is null in session.yaml')
        lock = read_json(self.sdir / 'session.json').get('plan_lock', {})
        if lock.get('sha256') != sha256(plan_file):
            raise Abort('plan is not locked for this session (plan_generator lock) or changed since')
        rows = read_csv(plan_file)
        sha = lock['sha256']
        prog_path = self.sdir / 'plan_progress.json'
        cap = float(s2['cap_s'])
        # progress spans sessions (gates.plan_rows_done); rows that failed QC or were discarded get a second pass
        for pass_no in (1, 2):
            pending = self._pending_rows(rows, sha)
            if not pending:
                break
            if pass_no == 2:
                self.get_logger().info(f'second pass: {len(pending)} row(s) to redo')
            for row in pending:
                self._stage2_row(row, rows, sha, prog_path, cap, s2)
        self.plan_info = dict(self.plan_info or {}, next_row=None)
        self.get_logger().info(f'plan complete ({len(self._pending_rows(rows, sha))} row(s) still not done)')

    def _pending_rows(self, rows, sha):
        done = gates.plan_rows_done(self.root, sha)
        prog = read_json(self.sdir / 'plan_progress.json')
        done |= {int(k) for k, v in prog.items() if v.get('status') == 'done'}
        return [r for r in rows if int(r['row']) not in done]

    def _stage2_row(self, row, rows, sha, prog_path, cap, s2):
        prog = read_json(prog_path)
        n_done = len(rows) - len(self._pending_rows(rows, sha))
        self.plan_info = {'file': str(self.scfg['stage2']['plan_file']), 'rows': len(rows),
                          'current_row': row['row'], 'done': n_done,
                          'redo': sum(1 for v in prog.values() if v.get('status') == 'redo'),
                          'next_row': es.row_summary(row)}
        # the operator sees the row (and stages a manual failure) before anything moves
        self.get_logger().info(f'next row {row["row"]}: {row["type"]} {row["target"]} "{row["instruction"]}"'
                               + (f' failure {row["failure_mode"]} / {row["staging_method"]}' if row['failure_mode'] else '')
                               + ': press N to start it')
        self.wait_cmd({'next'})
        while not self._presession_ready():
            self.get_logger().error(f'row not started: {self.presession_problem}. Fix it, then press N')
            self.wait_cmd({'next'})
        self.plan_info['next_row'] = None
        off = [float(row[k] or 0) for k in ('off_dx_mm', 'off_dy_mm', 'off_dz_mm',
                                             'off_rock_deg', 'off_tilt_deg', 'off_rot_deg')]
        # the session operator is who operated; the plan's assignment is kept for balance checks
        ep = self._new_episode(plan_row=int(row['row']), seed=int(row['seed']), target=row['target'],
                               type=row['type'], instruction=row['instruction'], start_offset=off,
                               planned_operator_id=row['operator_id'], plan_sha256=sha,
                               failure_mode=row['failure_mode'] or None,
                               staging_method=row['staging_method'] or None,
                               **es.row_task(row, self._waypoint_entry(row['target'])))
        if row['operator_id'] != self.operator_id:
            self.get_logger().warn(f'row {row["row"]} was planned for {row["operator_id"]}; '
                                   f'recorded as operated by {self.operator_id}')
        try:
            target = self._waypoint(row['target'])
            T_start = target['T'] @ self._offset_T(off)
            self._move(self._approach_pose(T_start), cone_T=target['T'])
            self._begin_recording(ep)
            self._zero_and_descend(self._taught_force(target))
            self._start_event(ep)
            self.event('phase', {'phase': 'search'})
            self._drain()
            self.call('teleop', SetBool.Request(data=True))
            pert = None
            if row['disturbance_kind'] not in ('', 'none'):
                pert = threading.Thread(target=self._perturb_at, args=(row, ep), daemon=True)
                pert.start()
            t_scan = now_ns()
            c = self.wait_cmd({'outcome', 'discard'}, timeout=lambda: self._cap_left(cap, t_scan))
            if c is None:
                ep['outcome'] = 'fail' if row['type'] == 'failure' else 'discard'
                ep['notes'] = 'cap reached without outcome key'
            elif c['cmd'] == 'discard':
                ep['outcome'] = 'discard'
            else:
                ep['outcome'] = {'recovered': 'recovered', 'success': 'success', 'fail': 'fail'}[c['value']]
                if ep['outcome'] in ('success', 'recovered'):
                    self.event('phase', {'phase': 'hold', 'hold_s': s2['success_hold_s']})
                    self.call('teleop', SetBool.Request(data=False))
                    time.sleep(float(s2['success_hold_s']))
        except Exception as e:  # noqa: BLE001  (Abort, RecorderError, OSError, ...: this episode only)
            self.get_logger().error(f"{ep['episode_id']} aborted, discarded: {e}")
            ep['outcome'] = 'discard'
            ep['notes'] = (ep['notes'] + f'\naborted: {e}').strip()
        finally:
            written = bool(ep.get('t_bag_ready_ns'))
            if written:
                ep = self._finish(ep, post=False)
            else:
                self._close_unstarted(ep)
            self._safe_retract()
            if written:
                ep = self._post_process(ep)
        ok = ep.get('outcome') != 'discard' and (ep.get('qc') or {}).get('pass') is True
        prog = read_json(prog_path)
        prog[row['row']] = {'status': 'done' if ok else 'redo', 'episode_id': ep['episode_id']}
        write_json_atomic(prog_path, prog)
        if not ok:
            self.get_logger().warn(f'row {row["row"]} marked for redo')

    def _perturb_at(self, row, ep):
        onset = float(row['disturbance_onset_s'] or 0)
        t0 = ep['t_start_ns']
        while (now_ns() - t0) * 1e-9 < onset:
            if self.ep is not ep:
                return
            time.sleep(0.01)
        req = InjectPerturbation.Request()
        req.kind = row['disturbance_kind']
        req.delta = [float(x) for x in json.loads(row['disturbance_delta'])]
        req.duration_s = float(row['disturbance_duration_s'])
        self.event('phase', {'phase': 'perturbation'})
        self._try_call('perturb', req)
        self.event('phase', {'phase': 'recovery'})

    # ------------------------------------------------------------------ geometry
    def _waypoint_file(self):
        from usrig_common.session import data_dir
        return data_dir(self.scfg, 'waypoints', self.sim) / f'{self.phantom_id}.yaml'

    def _waypoint(self, name):
        wps = config.load_yaml(self._waypoint_file())['waypoints']
        for w in wps:
            if w['name'] == name:
                problem = executable_problem(w, self.rig)
                if problem:
                    raise Abort(problem)
                w['T'] = geometry.pose_from_pq(w['position_m'], w['quaternion_xyzw'])
                return w
        raise Abort(f'waypoint {name} not found in {self._waypoint_file()}')

    def _waypoint_entry(self, name):
        """The taught waypoint as stored (organ, view, instructions), without the motion checks of _waypoint;
        None if it cannot be read (the row then fails in _waypoint, with its reason)."""
        try:
            return next((w for w in config.load_yaml(self._waypoint_file())['waypoints'] if w['name'] == name), None)
        except Exception:  # noqa: BLE001
            return None

    def _approach_pose(self, T):
        back = float(self.rig['motion']['approach_offset_mm']) / 1000.0
        A = T.copy()
        A[:3, 3] = T[:3, 3] - T[:3, 2] * back
        return A

    @staticmethod
    def _offset_T(off):
        """start_offset [dx, dy, dz mm, rock (about y), tilt (about x), rot (about z) deg], probe frame."""
        T = np.eye(4)
        T[:3, 3] = np.array(off[:3]) / 1000.0
        T[:3, :3] = Rotation.from_euler('yxz', [off[3], off[4], off[5]], degrees=True).as_matrix()
        return T


def main(args=None):
    rclpy.init(args=args)
    node = None

    def _abort(*_):
        if node is not None:
            node.recorder.abort()
        raise KeyboardInterrupt

    try:
        node = EpisodeManager()
        atexit.register(node.recorder.abort)
        signal.signal(signal.SIGTERM, _abort)
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        ex.spin()
    except (RuntimeError, config.UnsetLimitError) as e:
        print(f'[episode_manager] {e}', file=sys.stderr)
        sys.exit(2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ignore_sigint()
        if node is not None:
            node.recorder.abort()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
