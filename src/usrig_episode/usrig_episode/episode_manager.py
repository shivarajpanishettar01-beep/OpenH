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
                              ('collection_mode', 'robot'), ('camera', True)):
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        cfg_dir = p('config_dir') or None
        self.scfg = config.load_config('session', cfg_dir)
        if not bool(self.get_parameter('camera').value):
            # camera:=false: not started, so not recorded, not waited for, not checked by QC
            self.scfg['recording']['include_wrist_camera'] = False
        self.camera_recorded = bool(self.scfg['recording'].get('include_wrist_camera')
                                    and self.scfg['wrist_camera'].get('enabled'))
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
        self.recorder = EpisodeRecorder(self, self.scfg, self.sdir)
        self.recorder.ensure_micrus_stopped()
        self._recover_interrupted()

        update_session_json(self.sdir, session_id=self.session_id, stage=self.stage, sim=self.sim,
                            operators=self.scfg['session']['operators'], phantom_id=self.phantom_id,
                            phantom_placement=self.scfg['session'].get('phantom_placement'),
                            rig_git_commit=provenance.rig_git_commit(),
                            wrist_cam_cali_mtx=config.get_path(self.rig, 'calibration.wrist_cam_cali_mtx'))
        g = gates.check_stage(self.stage, self.scfg, self.root)
        update_session_json(self.sdir, gates={f'stage{self.stage}': {**g, 'checked_ns': now_ns()}})
        if not g['passed']:
            raise RuntimeError(f'stage {self.stage} gate has not passed: {json.dumps(g)}')
        if self.control and self.stage > 0:
            config.require_limits(self.rig)
        if not self.sim:
            ok, why = gates.presession_ok(read_json(self.sdir / 'session.json'), self.stage, self.control, now_ns(),
                                          self.scfg['session'].get('presession_max_age_h', 12))
            if not ok:
                raise RuntimeError(f'refusing to collect: {why}')

        self.worker = threading.Thread(target=self._run_stage, name='stage', daemon=True)
        self.worker.start()

    # ------------------------------------------------------------------ inputs
    def _on_cmd(self, msg):
        try:
            c = json.loads(msg.data)
        except ValueError:
            return
        cmd = c.get('cmd')
        if cmd == 'key':
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
                elif cmd in ('onset', 'recovery_start', 'recovery_end'):
                    key = {'onset': 'onset_t', 'recovery_start': 'recovery_start_t',
                           'recovery_end': 'recovery_end_t'}[cmd]
                    ep[key] = (t - ep['t_start_ns']) * 1e-9 if ep.get('t_start_ns') else None
                    if cmd == 'onset' and not ep.get('onset_source'):
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
        """Block until one of the wanted commands arrives; returns it (or None on timeout)."""
        t_end = None if timeout is None else time.monotonic() + timeout
        while rclpy.ok():
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

    def _wrist_info(self):
        cam = self.scfg['wrist_camera']
        return {'serial': cam.get('serial'), 'firmware': provenance.camera_firmware(cam.get('serial')),
                **provenance.realsense_versions(),
                'color_profile': f"{cam['color_width']}x{cam['color_height']}@{cam['fps']}",
                'depth_profile': f"{cam['depth_width']}x{cam['depth_height']}@{cam['fps']}",
                'exposure_us': cam.get('exposure_us'), 'white_balance_k': cam.get('white_balance_k'),
                'cali_ref': 'session.json:wrist_cam_cali_mtx'}

    def _new_episode(self, **fields):
        idx = self._next_index()
        ep_id = f'{self.session_id}_ep_{idx:04d}'
        ep = {
            'episode_id': ep_id, 'episode_dir': f'ep_{idx:04d}', 'session_id': self.session_id, 'sim': self.sim,
            'stage': self.stage, 'plan_row': None, 'seed': None,
            'mode': 'B' if self.stage == 2 and not self.adhoc else 'A',
            'operator_id': self.operator_id, 'phantom_id': self.phantom_id,
            'phantom_placement': self.scfg['session'].get('phantom_placement'),
            'target': None, 'type': None, 'instruction': None, 'start_offset': None,
            'disturbance': None, 'force_setpoint_n': config.get_path(self.rig, 'limits.force_setpoint_default_n'),
            'failure_mode': None, 'staging_method': None, 'onset_t': None, 'recovery_start_t': None,
            'recovered': None, 'recovery_end_t': None, 'onset_source': None,
            'outcome': None, 'success': None, 'view_reached_ns': [], 'gel_state': None, 'notes': '',
            'micrus_hdf5_path': None, 'micrus_settings': None, 'micrus_settings_constant': True,
            'rig_git_commit': provenance.rig_git_commit(),
            'micrus_version': provenance.micrus_version(self.scfg['micrus']['command']),
            'robot_tcp_setting': config.get_path(self.rig, 'robot.tcp_setting'),
            't_start_ns': None, 't_end_ns': None, 't_bag_ready_ns': None, 'duration_s': None, 'qc': None,
            'wrist_camera': self._wrist_info() if self.camera_recorded else None,
        }
        ep.update(fields)
        ep.update(self.adhoc)
        return ep

    # ------------------------------------------------------------------ episode skeleton
    def _begin_recording(self, ep):
        with self.ep_lock:
            self.ep = ep
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
        ep['outcome'] = 'discard'
        d = self.sdir / ep['episode_dir']
        if d.exists():
            write_json_atomic(d / 'episode.json', ep)
        (self.sdir / f'.{ep["episode_dir"]}.partial.json').unlink(missing_ok=True)

    def _start_event(self, ep):
        ep['t_start_ns'] = self.event('start', {k: ep.get(k) for k in
                                                ('stage', 'type', 'target', 'instruction', 'plan_row')})

    def _finish(self, ep, task_text=None):
        """end event -> bag stops -> micrus stops -> task.txt / episode.json -> QC."""
        if ep.get('t_start_ns') is None:
            ep['t_start_ns'] = ep.get('t_bag_ready_ns')
        ep['t_end_ns'] = self.event('end', {'outcome': ep.get('outcome')})
        if ep.get('outcome'):
            self.event('outcome', {'outcome': ep['outcome']})
        time.sleep(0.2)   # let the end / outcome events reach the bag
        rc, _ = self.recorder.stop()
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

    def _move(self, T, frame='phantom'):
        req = MoveToPose.Request()
        req.target = PoseStamped()
        req.target.header.frame_id = frame
        geometry.fill_pose_msg(req.target.pose, T)
        self.call('move', req)
        if frame == 'phantom':
            self._set_local_tilt(T)

    def _set_local_tilt(self, T_phantom):
        """D10: tilt cone around this waypoint's probe axis, sent to robot_control (base frame)."""
        reg = read_json(self.sdir / 'session.json').get('phantom_registration', {})
        if 'T_base_phantom' not in reg:
            return
        axis = (np.array(reg['T_base_phantom'])[:3, :3] @ np.asarray(T_phantom)[:3, 2]).tolist()
        self.pub_cmd.publish(String(data=json.dumps({'cmd': 'set_local_tilt_axis', 'axis_base': axis})))

    # ------------------------------------------------------------------ stages
    def _run_stage(self):
        try:
            if self.collection_mode == 'freedrive':
                self._freedrive()
                return
            {0: self._stage0, 1: self._stage1, 2: self._stage2}[self.stage]()
        except Abort as e:
            self.get_logger().error(f'stage stopped: {e}')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'stage error: {e!r}')
        finally:
            self.recorder.abort()

    def _stage0(self):
        cap = float(self.scfg['stage0']['max_duration_s'])
        while rclpy.ok():
            self.get_logger().info('Stage 0: press B (start_stop) to start an episode')
            self._drain()
            self.wait_cmd({'start_stop'})
            ep = self._new_episode(type='shakedown')
            try:
                wp_name = self.scfg['stage0'].get('approach_waypoint')
                if wp_name and self.control:
                    self._move(self._approach_pose(self._waypoint(wp_name)['T']))
                self._begin_recording(ep)
                if self.control:
                    self._zero_and_descend()
                    self.call('teleop', SetBool.Request(data=True))
                self._start_event(ep)
                c = self.wait_cmd({'start_stop', 'discard', 'outcome'}, timeout=cap)
                while c is not None and c['cmd'] == 'outcome':
                    ep['outcome'] = c['value']
                    c = self.wait_cmd({'start_stop', 'discard', 'outcome'},
                                      timeout=max(0.0, cap - (now_ns() - ep['t_start_ns']) * 1e-9))
                if c is None:
                    ep['notes'] = (ep['notes'] + '\nstopped by stage0.max_duration_s').strip()
                elif c['cmd'] == 'discard':
                    ep['outcome'] = 'discard'
            except (Abort, RecorderError) as e:
                self.get_logger().error(f"{ep['episode_id']} aborted, discarded: {e}")
                ep['outcome'] = 'discard'
                ep['notes'] = (ep['notes'] + f'\naborted: {e}').strip()
            finally:
                if ep.get('t_bag_ready_ns'):
                    self._finish(ep)
                else:
                    self._close_unstarted(ep)
                if self.control:
                    self._safe_retract()

    def _freedrive(self):
        """Kinesthetic episodes: the operator moves the arm with the freedrive pedal; nothing is commanded."""
        cap = float(self.scfg['stage0']['max_duration_s'])
        grip = config.get_path(self.scfg, 'freedrive.grip')
        while rclpy.ok():
            self.get_logger().info('Freedrive: probe in free air, press B to start; hold the pedal to guide '
                                   'the arm; B again to stop')
            self._drain()
            self.wait_cmd({'start_stop'})
            if self.count_publishers('/robot/control_status') > 0:
                self.get_logger().error('robot_control is running: stop it for freedrive collection (episode not started)')
                continue
            ep = self._new_episode(type='kinesthetic', collection_mode='freedrive', grip=grip,
                                   force_label_valid=grip == 'wrist_handle', force_setpoint_n=None)
            try:
                self._begin_recording(ep)          # includes the free-air hold (F/T offset window)
                self._start_event(ep)
                c = self.wait_cmd({'start_stop', 'discard', 'outcome'}, timeout=cap)
                while c is not None and c['cmd'] == 'outcome':
                    ep['outcome'] = c['value']
                    c = self.wait_cmd({'start_stop', 'discard', 'outcome'},
                                      timeout=max(0.0, cap - (now_ns() - ep['t_start_ns']) * 1e-9))
                if c is None:
                    ep['notes'] = (ep['notes'] + '\nstopped by stage0.max_duration_s').strip()
                elif c['cmd'] == 'discard':
                    ep['outcome'] = 'discard'
            except (Abort, RecorderError) as e:
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
            for e in spec['waypoints']:
                if isinstance(e, str):
                    self._waypoint(e)          # named waypoints must be verified for this rig configuration
            wps = resolve_path(spec['waypoints'], config.load_yaml(self._waypoint_file())['waypoints'])
            ep = self._new_episode(type='scripted', target=name, instruction=spec.get('instruction'),
                                   path_waypoints=[w if isinstance(w, str) else w['name'] for w in spec['waypoints']], speed_mm_s=st1['speed_mm_s'])
            try:
                self._move(self._approach_pose(wps[0]))
                self._begin_recording(ep)
                self._zero_and_descend()
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
            except (Abort, RecorderError) as e:
                self.get_logger().error(f"{ep['episode_id']} aborted, discarded: {e}")
                ep['outcome'] = 'discard'
                ep['notes'] = f'aborted: {e}'
            finally:
                if ep.get('t_bag_ready_ns'):
                    self._finish(ep, task_text=ep.get('instruction') or f'Scan path {name}')
                else:
                    self._close_unstarted(ep)
                self._safe_retract()

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
        prog_path = self.sdir / 'plan_progress.json'
        cap = float(s2['cap_s'])
        for row in rows:
            prog = read_json(prog_path)
            if prog.get(row['row'], {}).get('status') == 'done':
                continue
            off = [float(row[k] or 0) for k in ('off_dx_mm', 'off_dy_mm', 'off_dz_mm',
                                                 'off_rock_deg', 'off_tilt_deg', 'off_rot_deg')]
            ep = self._new_episode(plan_row=int(row['row']), seed=int(row['seed']), target=row['target'],
                                   type=row['type'], instruction=row['instruction'], start_offset=off,
                                   operator_id=row['operator_id'], failure_mode=row['failure_mode'] or None,
                                   staging_method=row['staging_method'] or None)
            self.get_logger().info(f'row {row["row"]}: {row["type"]} {row["target"]} '
                                   f'"{row["instruction"]}" operator {row["operator_id"]}')
            try:
                T_start = self._waypoint_T(row['target']) @ self._offset_T(off)
                self._move(self._approach_pose(T_start))
                self._begin_recording(ep)
                self._zero_and_descend()
                self._start_event(ep)
                self.event('phase', {'phase': 'search'})
                self._drain()
                self.call('teleop', SetBool.Request(data=True))
                pert = None
                if row['disturbance_kind'] not in ('', 'none'):
                    pert = threading.Thread(target=self._perturb_at, args=(row, ep), daemon=True)
                    pert.start()
                c = self.wait_cmd({'outcome', 'discard'}, timeout=cap)
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
            except (Abort, RecorderError) as e:
                self.get_logger().error(f"{ep['episode_id']} aborted, discarded: {e}")
                ep['outcome'] = 'discard'
                ep['notes'] = f'aborted: {e}'
            finally:
                if ep.get('t_bag_ready_ns'):
                    ep = self._finish(ep)
                else:
                    self._close_unstarted(ep)
                self._safe_retract()
            ok = ep.get('outcome') != 'discard' and (ep.get('qc') or {}).get('pass') is True
            prog = read_json(prog_path)
            prog[row['row']] = {'status': 'done' if ok else 'redo', 'episode_id': ep['episode_id']}
            write_json_atomic(prog_path, prog)
            if not ok:
                self.get_logger().warn(f'row {row["row"]} marked for redo')
            self.get_logger().info('press N for the next row')
            self.wait_cmd({'next'})
        self.get_logger().info('plan complete')

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

    def _waypoint_T(self, name):
        return self._waypoint(name)['T']

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
