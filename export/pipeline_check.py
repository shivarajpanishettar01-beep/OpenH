#!/usr/bin/env python3
"""End-to-end export check on SYNTHETIC episodes with known ground truth (no hardware).

  source scripts/env.sh
  python export/pipeline_check.py run [--work DIR] [--keep]

Writes two synthetic episodes as MCAP bags with every topic the rig records (ultrasound, frame info,
robot state 500 Hz, executed twist 125 Hz, wrist colour + 16-bit depth 30 Hz, camera info, events),
plus episode.json / session.json / annotation.json marked ``synthetic_pipeline_check: true``. Then runs
the real chain:

  stage_bags.py --pipeline-check -> bag_to_lerobot.py (v2.1) -> openh_export.py --pipeline-check
  (Open-H fields, LeRobot 0.6.0 v2.1 -> v3.0 converter) -> reload with LeRobotDataset

and compares the reloaded dataset with the ground truth: incremental TCP actions (recomputed from the
known trajectory), executed twist, force setpoint, expert flag, ultrasound frame selection and pixel
values (lossy video, tolerance), ultrasound host stamps, lossless 16-bit depth, g_t status labels,
instruction, splits and timestamps.

The work folder defaults to <session.sim_root>/checks/pipeline_check (never the real data root). The
dataset is named SYNTHETIC_*: stage_bags.py and openh_export.py refuse synthetic episodes without
--pipeline-check and refuse real episodes with it.
`verify` is run by `run` in the conda env openh-export (Python 3.12, LeRobot 0.6.0).
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
T0_NS = 1_800_000_000 * 10**9          # synthetic epoch: 2027-01-15, far from any real recording
DURATION_S = 4.0
US_HZ = 27.3                            # not a divisor of the dataset fps: no frame lands on a step
FPS = 25
DOWN = np.diag([1.0, -1.0, -1.0])
EPISODES = [  # (episode_id, type, stage, instruction, failure interval s, recovery interval s)
    ('ep_0001', 'success', 2, 'Find the long-axis view of the vessel', None, None),
    ('ep_0002', 'recovery-mid', 2, 'Recover the transverse view after the probe slips', (1.5, 2.2), (2.2, 3.0)),
    ('ep_0003', 'kinesthetic', 0, 'Guide the probe to the long-axis view by hand', None, None),
]
KINESTHETIC = {'ep_0003'}      # freedrive: no command (NaN marker on /robot/cmd_executed), no setpoint


# ------------------------------------------------------------------------------- ground truth
def tcp_pose(tau):
    p = np.array([0.40 + 0.010 * tau, 0.02 * math.sin(0.8 * tau), 0.20 - 0.003 * tau])
    R = (Rotation.from_euler('z', 8.0 * tau, degrees=True) * Rotation.from_euler('x', 5.0 * math.sin(tau), degrees=True)
         ).as_matrix() @ DOWN
    return p, R


def twist(tau):
    return [0.010, 0.016 * math.cos(0.8 * tau), -0.003, 0.0, 0.0, math.radians(8.0)]


def force_setpoint(tau):
    return 3.0 + 0.5 * tau


def expert(tau):
    return bool(1.0 <= tau < 3.0)


def us_value(k):
    return 20 + (7 * k) % 200


def depth_value(j):
    return 500 + j


def stamps():
    """Sample times (ns since T0) per stream, chosen so no stream sample sits on a dataset step."""
    s = {}
    s['robot'] = np.arange(0, int(DURATION_S * 1e9), 2_000_000)                         # 500 Hz
    s['cmd'] = 4_000_000 + np.arange(0, int(DURATION_S * 1e9) - 4_000_000, 8_000_000)  # 125 Hz
    s['us'] = 121_400_000 + (np.arange(int((DURATION_S - 0.13) * US_HZ)) * 1e9 / US_HZ).astype(np.int64)
    s['cam'] = 10_100_000 + (np.arange(int((DURATION_S - 0.02) * 30)) * 1e9 / 30).astype(np.int64)
    return s


# ------------------------------------------------------------------------------- writing (openh env)
def write_bag(bag_dir, ep):
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image, JointState
    from std_msgs.msg import String
    from usrig_interfaces.msg import EpisodeEvent, RobotState

    def tm(ns_rel):
        t = T0_NS + int(ns_rel)
        return Time(sec=t // 10**9, nanosec=t % 10**9)

    msgs = []   # (t_ns_abs, topic, msg)
    st = stamps()
    kin = ep[0] in KINESTHETIC
    for t in st['robot']:
        tau = t * 1e-9
        p, R = tcp_pose(tau)
        q = Rotation.from_matrix(R).as_quat()
        ps = PoseStamped()
        ps.header.stamp, ps.header.frame_id = tm(t), 'base'
        (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) = map(float, p)
        (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w) = map(float, q)
        js = JointState()
        js.header.stamp = tm(t)
        js.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                   'wrist_3_joint']
        js.position = [0.1 * tau + i for i in range(6)]
        js.velocity = [0.1] * 6           # as robot_state_cpp: rosbags rejects empty trailing arrays
        js.effort = [0.0] * 6
        w = WrenchStamped()
        w.header.stamp = tm(t)
        w.wrench.force.z = 2.0 + tau
        rs = RobotState()
        rs.header.stamp = tm(t)
        rs.controller_time_s = tau
        rs.tcp_base = ps.pose
        rs.wrench = [0.0, 0.0, 2.0 + tau, 0.0, 0.0, 0.0]
        if kin:              # as robot_state_cpp labels freedrive: pedal held for the whole episode
            rs.force_setpoint_n = math.nan
            rs.mode, rs.action_source, rs.action_is_expert = (RobotState.MODE_FREEDRIVE,
                                                              RobotState.SOURCE_KINESTHETIC, True)
        else:
            rs.force_setpoint_n = force_setpoint(tau)
            rs.mode = RobotState.MODE_TELEOP if expert(tau) else RobotState.MODE_MOVING
            rs.action_source = RobotState.SOURCE_TELEOP if expert(tau) else RobotState.SOURCE_TRANSIT
            rs.action_is_expert = expert(tau)
        msgs += [(t, '/robot/tcp_pose', ps), (t, '/joint_states', js), (t, '/robot/wrench', w), (t, '/robot/state', rs)]
    for t in st['cmd']:
        m = TwistStamped()
        m.header.stamp, m.header.frame_id = tm(t), ('none' if kin else 'probe')
        v = [math.nan] * 6 if kin else twist(t * 1e-9)
        (m.twist.linear.x, m.twist.linear.y, m.twist.linear.z, m.twist.angular.x, m.twist.angular.y,
         m.twist.angular.z) = v
        msgs.append((t, '/robot/cmd_executed', m))
    for k, t in enumerate(st['us']):
        im = Image()
        im.header.stamp, im.header.frame_id = tm(t), 'ultrasound'
        im.height, im.width, im.encoding, im.step = 96, 128, 'mono8', 128
        im.data = bytes([us_value(k)]) * (96 * 128)
        fi = {'stamp': (T0_NS + int(t)) * 1e-9, 'frame_index': 1000 + k,
              'image': {'pixel_spacing_mm': [0.1, 0.1], 'origin_mm': [-6.4, 0.0], 'width': 128, 'height': 96},
              'geometry': {'n_lines': 128},
              'settings': {'depth_mm': 40, 'gain': 64, 'power_db': -3, 'frequency_hz': 12000000, 'focus_mm': 16,
                           'dynamic_range_db': 72, 'tgc': [40, 45, 50, 55, 55], 'view_area': 100}}
        msgs += [(t, '/micrus/bmode', im), (t + 1000, '/micrus/frame_info', String(data=json.dumps(fi)))]
    ci = CameraInfo()
    ci.height, ci.width, ci.distortion_model = 48, 64, 'plumb_bob'
    ci.k = [60.0, 0.0, 32.0, 0.0, 60.0, 24.0, 0.0, 0.0, 1.0]
    ci.d = [0.0] * 5
    ci.p = [60.0, 0.0, 32.0, 0.0, 0.0, 60.0, 24.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    for j, t in enumerate(st['cam']):
        c = Image()
        c.header.stamp, c.header.frame_id = tm(t), 'd415_color_optical_frame'
        c.height, c.width, c.encoding, c.step = 48, 64, 'rgb8', 64 * 3
        c.data = bytes([(10 + 3 * j) % 256, 128, 60]) * (48 * 64)
        d = Image()
        d.header = c.header
        d.height, d.width, d.encoding, d.step = 48, 64, '16UC1', 64 * 2
        d.data = np.full((48, 64), depth_value(j), np.uint16).tobytes()
        ci.header = c.header
        msgs += [(t, '/wrist/d415/color/image_raw', c), (t, '/wrist/d415/aligned_depth_to_color/image_raw', d),
                 (t + 1, '/wrist/d415/color/camera_info', ci), (t + 1, '/wrist/d415/aligned_depth_to_color/camera_info', ci)]
    for t, kind, value in ((0, 'start', {}), (int(3.9e9), 'outcome', {'outcome': 'success'}), (int(3.95e9), 'end', {})):
        e = EpisodeEvent()
        e.header.stamp, e.episode_id, e.kind, e.value = tm(t), ep[0], kind, json.dumps(value)
        msgs.append((t, '/episode/events', e))

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for topic, typ in {'/robot/tcp_pose': 'geometry_msgs/msg/PoseStamped', '/joint_states': 'sensor_msgs/msg/JointState',
                       '/robot/wrench': 'geometry_msgs/msg/WrenchStamped', '/robot/state': 'usrig_interfaces/msg/RobotState',
                       '/robot/cmd_executed': 'geometry_msgs/msg/TwistStamped', '/micrus/bmode': 'sensor_msgs/msg/Image',
                       '/micrus/frame_info': 'std_msgs/msg/String', '/wrist/d415/color/image_raw': 'sensor_msgs/msg/Image',
                       '/wrist/d415/aligned_depth_to_color/image_raw': 'sensor_msgs/msg/Image',
                       '/wrist/d415/color/camera_info': 'sensor_msgs/msg/CameraInfo',
                       '/wrist/d415/aligned_depth_to_color/camera_info': 'sensor_msgs/msg/CameraInfo',
                       '/episode/events': 'usrig_interfaces/msg/EpisodeEvent'}.items():
        w.create_topic(rosbag2_py.TopicMetadata(name=topic, type=typ, serialization_format='cdr'))
    for t, topic, m in sorted(msgs, key=lambda x: x[0]):
        w.write(topic, serialize_message(m), T0_NS + int(t))
    del w


def write_session(root):
    sess = root / 'SYNTHETIC_session'
    sess.mkdir(parents=True)
    (sess / 'session.json').write_text(json.dumps({'session_id': 'SYNTHETIC_session', 'sim': False,
                                                   'synthetic_pipeline_check': True}, indent=1))
    for ep in EPISODES:
        eid, typ, stage, instr, fail, rec = ep
        d = sess / eid
        write_bag(d, ep)
        ann = {'status_intervals': [], 'phases': [{'phase': 'search', 't0_ns': T0_NS, 't1_ns': None}], 'gestures': []}
        if fail:
            ann['status_intervals'] = [
                {'status': 'failure', 't0_ns': T0_NS + int(fail[0] * 1e9), 't1_ns': T0_NS + int(fail[1] * 1e9)},
                {'status': 'recovery', 't0_ns': T0_NS + int(rec[0] * 1e9), 't1_ns': T0_NS + int(rec[1] * 1e9)}]
            ann['failure_class'] = {'value': 'probe_slip'}
        (d / 'annotation.json').write_text(json.dumps(ann, indent=1))
        (d / 'task.txt').write_text(instr)
        extra = ({'collection_mode': 'freedrive', 'grip': 'wrist_handle', 'force_label_valid': True}
                 if eid in KINESTHETIC else {'collection_mode': 'robot'})
        extra.update({'operator_id': 'op_synthetic', 'phantom_id': 'vascular_4vein', 'rig_git_commit': 'synthetic'})
        (d / 'episode.json').write_text(json.dumps({**extra,
            'episode_id': eid, 'session_id': 'SYNTHETIC_session', 'type': typ, 'stage': stage,
            'instruction': instr, 'outcome': 'recovered' if fail else 'success', 'qc': {'pass': True},
            'failure_mode': 'FM-SYN' if fail else None, 'sim': False, 'synthetic_pipeline_check': True,
            'time_offsets_s': {'ultrasound': -0.020, 'camera': 0.0}, 'capture_host': 'synthetic'}, indent=1))
    return sess


def run(a):
    import yaml
    sys.path.insert(0, str(REPO / 'src' / 'usrig_common'))
    from usrig_common import config
    from usrig_common.session import data_dir
    scfg = config.load_config('session')
    work = Path(a.work).expanduser() if a.work else data_dir(scfg, 'checks', sim=True) / 'pipeline_check'
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    t_start = time.monotonic()
    print(f'[1/6] synthetic episodes -> {work}')
    sess = write_session(work)

    print('[2/6] stage_bags.py --pipeline-check')
    staging = work / 'staging'
    sh([sys.executable, str(HERE / 'stage_bags.py'), '--sessions', str(sess), '--include-stage0', '--include-stage1',
        '--pipeline-check', '-o', str(staging)])

    print('[3/6] bag_to_lerobot.py (v2.1)')
    conv = os.environ.get('BAG_TO_LEROBOT')
    if not conv or not Path(conv).exists():
        sys.exit('BAG_TO_LEROBOT not set or missing (source scripts/env.sh)')
    v21 = work / 'v21'
    sh([sys.executable, conv, '-c', 'lerobot_rig.yaml', '-o', str(v21), str(staging)], cwd=HERE)
    srcs = [json.loads(x) for x in (v21 / 'meta' / 'micrus_sources.jsonl').read_text().splitlines() if x.strip()]
    lengths = {json.loads(x)['episode_index']: json.loads(x)['length']
               for x in (v21 / 'meta' / 'episodes.jsonl').read_text().splitlines() if x.strip()}

    print('[4/6] openh_export.py --pipeline-check (LeRobot 0.6.0 env)')
    rig = yaml.safe_load((REPO / 'src/usrig_bringup/config/rig.yaml').read_text())
    rig.setdefault('probe', {})['type'] = 'SYNTHETIC-PROBE'
    rig.setdefault('calibration', {})['prob_cali_mtx'] = np.eye(4).tolist()
    rig['calibration']['wrist_cam_cali_mtx'] = np.eye(4).tolist()
    rig_test = work / 'rig_pipeline_check.yaml'           # test values for this check only, never the rig's
    rig_test.write_text(yaml.safe_dump(rig, sort_keys=False))
    out = work / 'SYNTHETIC_openh_pipeline_check'
    py312 = a.export_python
    sh([py312, str(HERE / 'openh_export.py'), '--v21', str(v21), '--order', str(work / 'staging_order.json'),
        '--out', str(out), '--rig-config', str(rig_test), '--pipeline-check'])

    truth = {'t0_ns': T0_NS, 'fps': FPS, 'episodes': [
        {'episode_index': s['episode_index'], 'episode_id': Path(s['bag']).name, 'start_unix': s['start_unix'],
         'length': lengths[s['episode_index']]} for s in srcs],
        'us_time_offset_s': -0.020, 'image_offset_s': -0.020, 'depth_offset_s': 0.0}
    (work / 'truth.json').write_text(json.dumps(truth, indent=1))
    print('[5/6] reload and compare (LeRobot 0.6.0 env)')
    sh([py312, str(Path(__file__).resolve()), 'verify', '--dataset', str(out), '--truth', str(work / 'truth.json')])
    print('[6/6] dataset card + checksum manifest (release_card.py)')
    sh([py312, str(HERE / 'release_card.py'), '--dataset', str(out)])
    card = (out / 'README.md').read_text()
    prov = [json.loads(x) for x in (out / 'meta' / 'provenance.jsonl').read_text().splitlines() if x.strip()]
    problems = [what for ok, what in (
        ('SYNTHETIC PIPELINE-CHECK DATA' in card, 'card has the SYNTHETIC banner'),
        (f'- Episodes: {len(EPISODES)} ' in card, 'card episode count'),
        ("'freedrive': 1" in card and "'robot': 2" in card, 'card counts the collection modes'),
        ('Operators: 1; sessions: 1' in card, 'card counts operators / sessions'),
        ('prob_cali_mtx.json`: present' in card, 'card sees the calibration files'),
        (len(prov) == len(EPISODES) and {x['episode_id'] for x in prov} == {e[0] for e in EPISODES}, 'provenance.jsonl rows'),
    ) if not ok]
    sh([py312, str(HERE / 'release_card.py'), '--dataset', str(out), '--check'])
    victim = out / 'meta' / 'annotations.jsonl'
    keep = victim.read_bytes()
    victim.write_bytes(keep + b' ')
    tampered = subprocess.run([py312, str(HERE / 'release_card.py'), '--dataset', str(out), '--check'],
                              capture_output=True, text=True)
    victim.write_bytes(keep)
    if tampered.returncode == 0 or 'changed: meta/annotations.jsonl' not in tampered.stdout:
        problems.append('manifest did not catch a modified file')
    if problems:
        sys.exit(f'PIPELINE CHECK FAILED (card / manifest): {problems}')
    print(f'PIPELINE CHECK PASSED in {time.monotonic() - t_start:.0f} s ({out})')
    if not a.keep:
        shutil.rmtree(work)


def sh(cmd, cwd=None):
    r = subprocess.run(cmd, cwd=cwd)
    if r.returncode:
        sys.exit(f'FAILED ({r.returncode}): {" ".join(map(str, cmd))}')


# ------------------------------------------------------------------------------- verify (openh-export env)
def hold(stamps_s, t):
    return np.clip(np.searchsorted(stamps_s, t, side='right') - 1, 0, len(stamps_s) - 1)


def verify(a):
    sys.path.insert(0, str(HERE))
    import openh_fields as F
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    truth = json.loads(Path(a.truth).read_text())
    out = Path(a.dataset)
    ds = LeRobotDataset(f'local/{out.name}', root=out, video_backend='pyav')
    info = ds.meta.info
    fails = []

    def check(ok, what):
        print(('  ok    ' if ok else '  FAIL  ') + what)
        if not ok:
            fails.append(what)

    check(info.get('codebase_version', '').startswith('v3'), f'codebase_version {info.get("codebase_version")}')
    check('SYNTHETIC' in str(json.loads((out / 'meta' / 'info.json').read_text()).get('pipeline_check', '')),
          'info.json marks the dataset as synthetic')
    feats = info.features if hasattr(info, 'features') else info['features']
    meta = out / 'meta'
    for f in ('calibration/prob_cali_mtx.json', 'calibration/wrist_cali_mtx.json', 'calibration/camera_intrinsics.json',
              'annotations.jsonl', 'provenance.jsonl'):
        check((meta / f).exists(), f'meta/{f} survives the v3.0 conversion')
    raw_info = json.loads((meta / 'info.json').read_text())
    ann = [json.loads(x) for x in (meta / 'annotations.jsonl').read_text().splitlines() if x.strip()] \
        if (meta / 'annotations.jsonl').exists() else []
    check(len(ann) == len(EPISODES) and any(a.get('failure_class', {}).get('value') == 'probe_slip' for a in ann),
          'annotations.jsonl content')
    for k in ('action', 'observation.state', 'observation.images.ultrasound', 'observation.images.wrist_camera',
              'observation.images.wrist_depth', 'observation.meta.status', 'observation.meta.host_stamp_ns',
              'observation.meta.commanded_twist', 'observation.meta.force_setpoint_n',
              'observation.meta.action_is_expert', 'observation.meta.command_valid', 'instruction.text'):
        check(k in feats, f'feature {k}')
    act = raw_info['features']['action']
    check(list(act['shape']) == [6] and act.get('names') == F.ACTION_NAMES and act['info']['frame'] == 'base',
          'action is float32[6] dx..drz, base frame, rotation vector')
    dep = raw_info['features']['observation.images.wrist_depth']
    check(dep['dtype'] == 'image' and dep['info'] == {'encoding': 'png16', 'units': 'mm'}, 'wrist_depth is a 16-bit PNG image in mm')
    check(ds.num_episodes == len(truth['episodes']), f'{ds.num_episodes} episodes')
    check(ds.num_frames == sum(e['length'] for e in truth['episodes']), f'{ds.num_frames} frames')
    print(f'  info splits: {info.get("splits")}')
    check(dict(info.get('splits') or {}) == {'train': '0:2', 'recovery': '2:3'},
          'splits: success + kinesthetic in train, recovery contiguous')
    st_a = ds.meta.stats.get('action', {})
    check(all(np.isfinite(np.asarray(st_a.get(k, [np.nan]), float)).all() for k in ('mean', 'std', 'min', 'max')),
          'action statistics finite (NaN no-command steps do not leak into the action)')

    st = stamps()
    t_robot = (T0_NS + st['robot']) * 1e-9
    t_cmd = (T0_NS + st['cmd']) * 1e-9
    t_us = (T0_NS + st['us']) * 1e-9 + truth['image_offset_s']
    t_cam_ns = T0_NS + st['cam']
    poses = [tcp_pose(t * 1e-9) for t in st['robot']]
    P = np.array([[*p, *Rotation.from_matrix(R).as_quat()] for p, R in poses])
    worst = {'action': 0.0, 'twist': 0.0, 'us_px': 0.0}
    gidx = 0
    for e in truth['episodes']:
        n = e['length']
        ticks = e['start_unix'] + np.arange(n) / truth['fps']
        ir = hold(t_robot, ticks)
        exp_action = F.delta_actions(P[ir], 'base')
        exp_twist = np.array([twist(st['cmd'][i] * 1e-9) for i in hold(t_cmd, ticks)])
        iu = hold(t_us, ticks)
        t_steps_ns = ((float(e['start_unix']) + np.arange(n) / truth['fps']) * 1e9).astype(np.int64)
        id_ = F.sample_hold(t_cam_ns, t_steps_ns)
        ep_fail = e['episode_id'] == 'ep_0002'
        kin = e['episode_id'] in KINESTHETIC
        for k in range(n):
            item = ds[gidx + k]
            act = item['action'].numpy()
            worst['action'] = max(worst['action'], float(np.abs(act - exp_action[k]).max()))
            tw = item['observation.meta.commanded_twist'].numpy()
            tau = st['robot'][ir[k]] * 1e-9
            fsp = float(item['observation.meta.force_setpoint_n'].numpy().ravel()[0])
            valid = bool(item['observation.meta.command_valid'])
            if kin:                 # no command, no setpoint: command_valid false (values 0); always expert
                if valid or np.abs(tw).max() != 0 or fsp != 0:
                    fails.append(f'ep {e["episode_id"]} step {k}: kinesthetic step not flagged command_valid=false')
                if not bool(item['observation.meta.action_is_expert']):
                    fails.append(f'ep {e["episode_id"]} step {k}: kinesthetic not expert')
            else:
                if not valid:
                    fails.append(f'ep {e["episode_id"]} step {k}: commanded step flagged invalid')
                worst['twist'] = max(worst['twist'], float(np.abs(tw - exp_twist[k]).max()))
                if abs(fsp - force_setpoint(tau)) > 1e-4:
                    fails.append(f'ep {e["episode_id"]} step {k}: force setpoint')
                if bool(item['observation.meta.action_is_expert']) != expert(tau):
                    fails.append(f'ep {e["episode_id"]} step {k}: action_is_expert')
            us = item['observation.images.ultrasound'].numpy()            # C, H, W float in [0, 1]
            worst['us_px'] = max(worst['us_px'], abs(float(us.mean()) * 255.0 - us_value(int(iu[k]))))
            if int(np.asarray(item['observation.meta.host_stamp_ns']).ravel()[0]) != int(T0_NS + st['us'][iu[k]]):
                fails.append(f'ep {e["episode_id"]} step {k}: ultrasound host stamp')
            dep = item['observation.images.wrist_depth']
            dep = np.asarray(dep.numpy() if hasattr(dep, 'numpy') else dep)
            dval = np.unique(np.round(dep * (65535.0 if dep.max() <= 1.0 else 1.0)).astype(np.int64))
            if dval.tolist() != [depth_value(int(id_[k]))]:
                fails.append(f'ep {e["episode_id"]} step {k}: depth {dval[:3].tolist()} != {depth_value(int(id_[k]))}')
            tau_step = (ticks[k] * 1e9 - T0_NS) * 1e-9
            want = 'nominal'
            if ep_fail and 1.5 <= tau_step < 2.2:
                want = 'failure'
            elif ep_fail and 2.2 <= tau_step < 3.0:
                want = 'recovery'
            if item['observation.meta.status'] != want:
                fails.append(f'ep {e["episode_id"]} step {k}: status {item["observation.meta.status"]} != {want}')
        check(abs(float(ds[gidx]['timestamp']) - 0.0) < 1e-6 and
              abs(float(ds[gidx + n - 1]['timestamp']) - (n - 1) / truth['fps']) < 1e-4, f'{e["episode_id"]}: timestamps')
        check(ds[gidx]['instruction.text'] in [x[3] for x in EPISODES], f'{e["episode_id"]}: instruction text')
        gidx += n
    check(worst['action'] < 1e-6, f'incremental TCP action vs ground truth, max error {worst["action"]:.2e}')
    check(worst['twist'] < 1e-6, f'executed twist (commanded_twist) vs ground truth, max error {worst["twist"]:.2e}')
    check(worst['us_px'] <= 3.0, f'ultrasound frame selection + pixels (h264), max error {worst["us_px"]:.2f} grey levels')
    per_step = [f for f in fails if ' step ' in f]
    check(not per_step, f'per-step force setpoint / expert flag / command_valid / host stamp / depth / status: '
                        f'{len(per_step)} mismatches {per_step[:3]}')
    # LeRobot query with history, as a training loader would use it
    q = LeRobotDataset(f'local/{out.name}', root=out, video_backend='pyav',
                       delta_timestamps={'action': [-2 / truth['fps'], -1 / truth['fps'], 0.0]})
    check(tuple(q[5]['action'].shape) == (3, 6), 'delta_timestamps query on action')
    if fails:
        sys.exit(f'PIPELINE CHECK FAILED: {len(fails)} problems')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--work', help='default: <session.sim_root>/checks/pipeline_check')
    r.add_argument('--keep', action='store_true', help='keep the work folder')
    r.add_argument('--export-python', default=str(Path.home() / 'miniconda3/envs/openh-export/bin/python'))
    v = sub.add_parser('verify')
    v.add_argument('--dataset', required=True)
    v.add_argument('--truth', required=True)
    a = ap.parse_args()
    run(a) if a.cmd == 'run' else verify(a)


if __name__ == '__main__':
    main()
