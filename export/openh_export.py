#!/usr/bin/env python3
"""Open-H V2 export: LeRobot v2.1 (from bag_to_lerobot) -> Open-H fields -> LeRobot v3.0.

Follows the submitted proposal (IIT Mandi, "A Robotic Ultrasound Contribution to Open-H
Embodiment V2"). GR00T-H is out of scope here. Run in the Python 3.12 env with LeRobot 0.6.0
(conda env `openh-export`):

  python openh_export.py --v21 ~/datasets/us_v21 --order ~/datasets/staging_order.json \
      --out ~/datasets/iitmandi_us [--action-frame base|tool] [--depth png|none]

Per step (added to every episode parquet and to meta/info.json):
  action                                 float32[6]  [dx, dy, dz, drx, dry, drz]: TCP pose delta to the
                                                     next step, rotation vector (proposal eq. 2)
  observation.meta.commanded_twist       float32[6]  executed probe-frame twist (the rig's raw action)
  observation.meta.force_setpoint_n      float32[1]
  observation.meta.command_valid         bool        false where nothing was commanded (freedrive by hand,
                                                     robot_control off): twist and setpoint are 0 there and
                                                     must be ignored (NaN would become null in LeRobot v3.0)
  observation.meta.in_episode            bool        step between the start and end events; false on the pre-roll
                                                     (readiness, free-air hold, descent) and post-roll the bag holds
  observation.meta.paused                bool        inside an operator pause (robot held; episode.json pauses_ns)
  observation.meta.action_valid          bool        action joins two in-episode, unpaused steps; false on the last
                                                     step (zero action, no successor): train on action_valid only
  observation.images.wrist_depth        image       16-bit PNG, millimetres, aligned to colour (lossless)
  observation.meta.force_torque          float32[6]
  observation.meta.probe_type            string
  observation.meta.probe_acquisition_param float32[N] (= observation.ultrasound.settings)
  observation.meta.prob_cali_mtx         float32[16] this episode's session calibration (session.json snapshot;
                                                     the exporting PC's rig.yaml only as a recorded fallback)
  observation.meta.wrist_cali_mtx        float32[16] T_flange_camera: D415 colour optical frame in the UR flange
                                                     (tool0) frame (hand-eye), per session as above
  observation.meta.host_stamp_ns         int64       real header stamp of the ultrasound frame used
  observation.meta.status                string      g_t: nominal | failure | recovery
  observation.meta.phase                 string      proposal phases (automated or annotated)
  observation.meta.gesture               string      active gestures, '+'-joined
  observation.meta.{status,phase,gesture}_source string provenance of the label that won at this step
                                                     (expert | model_assisted | trainee | automated | '' = none):
                                                     QA-rejected entries dropped, expert > model_assisted >
                                                     trainee > automated where entries overlap
  observation.meta.action_is_expert      bool
  observation.meta.stage                 int64
  observation.meta.failure_mode          string      plan failure-mode id
  observation.meta.failure_class         string      one of the four proposal failure classes on failure / recovery
                                                     episodes (resolved label, else from the plan FM id); the
                                                     export stops if such an episode has none
  instruction.text                       string
Meta:
  meta/calibration/camera_intrinsics.json, wrist_cali_mtx.json, prob_cali_mtx.json: exact frame, per_session
                                         {session: matrix, source}; the top-level matrix only if all sessions agree
  meta/annotations.jsonl                 per episode: annotation.json + annotation.<annotator>.json merged (provenance, QA reviews, ratings)
  info.json splits: train / recovery / failure / test (contiguous, from stage_bags.py)
  meta/provenance.jsonl                  per episode: session, phantom, operator, type, mode, capture PC, commit, offsets
  README.md + MANIFEST.sha256           afterwards with export/release_card.py (dataset card from the data itself)
Then LeRobot's own v2.1 -> v3.0 converter, if this LeRobot version ships it (checked, not assumed),
and Open-H's validate_formatting.py (not in this repo: NOT VERIFIED).
"""
import argparse
import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import openh_fields as F  # noqa: E402


def read_jsonl(p):
    return [json.loads(x) for x in Path(p).read_text().splitlines() if x.strip()]


def write_jsonl(p, rows):
    Path(p).write_text(''.join(json.dumps(r) + '\n' for r in rows))


# per-episode provenance kept in meta/provenance.jsonl (the dataset card is built from it, no bags needed)
PROVENANCE_KEYS = ('episode_id', 'session_id', 'phantom_id', 'operator_id', 'type', 'stage', 'outcome',
                   'collection_mode', 'grip', 'failure_mode', 'staging_method', 'instruction', 'duration_s',
                   'capture_host', 'rig_git_commit', 'micrus_version', 'robot_tcp_setting', 'time_offsets_s',
                   'synthetic_pipeline_check')


def stats_of(arr):
    """NaN-aware (no-command steps are NaN); a column that is all NaN gets NaN stats."""
    import warnings
    a = np.asarray(arr, np.float64).reshape(len(arr), -1)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        return {'min': np.nanmin(a, 0).tolist(), 'max': np.nanmax(a, 0).tolist(), 'mean': np.nanmean(a, 0).tolist(),
                'std': np.nanstd(a, 0).tolist(), 'count': [len(a)]}


def read_streams(bag, us_topic, depth_topic, color_info_topic, depth_info_topic, want_depth):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    out = {'us': [], 'depth_st': [], 'depth': [], 'color_info': None, 'depth_info': None}
    topics = {us_topic, color_info_topic, depth_info_topic} | ({depth_topic} if want_depth else set())
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:
        for c, _, raw in r.messages(connections=[c for c in r.connections if c.topic in topics]):
            if c.topic in (color_info_topic, depth_info_topic):
                key = 'color_info' if c.topic == color_info_topic else 'depth_info'
                if out[key] is None:
                    out[key] = F.camera_info_dict(r.deserialize(raw, c.msgtype))
                continue
            m = r.deserialize(raw, c.msgtype)
            st = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            if c.topic == us_topic:
                out['us'].append(st)
            else:
                out['depth_st'].append(st)
                out['depth'].append(np.frombuffer(bytes(m.data), np.uint16)
                                    .reshape(m.height, m.step // 2)[:, :m.width].copy())
    return out


# Files the LeRobot v2.1 -> v3.0 converter does not keep (it rebuilds meta/): copied back afterwards.
EXTRA_META = ('calibration', 'annotations.jsonl', 'provenance.jsonl', 'modality.json', 'micrus_sources.jsonl')


def carry_meta(work, custom_info):
    """After conversion: the converter leaves the v2.1 copy in <work>_old. Copy the Open-H meta files
    and custom info.json keys into the v3.0 meta, check they arrived, then drop the v2.1 copy."""
    old = work.parent / (work.name + '_old')
    if not old.exists():
        sys.exit(f'{old} not found: the converter changed its layout; copy {EXTRA_META} into {work}/meta by hand')
    for name in EXTRA_META:
        s, d = old / 'meta' / name, work / 'meta' / name
        if s.is_dir():
            shutil.copytree(s, d, dirs_exist_ok=True)
        elif s.exists():
            shutil.copy2(s, d)
    if custom_info:
        p = work / 'meta' / 'info.json'
        info = json.loads(p.read_text())
        info.update(custom_info)
        p.write_text(json.dumps(info, indent=4))
    missing = [n for n in EXTRA_META if (old / 'meta' / n).exists() and not (work / 'meta' / n).exists()]
    if missing:
        sys.exit(f'meta files lost in conversion: {missing}')
    shutil.rmtree(old)


def session_id(bag):
    sj = Path(bag).parent / 'session.json'
    return (json.loads(sj.read_text()).get('session_id') if sj.exists() else None) or Path(bag).parent.name


def converter_config_sha(lr):
    """The hash bag_to_lerobot stores per episode (micrus_sources.jsonl config_sha256_16)."""
    import hashlib
    return hashlib.sha256(json.dumps(lr, sort_keys=True).encode()).hexdigest()[:16]


def feature_of(col, pa):
    typ = col.type
    if pa.types.is_string(typ):
        return {'dtype': 'string', 'shape': [1], 'names': None}
    if pa.types.is_boolean(typ):
        return {'dtype': 'bool', 'shape': [1], 'names': None}
    if pa.types.is_int64(typ):
        return {'dtype': 'int64', 'shape': [1], 'names': None}
    if pa.types.is_floating(typ):
        return {'dtype': 'float32', 'shape': [1], 'names': None}
    return {'dtype': 'float32', 'shape': [len(col[0].as_py())], 'names': None}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--v21', required=True)
    ap.add_argument('--order', required=True, help='<staging>_order.json from stage_bags.py')
    ap.add_argument('--out', required=True)
    ap.add_argument('--rig-config', default=str(HERE.parent / 'src/usrig_bringup/config/rig.yaml'))
    ap.add_argument('--session-config', default=str(HERE.parent / 'src/usrig_bringup/config/session.yaml'))
    ap.add_argument('--failure-catalog', default=str(HERE.parent / 'src/usrig_presets/config/failure_catalog.yaml'),
                    help='FM ids; a failure_class key on an entry overrides the PROPOSED table in annotation.py')
    ap.add_argument('--us-topic', default='/micrus/bmode')
    ap.add_argument('--converter-config', default=str(HERE / 'lerobot_rig.yaml'),
                    help='the bag_to_lerobot config the v2.1 dataset was converted with (checked by hash): its '
                         'time_offset_s values aligned the videos and are used for host stamps and depth too')
    ap.add_argument('--action-frame', choices=['base', 'tool'], default='base')
    ap.add_argument('--depth', choices=['png', 'none'], default='png')
    ap.add_argument('--skip-convert', action='store_true')
    ap.add_argument('--pipeline-check', action='store_true',
                    help='synthetic episodes only (pipeline_check.py); output name must start with SYNTHETIC_')
    a = ap.parse_args()

    import pyarrow as pa
    import pyarrow.parquet as pq
    import yaml

    rig = yaml.safe_load(Path(a.rig_config).read_text())
    scfg = yaml.safe_load(Path(a.session_config).read_text())
    cal = rig.get('calibration') or {}
    probe_type = (rig.get('probe') or {}).get('type')
    if probe_type is None:
        sys.exit('null in rig.yaml: probe.type (Open-H needs it)')
    # bag_to_lerobot aligned the videos with these: host stamps and depth use the same, and every episode's
    # measured offsets (episode.json, from its capture PC) must agree, else its frames would be off
    lr = yaml.safe_load(Path(a.converter_config).read_text())
    offsets, problems = F.converter_offsets(lr)
    ct = scfg['wrist_camera']['topics']

    src = Path(a.v21).expanduser()
    out = Path(a.out).expanduser()
    if a.pipeline_check and not out.name.startswith('SYNTHETIC_'):
        sys.exit('--pipeline-check output must be named SYNTHETIC_* (never confused with a real dataset)')
    work = out.parent / (out.name + '_v21_work')
    if out.exists() or work.exists():
        sys.exit(f'{out} or {work} exists')
    order = json.loads(Path(a.order).read_text())
    catalog = yaml.safe_load(Path(a.failure_catalog).read_text()) if Path(a.failure_catalog).exists() else None
    # checked before writing anything: dataset = staged episodes in staging order, converter config, offsets,
    # and a failure class on every failure / recovery episode
    src_rows = read_jsonl(src / 'meta' / 'micrus_sources.jsonl')
    problems += F.index_problems(order['episodes'], src_rows,
                                 [int(p.stem.split('_')[-1]) for p in src.glob('data/chunk-*/episode_*.parquet')])
    sha = converter_config_sha(lr)
    problems += [f'v2.1 episode {r["episode_index"]} was converted with another config ({r.get("config_sha256_16")}, '
                 f'{a.converter_config} is {sha}): its video offsets are unknown'
                 for r in src_rows if r.get('config_sha256_16') not in (None, sha)]
    staged, calib = [], {}           # calib: session id -> {exported name: (matrix, source)}
    for o in order['episodes']:
        epj = json.loads((Path(o['bag']) / 'episode.json').read_text())
        staged.append((epj, F.A.load_merged(o['bag'], episode=epj)))
        problems += F.offset_problems(epj, offsets)
        sid = session_id(o['bag'])
        if sid not in calib:
            calib[sid] = F.session_calibration(json.loads((Path(o['bag']).parent / 'session.json').read_text()), cal)
            if calib[sid]['prob_cali_mtx'][0] is None:
                problems.append(f'session {sid}: prob_cali_mtx neither in session.json nor in {a.rig_config} '
                                '(Open-H needs it)')
    problems += F.failure_class_problems(staged, catalog)
    problems += [f'{epj.get("episode_id")}: episode.json has no t_start_ns / t_end_ns (episode window unknown)'
                 for epj, _ in staged if epj.get('t_start_ns') is None or epj.get('t_end_ns') is None]
    if problems:
        hint = ('\nOffsets: set lerobot_rig.yaml time_offset_s to the capture PC values (stage and convert episodes '
                'from different capture PCs separately) and convert again.' if any('offset' in p for p in problems) else '')
        sys.exit(f'not exported ({len(problems)} problems; nothing written):\n  ' + '\n  '.join(problems) + hint)
    shutil.copytree(src, work)
    meta = work / 'meta'
    info = json.loads((meta / 'info.json').read_text())
    sources = {s['episode_index']: s for s in read_jsonl(meta / 'micrus_sources.jsonl')}
    by_bag = {str(Path(e['bag']).resolve()): e for e in order['episodes']}
    ep_stats = {e['episode_index']: e for e in read_jsonl(meta / 'episodes_stats.jsonl')}
    state_names = (info['features'].get('observation.state') or {}).get('names') or []
    tcp_idx = ([state_names.index(n) for n in ('tcp_x', 'tcp_y', 'tcp_z', 'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw')]
               if 'tcp_x' in state_names else list(range(6, 13)))

    new_features, annotations, intrinsics = {}, [], {}
    provenance = []
    for pf in sorted(work.glob('data/chunk-*/episode_*.parquet')):
        t = pq.read_table(pf)
        ep_index = int(t.column('episode_index')[0].as_py())
        s = sources[ep_index]
        bag = str(Path(s['bag']).resolve())
        o = by_bag.get(bag)
        if o is None or o['index'] != ep_index:
            sys.exit(f'episode {ep_index}: bag {bag} is not staged at that index in {a.order}')
        epj = json.loads((Path(bag) / 'episode.json').read_text())
        us_off_ep, d_off_ep = offsets['ultrasound'], offsets['camera']   # = the measured ones (checked above)
        if epj.get('sim'):
            sys.exit(f'{bag}: simulated episode (sim: true): never exported')
        if bool(epj.get('synthetic_pipeline_check')) != a.pipeline_check:
            sys.exit(f'{bag}: synthetic pipeline-check episodes only with --pipeline-check, and never mixed '
                     'with real episodes')
        sess = json.loads((Path(bag).parent / 'session.json').read_text())
        # annotation.json (automated) + every annotation.<annotator>.json, merged
        ann = F.A.load_merged(bag, episode=epj)
        n = t.num_rows
        t_steps = ((float(s['start_unix']) + np.arange(n) / float(s['fps'])) * 1e9).astype(np.int64)
        st = read_streams(bag, a.us_topic, ct['depth'], ct['color_info'], ct['depth_info'], a.depth == 'png')
        sid = session_id(bag)
        intrinsics.setdefault(sid, {'wrist_camera': st['color_info'], 'wrist_depth': st['depth_info']})

        us = np.sort(np.asarray(st['us'], np.int64))
        hi = F.sample_hold(us + int(us_off_ep * 1e9), t_steps)
        host = np.where(hi >= 0, us[np.clip(hi, 0, None)], -1)
        status, phase, gesture = F.step_labels(ann, t_steps)
        label_src = F.step_label_sources(ann, t_steps)
        # this session's calibration snapshot (session.json), the exporting PC's rig.yaml only as a fallback
        prob_cali, cam = calib[sid]['prob_cali_mtx'][0], calib[sid]['wrist_cali_mtx'][0]
        state = np.asarray(t.column('observation.state').to_pylist(), float)
        old_action = np.asarray(t.column('action').to_pylist(), float)
        # no command existed on kinesthetic steps (freedrive by hand) or without robot_control (NaN in the
        # bag): flagged by command_valid = false and stored as 0 (LeRobot v3.0 turns NaN into null)
        src = (np.asarray(t.column('observation.meta.action_source').to_pylist(), float).ravel()
               if 'observation.meta.action_source' in t.column_names else np.full(len(old_action), np.nan))
        command_valid = ~np.isnan(old_action).any(axis=1) & (src != 5)
        old_action[~command_valid, :] = 0.0
        act = F.delta_actions(state[:, tcp_idx], a.action_frame)
        # the converter keeps the whole bag: flag pre-roll / post-roll, operator pauses and the zero last action
        in_episode, paused, action_valid = F.step_flags(t_steps, epj['t_start_ns'], epj['t_end_ns'],
                                                        epj.get('pauses_ns'))
        fc, fc_src = F.failure_class(epj, ann, catalog)
        cols = {
            'action': pa.array(act.astype(np.float32).tolist(), pa.list_(pa.float32(), 6)),
            'observation.meta.commanded_twist': pa.array(old_action[:, :6].astype(np.float32).tolist(),
                                                         pa.list_(pa.float32(), 6)),
            # shape [1] numeric features are scalars in LeRobot's HF schema (a 1-element list fails to load)
            'observation.meta.force_setpoint_n': pa.array(old_action[:, 6].astype(np.float32), pa.float32()),
            'observation.meta.command_valid': pa.array(command_valid, pa.bool_()),
            'observation.meta.in_episode': pa.array(in_episode, pa.bool_()),
            'observation.meta.paused': pa.array(paused, pa.bool_()),
            'observation.meta.action_valid': pa.array(action_valid, pa.bool_()),
            'observation.meta.force_torque': t.column('observation.force_torque')
            if 'observation.force_torque' in t.column_names else pa.array([[np.nan] * 6] * n),
            'observation.meta.probe_type': pa.array([probe_type] * n),
            'observation.meta.probe_acquisition_param': t.column('observation.ultrasound.settings'),
            'observation.meta.prob_cali_mtx': pa.array([list(np.ravel(prob_cali).astype(float))] * n,
                                                       pa.list_(pa.float32(), 16)),
            'observation.meta.wrist_cali_mtx': pa.array(
                [list(np.ravel(cam).astype(float)) if cam is not None else [np.nan] * 16] * n,
                pa.list_(pa.float32(), 16)),
            'observation.meta.host_stamp_ns': pa.array(host, pa.int64()),
            'observation.meta.status': pa.array(status),
            'observation.meta.phase': pa.array(phase),
            'observation.meta.gesture': pa.array(gesture),
            'observation.meta.status_source': pa.array(label_src['status']),
            'observation.meta.phase_source': pa.array(label_src['phase']),
            'observation.meta.gesture_source': pa.array(label_src['gesture']),
            'observation.meta.action_is_expert': pa.array(
                np.asarray(t.column('observation.meta.action_is_expert').to_pylist()).ravel() > 0.5),
            'observation.meta.stage': pa.array([int(epj.get('stage', -1))] * n, pa.int64()),
            'observation.meta.failure_mode': pa.array([epj.get('failure_mode') or ''] * n),
            'observation.meta.failure_class': pa.array([fc] * n),
            'instruction.text': pa.array([epj.get('instruction') or s.get('task') or ''] * n),
        }
        if a.depth == 'png':
            if not st['depth']:
                sys.exit(f'{bag}: no depth frames on {ct["depth"]}')
            di = F.sample_hold(np.asarray(st['depth_st'], np.int64) + int(d_off_ep * 1e9), t_steps)
            if np.any(di < 0):
                sys.exit(f'{bag}: depth starts after the first step')
            cols['observation.images.wrist_depth'] = pa.array(
                [{'bytes': F.png16(st['depth'][i]), 'path': None} for i in di],
                pa.struct([('bytes', pa.binary()), ('path', pa.string())]))
        for k, v in cols.items():
            if k in t.column_names:
                t = t.drop([k])
            t = t.append_column(k, v)
        pq.write_table(t, pf)

        es = ep_stats[ep_index]['stats']
        for k in ('action', 'observation.meta.commanded_twist', 'observation.meta.force_setpoint_n',
                  'observation.meta.host_stamp_ns', 'observation.meta.stage', 'observation.meta.prob_cali_mtx',
                  'observation.meta.wrist_cali_mtx', 'observation.meta.force_torque',
                  'observation.meta.probe_acquisition_param', 'observation.meta.action_is_expert',
                  'observation.meta.command_valid', 'observation.meta.in_episode', 'observation.meta.paused',
                  'observation.meta.action_valid'):
            es[k] = stats_of(np.asarray(t.column(k).to_pylist(), np.float64))
        if a.depth == 'png':
            d = np.stack([st['depth'][i] for i in di]).astype(np.float64)
            es['observation.images.wrist_depth'] = {k: [[[float(v)]]] for k, v in
                                                    (('min', d.min()), ('max', d.max()), ('mean', d.mean()),
                                                     ('std', d.std()))} | {'count': [n]}
        if not new_features:
            for k in cols:
                if k == 'observation.images.wrist_depth':
                    h, w = st['depth'][0].shape
                    new_features[k] = {'dtype': 'image', 'shape': [h, w, 1],
                                       'names': ['height', 'width', 'channels'],
                                       'info': {'encoding': 'png16', 'units': 'mm'}}
                elif k == 'action':
                    new_features[k] = {'dtype': 'float32', 'shape': [6], 'names': F.ACTION_NAMES,
                                       'info': {'frame': a.action_frame, 'rotation': 'rotation_vector'}}
                else:
                    new_features[k] = feature_of(t.column(k), pa)
        annotations.append({'episode_index': ep_index, 'episode_id': epj.get('episode_id'), **F.A.public(ann)})
        provenance.append({'episode_index': ep_index, 'split': o['split'],
                           **{k: epj.get(k) for k in PROVENANCE_KEYS},
                           'session_sim': bool(sess.get('sim')), 'n_steps': n,
                           'n_steps_episode': int((in_episode & ~paused).sum()),      # what the card counts as hours
                           'failure_class': fc or None, 'failure_class_source': fc_src or None,
                           'time_offsets_used_s': offsets,
                           'calibration_sources': {k: v[1] for k, v in calib[sid].items()}})
        print(f'episode {ep_index}: {n} steps ({int(in_episode.sum())} in the episode, {int(paused.sum())} paused, '
              f'{int(action_valid.sum())} valid actions), split {o["split"]}, type {o["type"]}, '
              f'status {dict(zip(*np.unique(status, return_counts=True)))}')

    # wrist_rgb is named wrist_camera in the proposal (lerobot_rig.yaml already exports it so)
    info['features'].update(new_features)
    if a.pipeline_check:
        info['pipeline_check'] = 'SYNTHETIC data from export/pipeline_check.py: not a real dataset'
    write_jsonl(meta / 'episodes_stats.jsonl', [ep_stats[k] for k in sorted(ep_stats)])
    write_jsonl(meta / 'annotations.jsonl', annotations)
    write_jsonl(meta / 'provenance.jsonl', provenance)
    first = intrinsics[next(iter(intrinsics))] if intrinsics else {}
    F.write_json(meta / 'calibration' / 'camera_intrinsics.json',
                 {**first, 'depth_units': 'mm', 'depth_aligned_to': 'wrist_camera',
                  'same_in_all_sessions': len({json.dumps(v, sort_keys=True) for v in intrinsics.values()}) <= 1,
                  'per_session': intrinsics})
    used = {k: calib[k] for k in intrinsics}       # sessions that have episodes in the dataset
    for name in F.CALIBRATIONS:
        F.write_json(meta / 'calibration' / f'{name}.json', F.calibration_meta(used, name))

    groups = {}
    for o in order['episodes']:
        name = 'test' if o['split'] == 'test' else (
            'recovery' if o['type'] in ('recovery-start', 'recovery-mid') else
            'failure' if o['type'] == 'failure' else 'train')
        groups.setdefault(name, []).append(o['index'])
    splits = {}
    for name, idx in groups.items():
        lo, hi = min(idx), max(idx) + 1
        if hi - lo != len(idx):
            sys.exit(f'split {name} is not contiguous (re-run stage_bags.py)')
        splits[name] = (lo, hi)
    info['splits'] = {k: f'{lo}:{hi}' for k, (lo, hi) in splits.items()}
    (meta / 'info.json').write_text(json.dumps(info, indent=4))

    if a.skip_convert:
        print(f'v2.1 with Open-H fields in {work}; conversion skipped')
        return
    # LeRobot 0.6.0: lerobot.scripts.convert_dataset_v21_to_v30 (needs lerobot[dataset]); earlier
    # releases used lerobot.datasets.v30.*. Checked at run time, never assumed.
    mod = None
    for cand in ('lerobot.scripts.convert_dataset_v21_to_v30', 'lerobot.datasets.v30.convert_dataset_v21_to_v30'):
        try:
            if importlib.util.find_spec(cand) is not None:
                mod = cand
                break
        except ModuleNotFoundError:
            continue
    if mod is None:
        sys.exit('no official v2.1 -> v3.0 converter found in this LeRobot install (pip install "lerobot[dataset]"). '
                 f'The v2.1 dataset with Open-H fields is in {work}')
    h = subprocess.run([sys.executable, '-m', mod, '--help'], capture_output=True, text=True)
    print(h.stdout[-2000:])
    cmd = [sys.executable, '-m', mod, f'--repo-id=local/{out.name}', f'--root={work}', '--push-to-hub=false']
    print('running:', ' '.join(cmd))
    custom_info = {k: info[k] for k in ('pipeline_check',) if k in info}
    r = subprocess.run(cmd)
    if r.returncode:
        sys.exit(f'converter failed ({r.returncode}); check its --help above for this version')
    carry_meta(work, custom_info)
    shutil.move(str(work), str(out))
    print(f'wrote {out}. Next: python validate_formatting.py {out}  (Open-H)')


if __name__ == '__main__':
    main()
