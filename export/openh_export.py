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
  observation.images.wrist_depth         image       16-bit PNG, millimetres, aligned to colour (lossless)
  observation.meta.force_torque          float32[6]
  observation.meta.probe_type            string
  observation.meta.probe_acquisition_param float32[N] (= observation.ultrasound.settings)
  observation.meta.prob_cali_mtx         float32[16]
  observation.meta.wrist_cali_mtx        float32[16] wrist camera w.r.t. the end-effector (hand-eye)
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
  observation.meta.failure_class         string      one of the four proposal failure classes
  instruction.text                       string
Meta:
  meta/calibration/camera_intrinsics.json, wrist_cali_mtx.json, prob_cali_mtx.json
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


_warned = set()


def warn_offsets(epj, ep_off):
    """bag_to_lerobot aligned the images with lerobot_rig.yaml time_offset_s; warn once per value if that differs
    from what the recording PC measured (episode.json time_offsets_s)."""
    import yaml
    lr = yaml.safe_load((HERE / 'lerobot_rig.yaml').read_text())
    pairs = (('ultrasound', lr['images']['observation.images.ultrasound'].get('time_offset_s')),
             ('camera', lr['images']['observation.images.wrist_camera'].get('time_offset_s')))
    for name, used in pairs:
        meas = ep_off.get(name)
        if meas is not None and used is not None and abs(float(meas) - float(used)) > 0.001 and name not in _warned:
            _warned.add(name)
            print(f'WARNING: {epj.get("episode_id")}: lerobot_rig.yaml {name} time_offset_s {used} differs from the '
                  f'{meas} measured on {epj.get("capture_host")}: set lerobot_rig.yaml to the capture PC value and '
                  'convert again (bag_to_lerobot aligned the images with it)')


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
    ap.add_argument('--us-topic', default='/micrus/bmode')
    ap.add_argument('--us-time-offset-s', type=float, default=None,
                    help='default: rig.yaml calibration.ultrasound_time_offset_s (PLACEHOLDER until P9c)')
    ap.add_argument('--depth-time-offset-s', type=float, default=None,
                    help='default: rig.yaml calibration.camera_time_offset_s (PLACEHOLDER until P9c)')
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
    prob_cali = cal.get('prob_cali_mtx')
    missing = [k for k, v in (('probe.type', probe_type), ('calibration.prob_cali_mtx', prob_cali)) if v is None]
    if missing:
        sys.exit(f'null in rig.yaml: {missing} (Open-H needs them)')
    us_off = a.us_time_offset_s if a.us_time_offset_s is not None else float(cal.get('ultrasound_time_offset_s', 0))
    d_off = a.depth_time_offset_s if a.depth_time_offset_s is not None else float(cal.get('camera_time_offset_s', 0))
    ct = scfg['wrist_camera']['topics']

    src = Path(a.v21).expanduser()
    out = Path(a.out).expanduser()
    if a.pipeline_check and not out.name.startswith('SYNTHETIC_'):
        sys.exit('--pipeline-check output must be named SYNTHETIC_* (never confused with a real dataset)')
    work = out.parent / (out.name + '_v21_work')
    if out.exists() or work.exists():
        sys.exit(f'{out} or {work} exists')
    shutil.copytree(src, work)
    meta = work / 'meta'
    info = json.loads((meta / 'info.json').read_text())
    sources = {s['episode_index']: s for s in read_jsonl(meta / 'micrus_sources.jsonl')}
    order = json.loads(Path(a.order).read_text())
    by_bag = {str(Path(e['bag']).resolve()): e for e in order['episodes']}
    ep_stats = {e['episode_index']: e for e in read_jsonl(meta / 'episodes_stats.jsonl')}
    state_names = (info['features'].get('observation.state') or {}).get('names') or []
    tcp_idx = ([state_names.index(n) for n in ('tcp_x', 'tcp_y', 'tcp_z', 'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw')]
               if 'tcp_x' in state_names else list(range(6, 13)))

    new_features, annotations, intrinsics, wrist_cali = {}, [], None, None
    provenance = []
    for pf in sorted(work.glob('data/chunk-*/episode_*.parquet')):
        t = pq.read_table(pf)
        ep_index = int(t.column('episode_index')[0].as_py())
        s = sources[ep_index]
        bag = str(Path(s['bag']).resolve())
        o = by_bag.get(bag)
        if o is None:
            sys.exit(f'episode {ep_index}: bag {bag} not in {a.order}')
        epj = json.loads((Path(bag) / 'episode.json').read_text())
        # offsets measured on the PC that recorded this episode (episode.json) win over the exporting PC's rig.yaml
        ep_off = epj.get('time_offsets_s') or {}
        us_off_ep = us_off if a.us_time_offset_s is not None or ep_off.get('ultrasound') is None else float(ep_off['ultrasound'])
        d_off_ep = d_off if a.depth_time_offset_s is not None or ep_off.get('camera') is None else float(ep_off['camera'])
        warn_offsets(epj, ep_off)
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
        intrinsics = intrinsics or {'wrist_camera': st['color_info'], 'wrist_depth': st['depth_info'],
                                    'depth_units': 'mm', 'depth_aligned_to': 'wrist_camera'}

        us = np.sort(np.asarray(st['us'], np.int64))
        hi = F.sample_hold(us + int(us_off_ep * 1e9), t_steps)
        host = np.where(hi >= 0, us[np.clip(hi, 0, None)], -1)
        status, phase, gesture = F.step_labels(ann, t_steps)
        label_src = F.step_label_sources(ann, t_steps)
        cam = sess.get('wrist_cam_cali_mtx') or cal.get('wrist_cam_cali_mtx')
        wrist_cali = wrist_cali or cam
        state = np.asarray(t.column('observation.state').to_pylist(), float)
        old_action = np.asarray(t.column('action').to_pylist(), float)
        # no command existed on kinesthetic steps (freedrive by hand) or without robot_control (NaN in the
        # bag): flagged by command_valid = false and stored as 0 (LeRobot v3.0 turns NaN into null)
        src = (np.asarray(t.column('observation.meta.action_source').to_pylist(), float).ravel()
               if 'observation.meta.action_source' in t.column_names else np.full(len(old_action), np.nan))
        command_valid = ~np.isnan(old_action).any(axis=1) & (src != 5)
        old_action[~command_valid, :] = 0.0
        act = F.delta_actions(state[:, tcp_idx], a.action_frame)
        fc = (ann.get('failure_class') or {}).get('value') or ''
        cols = {
            'action': pa.array(act.astype(np.float32).tolist(), pa.list_(pa.float32(), 6)),
            'observation.meta.commanded_twist': pa.array(old_action[:, :6].astype(np.float32).tolist(),
                                                         pa.list_(pa.float32(), 6)),
            # shape [1] numeric features are scalars in LeRobot's HF schema (a 1-element list fails to load)
            'observation.meta.force_setpoint_n': pa.array(old_action[:, 6].astype(np.float32), pa.float32()),
            'observation.meta.command_valid': pa.array(command_valid, pa.bool_()),
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
                  'observation.meta.command_valid'):
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
                           'session_sim': bool(sess.get('sim')), 'n_steps': n})
        print(f'episode {ep_index}: {n} steps, split {o["split"]}, type {o["type"]}, '
              f'status {dict(zip(*np.unique(status, return_counts=True)))}')

    # wrist_rgb is named wrist_camera in the proposal (lerobot_rig.yaml already exports it so)
    info['features'].update(new_features)
    if a.pipeline_check:
        info['pipeline_check'] = 'SYNTHETIC data from export/pipeline_check.py: not a real dataset'
    write_jsonl(meta / 'episodes_stats.jsonl', [ep_stats[k] for k in sorted(ep_stats)])
    write_jsonl(meta / 'annotations.jsonl', annotations)
    write_jsonl(meta / 'provenance.jsonl', provenance)
    F.write_json(meta / 'calibration' / 'camera_intrinsics.json', intrinsics)
    F.write_json(meta / 'calibration' / 'prob_cali_mtx.json',
                 {'prob_cali_mtx': prob_cali, 'frame': 'probe w.r.t. end-effector'})
    F.write_json(meta / 'calibration' / 'wrist_cali_mtx.json',
                 {'wrist_cali_mtx': wrist_cali, 'frame': 'wrist camera w.r.t. end-effector (eye-in-hand)'})

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
