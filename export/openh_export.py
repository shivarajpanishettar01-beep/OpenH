#!/usr/bin/env python3
"""Open-H v2 export: LeRobot v2.1 (from bag_to_lerobot) -> Open-H fields -> LeRobot v3.0.

Run in the separate Python 3.12 env with LeRobot 0.6.0 (conda env `openh-export`):

  python openh_export.py --v21 ~/datasets/us_v21 --order ~/datasets/staging_order.json \
      --out ~/datasets/us_openh_v30 [--rig-config ../src/usrig_bringup/config/rig.yaml]

Steps
  1. copy the v2.1 dataset (the original stays untouched for GR00T)
  2. add per-frame Open-H fields to every episode parquet and to meta/info.json:
       observation.meta.force_torque         float32[6]   (from observation.force_torque)
       observation.meta.probe_type           string
       observation.meta.probe_acquisition_param float32[N] (= observation.ultrasound.settings)
       observation.meta.prob_cali_mtx        float32[16]
       observation.meta.host_stamp_ns        int64        real header stamp of the ultrasound
                                                          frame used at each step (from the bag)
       observation.meta.action_is_expert     bool
       observation.meta.stage                int64
       observation.meta.failure_mode         string
       observation.meta.wrist_cam_cali_mtx   float32[16]
       instruction.text                      string
  3. write the train / recovery / failure / test episode ranges into info.json["splits"]
     (episodes were ordered by stage_bags.py so the ranges are contiguous)
  4. convert with LeRobot's own v2.1 -> v3.0 converter, if this LeRobot version ships it
     (checked at run time, never assumed)
  5. copy the Open-H README template into meta/
Then run Open-H's validate_formatting.py on --out (not shipped here: NOT VERIFIED).
Field names follow the master prompt; check them against the Open-H guide before release.
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


def read_jsonl(p):
    return [json.loads(x) for x in Path(p).read_text().splitlines() if x.strip()]


def write_jsonl(p, rows):
    Path(p).write_text(''.join(json.dumps(r) + '\n' for r in rows))


def frame_stamps(bag, topic, offset_s, t_steps_ns):
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore
    st = []
    with AnyReader([Path(bag)], default_typestore=get_typestore(Stores.ROS2_HUMBLE)) as r:
        conns = [c for c in r.connections if c.topic == topic]
        for c, _, raw in r.messages(connections=conns):
            m = r.deserialize(raw, c.msgtype)
            st.append(m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec)
    st = np.sort(np.asarray(st, np.int64))
    idx = np.searchsorted(st + int(offset_s * 1e9), t_steps_ns, side='right') - 1
    return np.where(idx >= 0, st[np.clip(idx, 0, None)], -1)


def stats_of(arr):
    a = np.asarray(arr, np.float64).reshape(len(arr), -1)
    return {'min': a.min(0).tolist(), 'max': a.max(0).tolist(), 'mean': a.mean(0).tolist(),
            'std': a.std(0).tolist(), 'count': [len(a)]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--v21', required=True)
    ap.add_argument('--order', required=True, help='<staging>_order.json from stage_bags.py')
    ap.add_argument('--out', required=True)
    ap.add_argument('--rig-config', default=str(HERE.parent / 'src/usrig_bringup/config/rig.yaml'))
    ap.add_argument('--us-topic', default='/micrus/bmode')
    ap.add_argument('--us-time-offset-s', type=float, default=-0.020, help='PLACEHOLDER until P9c')
    ap.add_argument('--skip-convert', action='store_true')
    a = ap.parse_args()

    import pyarrow as pa
    import pyarrow.parquet as pq
    import yaml

    rig = yaml.safe_load(Path(a.rig_config).read_text())
    probe_type = (rig.get('probe') or {}).get('type')
    prob_cali = (rig.get('calibration') or {}).get('prob_cali_mtx')
    missing = [k for k, v in (('probe.type', probe_type), ('calibration.prob_cali_mtx', prob_cali)) if v is None]
    if missing:
        sys.exit(f'null in rig.yaml: {missing} (Open-H needs them)')

    src = Path(a.v21).expanduser()
    out = Path(a.out).expanduser()
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

    new_features = {}
    for pf in sorted(work.glob('data/chunk-*/episode_*.parquet')):
        t = pq.read_table(pf)
        ep_index = int(t.column('episode_index')[0].as_py())
        s = sources[ep_index]
        bag = str(Path(s['bag']).resolve())
        o = by_bag.get(bag)
        if o is None:
            sys.exit(f'episode {ep_index}: bag {bag} not in {a.order}')
        epj = json.loads((Path(bag) / 'episode.json').read_text())
        sess = json.loads((Path(bag).parent / 'session.json').read_text())
        n = t.num_rows
        t_steps = (float(s['start_unix']) + np.arange(n) / float(s['fps'])) * 1e9
        host = frame_stamps(bag, a.us_topic, a.us_time_offset_s, t_steps.astype(np.int64))
        cam = sess.get('wrist_cam_cali_mtx') or (rig.get('calibration') or {}).get('wrist_cam_cali_mtx')
        cols = {
            'observation.meta.force_torque': t.column('observation.force_torque')
            if 'observation.force_torque' in t.column_names else pa.array([[np.nan] * 6] * n),
            'observation.meta.probe_type': pa.array([probe_type] * n),
            'observation.meta.probe_acquisition_param': t.column('observation.ultrasound.settings'),
            'observation.meta.prob_cali_mtx': pa.array([list(np.ravel(prob_cali).astype(float))] * n,
                                                       pa.list_(pa.float32(), 16)),
            'observation.meta.host_stamp_ns': pa.array(host, pa.int64()),
            'observation.meta.action_is_expert': pa.array(
                np.asarray(t.column('observation.meta.action_is_expert').to_pylist()).ravel() > 0.5),
            'observation.meta.stage': pa.array([int(epj.get('stage', -1))] * n, pa.int64()),
            'observation.meta.failure_mode': pa.array([epj.get('failure_mode') or ''] * n),
            'observation.meta.wrist_cam_cali_mtx': pa.array(
                [list(np.ravel(cam).astype(float)) if cam is not None else [np.nan] * 16] * n,
                pa.list_(pa.float32(), 16)),
            'instruction.text': pa.array([epj.get('instruction') or s.get('task') or ''] * n),
        }
        for k, v in cols.items():
            if k in t.column_names:
                t = t.drop([k])
            t = t.append_column(k, v)
        pq.write_table(t, pf)
        st = ep_stats[ep_index]['stats']
        for k in ('observation.meta.host_stamp_ns', 'observation.meta.stage', 'observation.meta.prob_cali_mtx',
                  'observation.meta.wrist_cam_cali_mtx', 'observation.meta.force_torque',
                  'observation.meta.probe_acquisition_param'):
            st[k] = stats_of(np.asarray(t.column(k).to_pylist(), np.float64))
        st['observation.meta.action_is_expert'] = stats_of(
            np.asarray(t.column('observation.meta.action_is_expert').to_pylist(), np.float64))
        if not new_features:
            for k in cols:
                col = t.column(k)
                typ = col.type
                if pa.types.is_string(typ):
                    new_features[k] = {'dtype': 'string', 'shape': [1], 'names': None}
                elif pa.types.is_boolean(typ):
                    new_features[k] = {'dtype': 'bool', 'shape': [1], 'names': None}
                elif pa.types.is_int64(typ):
                    new_features[k] = {'dtype': 'int64', 'shape': [1], 'names': None}
                else:
                    shape = len(col[0].as_py())
                    new_features[k] = {'dtype': 'float32', 'shape': [shape], 'names': None}
        print(f'episode {ep_index}: {n} steps, split {o["split"]}, type {o["type"]}')
    info['features'].update(new_features)
    write_jsonl(meta / 'episodes_stats.jsonl', [ep_stats[k] for k in sorted(ep_stats)])

    # splits: LeRobot v2.1 uses "start:end" strings over episode indices
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
    tmpl = HERE / 'openh_README_template.md'
    if tmpl.exists():
        shutil.copy(tmpl, meta / 'README.md')

    if a.skip_convert:
        print(f'v2.1 with Open-H fields in {work}; conversion skipped')
        return
    mod = 'lerobot.datasets.v30.convert_dataset_v21_to_v30'
    if importlib.util.find_spec(mod) is None:
        sys.exit(f'{mod} not found in this LeRobot version: no official v2.1 -> v3.0 converter. '
                 f'The v2.1 dataset with Open-H fields is in {work}')
    h = subprocess.run([sys.executable, '-m', mod, '--help'], capture_output=True, text=True)
    print(h.stdout[-2000:])
    cmd = [sys.executable, '-m', mod, f'--repo-id=local/{out.name}', f'--root={work}', '--push-to-hub=false']
    print('running:', ' '.join(cmd))
    r = subprocess.run(cmd)
    if r.returncode:
        sys.exit(f'converter failed ({r.returncode}); check its --help above for this version')
    shutil.move(str(work), str(out))
    print(f'wrote {out}. Next: python validate_formatting.py {out}  (Open-H)')


if __name__ == '__main__':
    main()
