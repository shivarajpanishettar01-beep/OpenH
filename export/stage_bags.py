#!/usr/bin/env python3
"""Select and order episode bags for conversion (bag_to_lerobot takes a folder of bags in name
order). Writes a staging folder of symlinks:

  python stage_bags.py --sessions ~/data/sessions/S1 ~/data/sessions/S2 --test-sessions ~/data/sessions/S3 -o ~/datasets/staging

* Simulated sessions / episodes (sim: true) are refused outright.
* Synthetic pipeline-check episodes (synthetic_pipeline_check: true, written by pipeline_check.py)
  are refused unless --pipeline-check, which in turn refuses every episode that is not synthetic.
* Stage 0 episodes are excluded unless --include-stage0; Stage 1 unless --include-stage1
  (open decision 6); discards and episodes whose QC failed are always excluded. The rule is
  usrig_common.export_rules.exportable, shared with collection_progress / collection_plan.
* Freedrive (kinesthetic) episodes are expert demonstrations: included whatever their stage, in the
  train split, unless --exclude-kinesthetic.
* Every bag must have messages on every topic bag_to_lerobot reads (--converter-config) and, with
  --depth png, the wrist depth topic: a bag without one is skipped and reported (bag_to_lerobot would
  otherwise stop the whole conversion at it).
* Order inside train / test: success, contact-establishment, recovery-start, recovery-mid,
  failure, so the recovery and failure ranges in info.json are contiguous.
* Test = whole held-out sessions (never episodes from sessions also used for training).
Prints the episode ranges per split and type; openh_export.py reads <out>_order.json.
"""
import argparse
import json
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'src' / 'usrig_common'))
from usrig_common.export_rules import exportable  # noqa: E402

TYPE_ORDER = ['success', 'kinesthetic', 'shakedown', 'scripted', 'contact-establishment', 'recovery-start',
              'recovery-mid', 'failure']


def check_synthetic(ep, pipeline_check, where):
    synthetic = bool(ep.get('synthetic_pipeline_check'))
    if synthetic and not pipeline_check:
        raise SystemExit(f'{where}: synthetic pipeline-check episode: only with --pipeline-check')
    if pipeline_check and not synthetic:
        raise SystemExit(f'{where}: real episode in a --pipeline-check run: refused (never mixed)')


def converter_topics(cfg_path):
    """Every topic bag_to_lerobot reads with this config: it stops the whole conversion if one is missing."""
    lr = yaml.safe_load(Path(cfg_path).read_text())
    t = {d['topic'] for d in (lr.get('images') or {}).values()}
    for key in ('state', 'action'):
        t |= {s['topic'] for s in lr.get(key) or []}
    for srcs in (lr.get('extra') or {}).values():
        t |= {s['topic'] for s in srcs}
    if lr.get('micrus_frame_info'):
        t.add(lr['micrus_frame_info'].get('topic', '/micrus/frame_info'))
    return t


def missing_topics(bag_dir, required):
    """Required topics without messages in the bag (from its metadata.yaml), or ['metadata.yaml'] without one."""
    m = Path(bag_dir) / 'metadata.yaml'
    if not m.exists():
        return ['metadata.yaml']
    info = (yaml.safe_load(m.read_text()) or {}).get('rosbag2_bagfile_information') or {}
    counts = {x['topic_metadata']['name']: x.get('message_count', 0) for x in info.get('topics_with_message_count') or []}
    return sorted(t for t in required if not counts.get(t))


def collect(sessions, a, required=None, skipped=None):
    """Exportable episodes, sorted. required: topics every bag must have; bags without them are left out and
    appended to `skipped` (with the missing topics) instead of stopping the conversion."""
    eps = []
    for s in sessions:
        sj = Path(s).expanduser() / 'session.json'
        if sj.exists() and json.loads(sj.read_text()).get('sim'):
            raise SystemExit(f'{s} is a simulated session (sim: true): never exported')
        for f in sorted(Path(s).expanduser().glob('ep_*/episode.json')):
            e = json.loads(f.read_text())
            if e.get('sim'):
                raise SystemExit(f'{f}: simulated episode (sim: true): never exported')
            check_synthetic(e, a.pipeline_check, f)
            # the same rule collection_progress / collection_plan count hours with (freedrive kept at any stage)
            if not exportable(e, a.include_stage0, a.include_stage1, not a.exclude_kinesthetic, a.pipeline_check)[0]:
                continue
            miss = missing_topics(f.parent, required) if required else []
            if miss:
                if skipped is not None:
                    skipped.append({'episode_id': e.get('episode_id'), 'bag': str(f.parent), 'missing': miss})
                continue
            e['_bag'] = str(f.parent.resolve())
            eps.append(e)
    key = lambda e: (TYPE_ORDER.index(e.get('type')) if e.get('type') in TYPE_ORDER else 99,  # noqa: E731
                     e['session_id'], e['episode_id'])
    return sorted(eps, key=key)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--sessions', nargs='+', required=True)
    ap.add_argument('--test-sessions', nargs='*', default=[])
    ap.add_argument('--include-stage0', action='store_true')
    ap.add_argument('--include-stage1', action='store_true')
    ap.add_argument('--exclude-kinesthetic', action='store_true', help='leave freedrive episodes out')
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--pipeline-check', action='store_true', help='synthetic episodes only (pipeline_check.py)')
    ap.add_argument('--converter-config', default=str(HERE / 'lerobot_rig.yaml'),
                    help='bag_to_lerobot config: every bag must have its topics (else skipped and reported)')
    ap.add_argument('--depth', choices=['png', 'none'], default='png',
                    help='png (as openh_export): the wrist depth topic is required too')
    ap.add_argument('--session-config', default=str(HERE.parent / 'src/usrig_bringup/config/session.yaml'))
    a = ap.parse_args()
    required = converter_topics(a.converter_config)
    if a.depth == 'png':
        required.add(yaml.safe_load(Path(a.session_config).read_text())['wrist_camera']['topics']['depth'])
    skipped = []
    overlap = {Path(s).resolve() for s in a.sessions} & {Path(s).resolve() for s in a.test_sessions}
    if overlap:
        raise SystemExit(f'sessions in both train and test: {overlap}')
    out = Path(a.out).expanduser()
    if out.exists():
        raise SystemExit(f'{out} exists')
    out.mkdir(parents=True)
    order, i = [], 0
    for split, sess in (('train', a.sessions), ('test', a.test_sessions)):
        for e in collect(sess, a, required, skipped):
            link = out / f'{i:06d}_{e["episode_id"]}'
            link.symlink_to(e['_bag'], target_is_directory=True)
            order.append({'index': i, 'split': split, 'type': e.get('type'), 'stage': e.get('stage'),
                          'episode_id': e['episode_id'], 'bag': e['_bag']})
            i += 1
    ranges = {}
    for o in order:
        k = f'{o["split"]}:{o["type"]}'
        lo, hi = ranges.get(k, (o['index'], o['index']))
        ranges[k] = (min(lo, o['index']), max(hi, o['index']) + 1)
    order_file = out.parent / f'{out.name}_order.json'
    order_file.write_text(json.dumps({'episodes': order, 'ranges': ranges, 'skipped': skipped}, indent=1))
    for k, (lo, hi) in ranges.items():
        print(f'{k:35s} episodes [{lo}, {hi})')
    for x in skipped:
        print(f'SKIPPED {x["bag"]}: missing {", ".join(x["missing"])} (the converter needs every topic)')
    print(f'{len(order)} episodes staged in {out}; order in {order_file}'
          + (f'; {len(skipped)} skipped (listed above and in the order file)' if skipped else ''))


if __name__ == '__main__':
    main()
