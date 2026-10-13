#!/usr/bin/env python3
"""Select and order episode bags for conversion (bag_to_lerobot takes a folder of bags in name
order). Writes a staging folder of symlinks:

  python stage_bags.py --sessions ~/data/S1 ~/data/S2 --test-sessions ~/data/S3 -o ~/datasets/staging

* Simulated sessions / episodes (sim: true) are refused outright.
* Synthetic pipeline-check episodes (synthetic_pipeline_check: true, written by pipeline_check.py)
  are refused unless --pipeline-check, which in turn refuses every episode that is not synthetic.
* Stage 0 episodes are excluded unless --include-stage0; Stage 1 unless --include-stage1
  (open decision 6); discards and episodes whose QC failed are always excluded.
* Order inside train / test: success, contact-establishment, recovery-start, recovery-mid,
  failure, so the recovery and failure ranges in info.json are contiguous.
* Test = whole held-out sessions (never episodes from sessions also used for training).
Prints the episode ranges per split and type; openh_export.py reads <out>_order.json.
"""
import argparse
import json
from pathlib import Path

TYPE_ORDER = ['success', 'shakedown', 'scripted', 'contact-establishment', 'recovery-start',
              'recovery-mid', 'failure']


def check_synthetic(ep, pipeline_check, where):
    synthetic = bool(ep.get('synthetic_pipeline_check'))
    if synthetic and not pipeline_check:
        raise SystemExit(f'{where}: synthetic pipeline-check episode: only with --pipeline-check')
    if pipeline_check and not synthetic:
        raise SystemExit(f'{where}: real episode in a --pipeline-check run: refused (never mixed)')


def collect(sessions, a):
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
            if e.get('outcome') == 'discard' or not (e.get('qc') or {}).get('pass'):
                continue
            if e.get('stage') == 0 and not a.include_stage0:
                continue
            if e.get('stage') == 1 and not a.include_stage1:
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
    ap.add_argument('-o', '--out', required=True)
    ap.add_argument('--pipeline-check', action='store_true', help='synthetic episodes only (pipeline_check.py)')
    a = ap.parse_args()
    overlap = {Path(s).resolve() for s in a.sessions} & {Path(s).resolve() for s in a.test_sessions}
    if overlap:
        raise SystemExit(f'sessions in both train and test: {overlap}')
    out = Path(a.out).expanduser()
    if out.exists():
        raise SystemExit(f'{out} exists')
    out.mkdir(parents=True)
    order, i = [], 0
    for split, sess in (('train', a.sessions), ('test', a.test_sessions)):
        for e in collect(sess, a):
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
    order_file.write_text(json.dumps({'episodes': order, 'ranges': ranges}, indent=1))
    for k, (lo, hi) in ranges.items():
        print(f'{k:35s} episodes [{lo}, {hi})')
    print(f'{len(order)} episodes staged in {out}; order in {order_file}')


if __name__ == '__main__':
    main()
