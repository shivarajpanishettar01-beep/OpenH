"""Progress against the Open-H V2 commitment: hours per phantom, episode lengths, and the share of
failure / recovery episodes. Counts exactly the episodes the export takes (usrig_common.export_rules,
the rule stage_bags uses: real, kept, QC-passed; Stage 0 / 1 only with --include-stage0 / 1; freedrive
at any stage unless --exclude-kinesthetic), each as duration_s - paused_s.

  ros2 run usrig_qc collection_progress [--include-stage0] [--include-stage1] [--exclude-kinesthetic]
      [--config-dir DIR] [--json out.json]
"""
import argparse
import json
from pathlib import Path

from usrig_common import config
from usrig_common.export_rules import FAILURE_RECOVERY_TYPES, REASONS, exportable, recorded_s
from usrig_common.session import data_root, episode_files


def add_export_args(ap):
    """The stage_bags selection options, so progress / plan count what that export will contain."""
    ap.add_argument('--include-stage0', action='store_true')
    ap.add_argument('--include-stage1', action='store_true')
    ap.add_argument('--exclude-kinesthetic', action='store_true')


def export_opts(a):
    return {'include_stage0': a.include_stage0, 'include_stage1': a.include_stage1,
            'include_kinesthetic': not a.exclude_kinesthetic}


def progress(scfg, root=None, **opts):
    """opts: exportable() options (stage_bags defaults when omitted)."""
    root = Path(root) if root else data_root(scfg, False)
    phantoms = scfg.get('phantoms') or {}
    tgt = scfg.get('targets') or {}
    lo, hi = (tgt.get('episode_s') or [None, None])
    rows = {p: {'hours': 0.0, 'episodes': 0, 'failure_recovery': 0, 'outside_length': 0} for p in phantoms}
    skipped = {**{k: 0 for k in REASONS}, 'unknown_phantom': 0}
    for f in episode_files(root):
        e = json.loads(f.read_text())
        ok, why = exportable(e, **opts)
        if not ok:
            skipped[why] += 1
            continue
        r = rows.get(e.get('phantom_id'))
        if r is None:
            skipped['unknown_phantom'] += 1
            continue
        d = recorded_s(e)
        r['hours'] += d / 3600
        r['episodes'] += 1
        r['failure_recovery'] += e.get('type') in FAILURE_RECOVERY_TYPES
        if lo is not None and not (lo <= d <= hi) and e.get('stage') == 2:
            r['outside_length'] += 1
    total_h = sum(r['hours'] for r in rows.values())
    total_n = sum(r['episodes'] for r in rows.values())
    fr = sum(r['failure_recovery'] for r in rows.values())
    return {'root': str(root), 'phantoms': {p: {**r, 'target_hours': phantoms[p].get('target_hours')}
                                           for p, r in rows.items()},
            'total_hours': total_h, 'target_hours': tgt.get('total_hours'), 'episodes': total_n,
            'failure_recovery_fraction': (fr / total_n) if total_n else None,
            'target_failure_recovery_fraction': tgt.get('failure_recovery_fraction'), 'skipped': skipped}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config-dir')
    ap.add_argument('--json')
    add_export_args(ap)
    a, _ = ap.parse_known_args()
    p = progress(config.load_config('session', a.config_dir), **export_opts(a))
    for name, r in p['phantoms'].items():
        print(f'{name:20s} {r["hours"]:6.2f} / {r["target_hours"]} h  {r["episodes"]:5d} episodes  '
              f'failure/recovery {r["failure_recovery"]}  stage-2 length outside range {r["outside_length"]}')
    frac = p['failure_recovery_fraction']
    print(f'TOTAL {p["total_hours"]:.2f} / {p["target_hours"]} h, {p["episodes"]} episodes, failure/recovery '
          f'{"n/a" if frac is None else f"{frac:.0%}"} (target {p["target_failure_recovery_fraction"]:.0%})')
    print(f'not counted: {p["skipped"]}')
    if a.json:
        Path(a.json).write_text(json.dumps(p, indent=2))


if __name__ == '__main__':
    main()
