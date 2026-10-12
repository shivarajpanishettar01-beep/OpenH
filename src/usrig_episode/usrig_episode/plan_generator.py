"""Stage 2 plan generator: plan_v1.csv with fixed seeds, balanced and shuffled.

  ros2 run usrig_episode plan_generator generate --waypoints waypoints/<phantom>.yaml \
      --rows 400 --seed 1 --operators op1 op2 -o plan_v1.csv
  ros2 run usrig_episode plan_generator lock --plan plan_v1.csv --session <session_dir>

* start offsets: 8 directions x 2 radii (10 mm, 25 mm), plus rock / tilt / rotation in +-10 deg
* types: 55 % success, 30 % recovery (recovery-start at most a third of it), 15 % failure
  (optional --contact-frac of contact-establishment taken from success)
* every recovery / failure row gets an FM ID and a staging method from failure_catalog.yaml
* 3-5 paraphrased instructions per target (from the waypoint file); a second operator on at
  least 20 % of rows
* the plan is locked by storing its SHA-256 in session.json before collection
"""
import argparse
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

COLUMNS = ['row', 'seed', 'target', 'type', 'instruction', 'operator_id',
           'off_dx_mm', 'off_dy_mm', 'off_dz_mm', 'off_rock_deg', 'off_tilt_deg', 'off_rot_deg',
           'failure_mode', 'staging_method', 'disturbance_kind', 'disturbance_delta',
           'disturbance_onset_s', 'disturbance_duration_s']

DIRECTIONS = 8
RADII_MM = (10.0, 25.0)
ANGLE_RANGE_DEG = 10.0


def type_counts(n, contact_frac=0.0):
    n_fail = round(0.15 * n)
    n_rec = round(0.30 * n)
    n_rec_start = n_rec // 3
    n_contact = round(contact_frac * n)
    n_succ = n - n_fail - n_rec - n_contact
    return {'success': n_succ, 'recovery-start': n_rec_start, 'recovery-mid': n_rec - n_rec_start,
            'failure': n_fail, 'contact-establishment': n_contact}


def load_catalog(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _null_in(x):
    if x is None:
        return True
    if isinstance(x, (list, tuple)):
        return any(_null_in(v) for v in x)
    if isinstance(x, dict):
        return any(_null_in(v) for v in x.values())
    return False


def generate(targets, n_rows, seed, operators, catalog, contact_frac=0.0, onset_range=(5.0, 15.0),
             second_operator_frac=0.2):
    rng = np.random.default_rng(seed)
    for name, t in targets.items():
        k = len(t.get('instructions') or [])
        if not 3 <= k <= 5:
            raise ValueError(f'target {name}: needs 3-5 instructions, has {k}')
    counts = type_counts(n_rows, contact_frac)
    types = [t for t, c in counts.items() for _ in range(c)]

    fm_methods = [(fm, m) for fm, spec in catalog.items() for m in spec['methods']]
    needed = []
    rows = []
    target_names = sorted(targets)
    offsets = [(r, d) for r in RADII_MM for d in range(DIRECTIONS)]
    fm_i = 0
    for i, typ in enumerate(types):
        target = target_names[i % len(target_names)]
        r, d = offsets[(i // len(target_names)) % len(offsets)]
        ang = 2 * math.pi * d / DIRECTIONS
        row = {'type': typ, 'target': target,
               'off_dx_mm': round(r * math.cos(ang), 3), 'off_dy_mm': round(r * math.sin(ang), 3),
               'off_dz_mm': 0.0,
               'off_rock_deg': round(float(rng.uniform(-ANGLE_RANGE_DEG, ANGLE_RANGE_DEG)), 2),
               'off_tilt_deg': round(float(rng.uniform(-ANGLE_RANGE_DEG, ANGLE_RANGE_DEG)), 2),
               'off_rot_deg': round(float(rng.uniform(-ANGLE_RANGE_DEG, ANGLE_RANGE_DEG)), 2),
               'failure_mode': '', 'staging_method': '', 'disturbance_kind': 'none',
               'disturbance_delta': '', 'disturbance_onset_s': '', 'disturbance_duration_s': ''}
        instr = list(targets[target]['instructions'])
        row['instruction'] = instr[int(rng.integers(len(instr)))]
        if typ in ('recovery-start', 'recovery-mid', 'failure'):
            fm, method = fm_methods[fm_i % len(fm_methods)]
            fm_i += 1
            spec = catalog[fm]['methods'][method]
            row['failure_mode'], row['staging_method'] = fm, method
            if fm == 'FM-12':
                others = [n for n in target_names if n != target]
                if method == 'mismatched_instruction' and others:
                    o = others[int(rng.integers(len(others)))]
                    row['instruction'] = targets[o]['instructions'][0]
                elif method == 'ambiguous_instruction':
                    row['instruction'] = 'Find the view.'
            if spec.get('kind', 'none') != 'none':
                if _null_in(spec.get('delta')) or spec.get('duration_s') is None:
                    needed.append(f'{fm}.{method}')
                else:
                    row['disturbance_kind'] = spec['kind']
                    row['disturbance_delta'] = json.dumps(spec['delta'])
                    row['disturbance_duration_s'] = spec['duration_s']
                    row['disturbance_onset_s'] = (0.0 if typ == 'recovery-start'
                                                  else round(float(rng.uniform(*onset_range)), 2))
        rows.append(row)
    if needed:
        raise ValueError('failure_catalog.yaml magnitudes are null for: ' + ', '.join(sorted(set(needed))))

    order = rng.permutation(len(rows))
    rows = [rows[i] for i in order]
    # operators: second operator on at least second_operator_frac of rows (when given)
    n_second = math.ceil(second_operator_frac * len(rows)) if len(operators) > 1 else 0
    second_idx = set(rng.choice(len(rows), n_second, replace=False).tolist()) if n_second else set()
    for i, row in enumerate(rows):
        row['row'] = i
        row['seed'] = int(seed) * 100000 + i
        row['operator_id'] = operators[1] if i in second_idx else operators[0]
    return rows


def write_csv(rows, path):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in COLUMNS})


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('generate')
    g.add_argument('--waypoints', required=True)
    g.add_argument('--catalog', default=None, help='failure_catalog.yaml (default: package copy)')
    g.add_argument('--rows', type=int, required=True)
    g.add_argument('--seed', type=int, required=True)
    g.add_argument('--operators', nargs='+', required=True)
    g.add_argument('--contact-frac', type=float, default=0.0)
    g.add_argument('-o', '--output', required=True)
    lk = sub.add_parser('lock')
    lk.add_argument('--plan', required=True)
    lk.add_argument('--session', required=True, help='session directory (session.json)')
    a = ap.parse_args(argv)

    if a.cmd == 'generate':
        catalog_path = a.catalog
        if catalog_path is None:
            from ament_index_python.packages import get_package_share_directory
            catalog_path = Path(get_package_share_directory('usrig_episode')) / 'config' / 'failure_catalog.yaml'
        with open(a.waypoints) as f:
            wp = yaml.safe_load(f)
        targets = {w['name']: w for w in wp['waypoints'] if w.get('verified')}
        if not targets:
            sys.exit('no verified waypoints in ' + a.waypoints)
        try:
            rows = generate(targets, a.rows, a.seed, a.operators, load_catalog(catalog_path), a.contact_frac)
        except ValueError as e:
            sys.exit(str(e))
        if Path(a.output).exists():
            sys.exit(f'{a.output} exists; plans are never overwritten')
        write_csv(rows, a.output)
        c = {}
        for r in rows:
            c[r['type']] = c.get(r['type'], 0) + 1
        print(f'wrote {len(rows)} rows to {a.output}: {c}')
    else:
        from usrig_common.session import update_session_json
        h = sha256(a.plan)
        update_session_json(a.session, plan_lock={'plan_file': str(Path(a.plan).resolve()), 'sha256': h})
        print(f'locked {a.plan} sha256={h}')


if __name__ == '__main__':
    main()
