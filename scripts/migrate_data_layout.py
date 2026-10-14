#!/usr/bin/env python3
"""Move data written before 14 Oct 2026 into the layout of usrig_common/session.py.

  python scripts/migrate_data_layout.py            # dry run: prints every move, changes nothing
  python scripts/migrate_data_layout.py --apply    # moves (never deletes, never overwrites)

For session.root and session.sim_root:
  <root>/<session_id>/          (a folder with session.json)  -> <root>/sessions/<session_id>/
  <old micrus out>/<session_id>/                              -> <root>/micrus/<session_id>/
    old micrus out: ~/MicrUs for the real root, ~/MicrUs_sim for the sim root (the former
    micrus.out_root / out_root_sim); only folders named like a session are moved, any other file there
    (e.g. your own micrus captures) is left alone.
  every JSON file in a moved session: paths pointing into the old micrus folder are rewritten.
The asset folders (waypoints, paths, phantoms, calibration) already used <root>/<name> and stay where
they are; test presets saved in <root>/waypoints/base_presets.yaml move to <root>/presets/.
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src' / 'usrig_common'))
from usrig_common import config  # noqa: E402
from usrig_common.session import LAYOUT, data_root  # noqa: E402

OLD_MICRUS = {False: '~/MicrUs', True: '~/MicrUs_sim'}


def rewrite(obj, old, new):
    if isinstance(obj, dict):
        return {k: rewrite(v, old, new) for k, v in obj.items()}
    if isinstance(obj, list):
        return [rewrite(v, old, new) for v in obj]
    if isinstance(obj, str) and (obj == old or obj.startswith(old + "/")):
        return new + obj[len(old):]
    return obj


def plan(scfg):
    moves, rewrites = [], []
    for sim in (False, True):
        root = data_root(scfg, sim)
        if not root.exists():
            continue
        old_mic = Path(OLD_MICRUS[sim]).expanduser()
        for d in sorted(p for p in root.iterdir() if p.is_dir() and p.name not in LAYOUT):
            if not (d / 'session.json').exists():
                continue
            dst = root / 'sessions' / d.name
            moves.append((d, dst))
            mic = old_mic / d.name
            if mic.is_dir():
                mdst = root / 'micrus' / d.name
                moves.append((mic, mdst))
                rewrites.append((dst, str(mic), str(mdst)))
        old_presets = root / 'waypoints' / 'base_presets.yaml'
        if old_presets.exists():
            moves.append((old_presets, root / 'presets' / 'base_presets.yaml'))
    return moves, rewrites


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--config-dir')
    a = ap.parse_args()
    moves, rewrites = plan(config.load_config('session', a.config_dir))
    if not moves:
        print('nothing to migrate: the data already follows the layout')
        return
    clash = [dst for _, dst in moves if dst.exists()]
    if clash:
        raise SystemExit(f'destination exists, not overwriting: {clash}')
    for src, dst in moves:
        print(f'{"move" if a.apply else "would move"}  {src}  ->  {dst}')
    for sdir, old, new in rewrites:
        print(f'{"rewrite" if a.apply else "would rewrite"} paths {old} -> {new} in {sdir}/**/*.json')
    if not a.apply:
        print('\ndry run: nothing changed (add --apply)')
        return
    for src, dst in moves:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(src), str(dst))
    n = 0
    for sdir, old, new in rewrites:
        for f in list(sdir.rglob('*.json')) + list(sdir.glob('.*.json')):
            data = json.loads(f.read_text())
            out = rewrite(data, old, new)
            if out != data:
                f.write_text(json.dumps(out, indent=2) + '\n')
                n += 1
    print(f'done: {len(moves)} moves, {n} JSON files updated')


if __name__ == '__main__':
    main()
