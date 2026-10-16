#!/usr/bin/env python3
"""Dataset card + checksum manifest for an exported Open-H dataset, built from the data itself.

  python export/release_card.py --dataset ~/datasets/iitmandi_us            # writes README.md + MANIFEST.sha256
  python export/release_card.py --dataset ~/datasets/iitmandi_us --check    # verify the manifest only

Run in the export env (conda openh-export: pyarrow). Every number in the card comes from the dataset:
meta/info.json (features, splits, totals), meta/provenance.jsonl (per episode: phantom, type, operator,
session, collection mode, capture PC, code commit, time offsets; written by openh_export.py), the data
parquet files (g_t label shares, expert / command_valid shares, instructions), meta/annotations.jsonl
(label provenance) and meta/calibration/. Nothing is filled from memory; what is missing is listed under
"Known limitations" instead of being invented. Targets come from session.yaml `phantoms` / `targets`.
A dataset built from synthetic pipeline-check episodes gets a SYNTHETIC banner.
"""
import argparse
import hashlib
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
LICENCE = 'OpenMDW-1.1 (dataset); code: MIT (github.com/shivarajpanishettar01-beep/OpenH)'
SOURCES = {0: 'teleop', 1: 'scripted_task', 2: 'transit', 3: 'perturbation', 4: 'none', 5: 'kinesthetic'}


def read_jsonl(p):
    p = Path(p)
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []


def sha256(p, chunk=1 << 20):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(chunk), b''):
            h.update(b)
    return h.hexdigest()


def manifest_files(ds):
    return sorted(f for f in ds.rglob('*') if f.is_file() and f.name != 'MANIFEST.sha256')


def write_manifest(ds):
    lines = [f'{sha256(f)}  {f.relative_to(ds)}' for f in manifest_files(ds)]
    (ds / 'MANIFEST.sha256').write_text('\n'.join(lines) + '\n')
    return len(lines)


def check_manifest(ds):
    """[(path, problem)]: missing, changed, or not listed."""
    m = ds / 'MANIFEST.sha256'
    if not m.exists():
        return [('MANIFEST.sha256', 'missing')]
    listed = {}
    for line in m.read_text().splitlines():
        if line.strip():
            h, rel = line.split('  ', 1)
            listed[rel] = h
    out = []
    for rel, h in listed.items():
        f = ds / rel
        if not f.exists():
            out.append((rel, 'missing'))
        elif sha256(f) != h:
            out.append((rel, 'changed'))
    for f in manifest_files(ds):
        if str(f.relative_to(ds)) not in listed:
            out.append((str(f.relative_to(ds)), 'not in the manifest'))
    return out


def data_columns(ds, cols):
    import pyarrow.parquet as pq
    out = defaultdict(list)
    for f in sorted((ds / 'data').rglob('*.parquet')):
        names = pq.read_schema(f).names
        t = pq.read_table(f, columns=[c for c in cols if c in names])
        for c in t.column_names:
            out[c].extend(t.column(c).to_pylist())
    return out


def summarize(ds, scfg):
    info = json.loads((ds / 'meta' / 'info.json').read_text())
    fps = float(info['fps'])
    prov = read_jsonl(ds / 'meta' / 'provenance.jsonl')
    ann = read_jsonl(ds / 'meta' / 'annotations.jsonl')
    cal_dir = ds / 'meta' / 'calibration'
    cal = {f.stem: json.loads(f.read_text()) for f in cal_dir.glob('*.json')} if cal_dir.exists() else {}
    cols = data_columns(ds, ['episode_index', 'observation.meta.status', 'observation.meta.action_is_expert',
                             'observation.meta.command_valid', 'observation.meta.action_source', 'instruction.text',
                             'observation.meta.status_source', 'observation.meta.phase_source',
                             'observation.meta.in_episode', 'observation.meta.paused'])
    n_steps = len(cols.get('episode_index', []))
    s = {'info': info, 'fps': fps, 'prov': prov, 'ann': ann, 'cal': cal, 'n_steps': n_steps,
         'synthetic': 'pipeline_check' in info or any(p.get('synthetic_pipeline_check') for p in prov)}
    # hours = episode time without pauses (as collection_progress counts it), not the whole bag; older exports
    # without the flags count every step
    ins, pau = cols.get('observation.meta.in_episode'), cols.get('observation.meta.paused')
    n_ep = sum(bool(i) and not bool(p) for i, p in zip(ins, pau)) if ins and pau else n_steps
    s['n_steps_episode'] = n_ep
    s['hours'] = n_ep / fps / 3600.0
    s['status'] = Counter(cols.get('observation.meta.status', []))
    s['label_sources'] = {k: dict(Counter(x or 'none' for x in cols.get(f'observation.meta.{k}_source', [])))
                          for k in ('status', 'phase')}
    s['expert_share'] = (sum(bool(x) for x in cols.get('observation.meta.action_is_expert', [])) / n_steps) if n_steps else 0.0
    cv = cols.get('observation.meta.command_valid', [])
    s['command_valid_share'] = (sum(bool(x) for x in cv) / len(cv)) if cv else None
    src = Counter(int(x) for x in cols.get('observation.meta.action_source', []) if x == x)
    s['sources'] = {SOURCES.get(k, str(k)): v for k, v in sorted(src.items())}
    s['instructions'] = len(set(cols.get('instruction.text', [])))
    by = lambda key: Counter(p.get(key) or 'unknown' for p in prov)  # noqa: E731
    s['types'], s['modes'], s['operators'] = by('type'), by('collection_mode'), by('operator_id')
    s['sessions'], s['hosts'], s['commits'] = by('session_id'), by('capture_host'), by('rig_git_commit')
    s['test_sessions'] = sorted({p.get('session_id') or 'unknown' for p in prov if p.get('split') == 'test'})
    hours_ph = defaultdict(float)
    fr_steps = defaultdict(float)
    for p in prov:
        h = (p.get('n_steps_episode', p.get('n_steps')) or 0) / fps / 3600.0
        hours_ph[p.get('phantom_id') or 'unknown'] += h
        if p.get('type') in ('failure', 'recovery-start', 'recovery-mid'):
            fr_steps[p.get('phantom_id') or 'unknown'] += h
    s['hours_phantom'], s['fr_hours_phantom'] = dict(hours_ph), dict(fr_steps)
    offs = defaultdict(set)
    for p in prov:
        for k, v in (p.get('time_offsets_s') or {}).items():
            offs[(p.get('capture_host') or 'unknown', k)].add(v)
    s['offsets'] = {f'{h} {k}': sorted(v, key=str) for (h, k), v in offs.items()}
    lab = Counter()
    for a in ann:
        for kind in ('phases', 'gestures', 'status_intervals'):
            for x in a.get(kind) or []:
                lab[(kind, x.get('provenance', 'automated'), x.get('qa_status', '-'))] += 1
    s['labels'] = lab
    s['targets'] = {'phantoms': {k: (v or {}).get('target_hours') for k, v in (scfg.get('phantoms') or {}).items()},
                    'total_hours': (scfg.get('targets') or {}).get('total_hours'),
                    'failure_recovery_fraction': (scfg.get('targets') or {}).get('failure_recovery_fraction')}
    return s


def cal_present(c, name):
    """A calibration file has its matrix for every session (per_session) or, in older exports, at the top level."""
    per = (c or {}).get('per_session')
    if per:
        return all((v.get('matrix') if isinstance(v, dict) and 'matrix' in v else v) is not None for v in per.values())
    return (c or {}).get(name) is not None


def limitations(s):
    out = []
    cal = s['cal']
    if not cal_present(cal.get('prob_cali_mtx'), 'prob_cali_mtx'):
        out.append('Ultrasound image-to-probe calibration (prob_cali_mtx) missing (in at least one session).')
    if not cal_present(cal.get('wrist_cali_mtx'), 'wrist_cali_mtx'):
        out.append('Wrist camera hand-eye calibration (wrist_cali_mtx) missing (in at least one session).')
    fallback = sorted({f'{k} ({sid})' for k in ('prob_cali_mtx', 'wrist_cali_mtx')
                       for sid, v in ((cal.get(k) or {}).get('per_session') or {}).items()
                       if 'exporting PC' in str((v or {}).get('source'))})
    if fallback:
        out.append('Calibration taken from the exporting PC, not snapshotted with the session: ' + ', '.join(fallback))
    if not s['prov']:
        out.append('meta/provenance.jsonl missing: per-episode provenance unknown (export with the current openh_export.py).')
    for k in ('hosts', 'commits', 'operators'):
        if 'unknown' in s[k]:
            out.append(f'{s[k]["unknown"]} episode(s) without {k[:-1]} information.')
    if any(len(v) > 1 for v in s['offsets'].values()):
        out.append('Time offsets changed within a capture PC across episodes: check the temporal calibration history.')
    if not any(k[1] == 'expert' for k in s['labels']):
        out.append('All phase / gesture / g_t labels are automated (no expert annotation yet).')
    if s['test_sessions'] == []:
        out.append('No held-out test sessions.')
    out.append('Open-H validate_formatting.py: run it on this folder before submission (not part of this repository).')
    return out


def fmt_h(h):
    return f'{h:.2f} h' if h >= 0.1 else f'{h * 60:.1f} min'


def known(counter):
    """Distinct known values ('unknown' is reported under Known limitations, not counted)."""
    return len([k for k in counter if k != 'unknown'])


def card(s, name):
    i, t = s['info'], s['targets']
    L = []
    if s['synthetic']:
        L += ['> **SYNTHETIC PIPELINE-CHECK DATA. Not a real dataset; never release.**', '']
    L += [f'# {name}: robotic ultrasound on phantoms (UR7e + Telemed probe)', '',
          'Generated by `export/release_card.py` from the dataset files; edit only the free-text parts.', '',
          '## Summary',
          f'- Episodes: {i.get("total_episodes")} ({fmt_h(s["hours"])} of episode time without pauses = '
          f'{s["n_steps_episode"]} steps at {s["fps"]:g} fps; {s["n_steps"]} steps with pre-/post-roll and pauses); '
          f'format LeRobot {i.get("codebase_version")}, robot type `{i.get("robot_type")}`',
          f'- Episode types: {dict(s["types"])}',
          f'- Collection modes: {dict(s["modes"])}',
          f'- Operators: {known(s["operators"])}; sessions: {known(s["sessions"])}; held-out test sessions: '
          f'{", ".join(s["test_sessions"]) or "none"}',
          f'- Distinct instructions: {s["instructions"]}', '',
          '## Progress against the commitment',
          '| phantom | recorded | target h | failure / recovery |', '|---|---|---|---|']
    for ph in sorted(set(s['hours_phantom']) | set(t['phantoms'])):
        L.append(f'| {ph} | {fmt_h(s["hours_phantom"].get(ph, 0.0))} | {t["phantoms"].get(ph, "-")} | '
                 f'{fmt_h(s["fr_hours_phantom"].get(ph, 0.0))} |')
    fr = sum(s['fr_hours_phantom'].values())
    L += [f'| **total** | {fmt_h(s["hours"])} | {t["total_hours"]} | {fmt_h(fr)} '
          f'({(fr / s["hours"] * 100) if s["hours"] else 0:.0f} %, target {(t["failure_recovery_fraction"] or 0) * 100:.0f} %) |', '',
          '## Features', '| key | dtype | shape |', '|---|---|---|']
    for k, f in sorted(i.get('features', {}).items()):
        L.append(f'| `{k}` | {f.get("dtype")} | {f.get("shape")} |')
    L += ['', '## Action',
          '`action` = incremental TCP pose from step t to t+1, `[dx, dy, dz, drx, dry, drz]` (m, rotation vector in rad), '
          f'frame `{i.get("features", {}).get("action", {}).get("info", {}).get("frame", "?")}`, computed from the measured '
          'robot trajectory for every collection mode.',
          '`observation.meta.commanded_twist` / `force_setpoint_n` are the controller command where one existed; '
          '`observation.meta.command_valid` is false where nothing was commanded (freedrive by hand)'
          + (f' ({(1 - s["command_valid_share"]) * 100:.1f} % of steps).' if s['command_valid_share'] is not None else '.'),
          f'Action source per step: {s["sources"]}; `action_is_expert` true on {s["expert_share"] * 100:.1f} % of steps.', '',
          '## Labels',
          f'Per-step g_t: {dict(s["status"])}.',
          f'Per-step label source (resolver: QA-rejected dropped, expert > model_assisted > trainee > automated): '
          f'g_t {s["label_sources"]["status"]}, phase {s["label_sources"]["phase"]}.',
          '| label | provenance | QA | count |', '|---|---|---|---|']
    for (kind, prov, qa), n in sorted(s['labels'].items()):
        L.append(f'| {kind} | {prov} | {qa} | {n} |')
    L += ['', '## Splits', f'`meta/info.json` splits: {i.get("splits")} (test = whole held-out sessions).', '',
          '## Calibration and timing']
    for k, v in sorted(s['cal'].items()):
        present = (cal_present(v, k) if k in ('prob_cali_mtx', 'wrist_cali_mtx') else
                   any(x is not None for x in v.values()) if isinstance(v, dict) else v is not None)
        frame = f' ({v["frame"]})' if isinstance(v, dict) and v.get('frame') else ''
        L.append(f'- `meta/calibration/{k}.json`: {"present" if present else "MISSING"}{frame}')
    for k, v in sorted(s['offsets'].items()):
        L.append(f'- time offset {k}: {v} s')
    L += ['', '## Provenance',
          f'- Capture PCs: {dict(s["hosts"])}', f'- Rig code commits: {dict(s["commits"])}', '',
          '## Known limitations'] + [f'- {x}' for x in limitations(s)] + [
          '', '## Licence', LICENCE, '', '## Files',
          'Every file is listed with its SHA-256 in `MANIFEST.sha256` '
          '(`python export/release_card.py --dataset <dir> --check`).', '']
    return '\n'.join(L)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--name', default=None, help='dataset name in the title (default: folder name)')
    ap.add_argument('--session-config', default=str(HERE.parent / 'src/usrig_bringup/config/session.yaml'))
    ap.add_argument('--check', action='store_true', help='only verify MANIFEST.sha256')
    a = ap.parse_args(argv)
    ds = Path(a.dataset).expanduser()
    if a.check:
        bad = check_manifest(ds)
        for rel, why in bad:
            print(f'  {why}: {rel}')
        print('MANIFEST OK' if not bad else f'MANIFEST: {len(bad)} problem(s)')
        return 0 if not bad else 1
    scfg = yaml.safe_load(Path(a.session_config).read_text())
    s = summarize(ds, scfg)
    (ds / 'README.md').write_text(card(s, a.name or ds.name))
    n = write_manifest(ds)
    print(f'wrote {ds / "README.md"} and MANIFEST.sha256 ({n} files); {fmt_h(s["hours"])}, '
          f'{s["info"].get("total_episodes")} episodes' + ('  [SYNTHETIC]' if s['synthetic'] else ''))
    for x in limitations(s):
        print(f'  limitation: {x}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
