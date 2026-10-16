"""Automated annotation layers for one episode, from its bag.

  ros2 run usrig_qc auto_annotate <session>/ep_0001 [--config-dir DIR]

Writes <ep>/annotation.json: phase segments (from phase events), gesture segments (from teleop
key events), failure / recovery status intervals (g_t), task and outcome. Everything written
here has provenance "automated". Labels with any other provenance already in the file
(expert / trainee / model_assisted, from before per-annotator files) are kept untouched; the
annotator writes annotation.<annotator>.json, which this never touches.
"""
import argparse
import json
from pathlib import Path

from usrig_common import annotation as A
from usrig_common import config

from .episode_qc import read_bag, stamp_ns


def auto_annotate(ep_dir, episode, keymap, write=True):
    ep_dir = Path(ep_dir)
    data, _ = read_bag(ep_dir, ['/episode/events', '/operator/command'])
    events = [(stamp_ns(m), m.kind, json.loads(m.value or '{}')) for _, m in data['/episode/events']]
    keys = []
    for t, m in data['/operator/command']:
        try:
            c = json.loads(m.data)
        except ValueError:
            continue
        if c.get('cmd') == 'key':
            keys.append((int(c.get('t_ns') or t), c.get('key'), bool(c.get('down'))))
    t_end = episode.get('t_end_ns') or max((e[0] for e in events), default=0)

    path = ep_dir / 'annotation.json'
    ann = json.loads(path.read_text()) if path.exists() else A.new_annotation(episode)
    auto = A.new_annotation(episode)
    auto['phases'] = [dict(s, provenance='automated', qa_status='unreviewed') for s in A.phase_segments(events, t_end)]
    auto['gestures'] = [dict(s, provenance='automated', qa_status='unreviewed') for s in A.gesture_segments(keys, keymap)]
    auto['status_intervals'] = [{'status': s, 't0_ns': int(t0), 't1_ns': (int(t1) if t1 else None),
                                 'provenance': 'automated', 'qa_status': 'unreviewed'}
                                for s, t0, t1 in A.status_intervals(events)]
    # replace automated entries, keep human / model-assisted ones
    for layer in ('phases', 'gestures', 'status_intervals'):
        kept = [x for x in ann.get(layer, []) if x.get('provenance') != 'automated']
        ann[layer] = kept + auto[layer]
    for single in ('task', 'outcome'):
        if ann.get(single) is None or ann[single].get('provenance') == 'automated':
            ann[single] = auto[single]
    A.validate(ann)
    if write:                       # atomic: the annotator may be reading it (it never writes it)
        tmp = path.with_suffix('.tmp')
        tmp.write_text(A.dumps(ann) + '\n')
        tmp.replace(path)
    return ann


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('episode_dir')
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    ep = json.loads((Path(a.episode_dir) / 'episode.json').read_text())
    ann = auto_annotate(a.episode_dir, ep, config.load_config('keymap', a.config_dir))
    print(f'phases {len(ann["phases"])}, gestures {len(ann["gestures"])}, '
          f'status intervals {len(ann["status_intervals"])} -> {Path(a.episode_dir) / "annotation.json"}')


if __name__ == '__main__':
    main()
