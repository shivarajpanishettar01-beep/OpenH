"""label_agreement: inter-rater agreement of the human labels (phases, gestures, g_t, episode labels).

  ros2 run usrig_qc label_agreement ~/data/sessions/S1 [~/data/sessions/S2 ...] [--raters alice bob] [--hz 10]
      [--json agreement.json] [--pick 0.15 [--seed 0]]

Uses every episode's annotation.json + annotation.<annotator>.json (merged). Only human ratings count: a rater's
own labels (provenance expert / trainee / model_assisted, with an `annotator`) and every entry the rater ACCEPTED
(an accepted automated segment is that rater's rating); unreviewed automated labels and labels whose QA status
is rejected are ignored. For kappa, the second rater labels blind (`annotator --blind`).
- Segment layers (phases, status_intervals = g_t): each rater's label is sampled every 1/hz s over the
  episode (start..end event); "none" where a rater has no segment. Two raters: Cohen's kappa; more: Fleiss'.
- Gestures (several at once): one yes/no item per gesture and sample, kappa over all of them.
- Episode labels (failure_class, outcome: nominal kappa; motion_quality 1-5: quadratic-weighted kappa) from
  `ratings` (each annotator's own value).
Only episodes labelled by at least two of the raters enter a statistic; the number of episodes is printed
next to every value. The Landis & Koch band is a reading aid; the acceptance threshold is yours.
Prints how many rated episodes are double-rated (plan: 10-20 %); --pick lists single-rated episodes to give
a second rater, round-robin over episode types.
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path

from usrig_common import agreement as AG
from usrig_common import annotation as A

HUMAN = {'expert', 'trainee', 'model_assisted'}
SEGMENT_LAYERS = {'phases': 'phase', 'status_intervals': 'status'}
EPISODE_LABELS = {'failure_class': None, 'outcome': None, 'motion_quality': 'quadratic'}


def human_segments(ann, layer):
    """{annotator: [segments]}: each rater's own non-rejected segments, plus every segment (automated or another
    rater's) that this rater accepted: accepting is rating it."""
    out = defaultdict(list)
    for s in ann.get(layer) or []:
        if s.get('provenance') in HUMAN and s.get('annotator') and s.get('qa_status') != 'rejected':
            out[s['annotator']].append(s)
        for who, r in (s.get('qa_reviews') or {}).items():
            if r.get('qa_status') == 'accepted' and who != s.get('annotator'):
                out[who].append(s)
    return out


def sample_times(ep, hz):
    t0, t1 = ep.get('t_start_ns'), ep.get('t_end_ns')
    if t0 is None or t1 is None or t1 <= t0:
        return []
    step = int(1e9 / hz)
    return list(range(int(t0), int(t1), step))


class Aligned:
    """{rater: [labels]} kept aligned item by item: a rater missing for an item gets None there."""

    def __init__(self):
        self.n, self.by = 0, {}

    def add(self, values_by_rater, length=1):
        """values_by_rater: {rater: list of `length` labels} for the next `length` items."""
        for r in values_by_rater:
            self.by.setdefault(r, [None] * self.n)
        for r, labels in self.by.items():
            labels.extend(values_by_rater.get(r, [None] * length))
        self.n += length


def collect(sessions, raters=None, hz=10.0):
    """Per label: Aligned over all items, and the number of episodes with at least two of the raters."""
    seg = {layer: Aligned() for layer in SEGMENT_LAYERS}
    ges = Aligned()
    epl = {name: Aligned() for name in EPISODE_LABELS}
    eps = defaultdict(int)
    coverage = []                  # per episode: dir, type, human raters (any label)
    for s in sessions:
        for d in sorted(Path(s).expanduser().glob('ep_*')):
            ef = d / 'episode.json'
            if not ((d / A.BASE_FILE).exists() or A.annotator_files(d)) or not ef.exists():
                continue
            ann, ep = A.load_merged(d), json.loads(ef.read_text())
            coverage.append({'dir': str(d), 'type': ep.get('type'), 'sim': bool(ep.get('sim')),
                             'raters': sorted(r for r in rater_ids(ann) if raters is None or r in raters)})
            ts = sample_times(ep, hz)
            for layer, key in SEGMENT_LAYERS.items():
                by = human_segments(ann, layer)
                who = [r for r in by if raters is None or r in raters]
                if len(who) >= 2 and ts:
                    eps[layer] += 1
                    seg[layer].add({r: A.segment_at(by[r], key, ts, default='none') for r in who}, len(ts))
            by = human_segments(ann, 'gestures')
            who = [r for r in by if raters is None or r in raters]
            if len(who) >= 2 and ts:
                eps['gestures'] += 1
                vals = {}
                for r in who:
                    v = []
                    for g in A.GESTURES:
                        segs = [x for x in by[r] if x['gesture'] == g]
                        v += [x == g for x in A.segment_at(segs, 'gesture', ts, default='')]
                    vals[r] = v
                ges.add(vals, len(ts) * len(A.GESTURES))
            for name in EPISODE_LABELS:
                rated = {k: v['value'] for k, v in ((ann.get('ratings') or {}).get(name) or {}).items() if v}
                who = [r for r in rated if raters is None or r in raters]
                if len(who) >= 2:
                    eps[name] += 1
                    epl[name].add({r: [rated[r]] for r in who}, 1)
    return seg, ges, epl, eps, coverage


def rater_ids(ann):
    """Human annotators with any rating on this episode (segments, accepted entries or episode labels)."""
    who = set()
    for layer in (*SEGMENT_LAYERS, 'gestures'):
        who |= set(human_segments(ann, layer))
    for r in (ann.get('ratings') or {}).values():
        who |= {k for k, v in (r or {}).items() if v}
    return who


def pick_for_double_rating(coverage, fraction, seed=0):
    """Episodes (real, rated by exactly one rater) to give a second rater so that `fraction` of the rated
    episodes are double-rated, stratified by episode type (seeded, reproducible)."""
    import random
    real = [c for c in coverage if not c['sim'] and c['raters']]
    need = max(0, round(fraction * len(real)) - sum(len(c['raters']) >= 2 for c in real))
    single = [c for c in real if len(c['raters']) == 1]
    by_type = defaultdict(list)
    for c in single:
        by_type[c['type'] or 'unknown'].append(c)
    rng = random.Random(seed)
    for v in by_type.values():
        rng.shuffle(v)
    out, types = [], sorted(by_type)
    while len(out) < min(need, len(single)):
        for t in types:                        # round-robin: rare types (failure, recovery) get double-rated too
            if by_type[t] and len(out) < need:
                out.append(by_type[t].pop())
    return out


def stat(by_rater, weights=None):
    """(kappa, percent agreement or None, method) for {rater: labels} aligned item by item."""
    raters = sorted(by_rater)
    if len(raters) < 2:
        return None, None, '-'
    if len(raters) == 2:
        a, b = by_rater[raters[0]], by_rater[raters[1]]
        return AG.cohen_kappa(a, b, weights), AG.percent_agreement(a, b), 'Cohen' + (f' ({weights})' if weights else '')
    items = [[by_rater[r][i] for r in raters] for i in range(len(by_rater[raters[0]]))]
    items = [it for it in items if all(x is not None for x in it)]
    return AG.fleiss_kappa(items), None, f'Fleiss ({len(raters)} raters)'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--raters', nargs='+')
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--json')
    ap.add_argument('--pick', type=float, metavar='FRACTION',
                    help='list single-rated episodes to give a second rater (blind) so FRACTION (e.g. 0.15) are '
                         'double-rated, stratified by type')
    ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args(argv)
    seg, ges, epl, eps, coverage = collect(a.sessions, a.raters, a.hz)
    rated = [c for c in coverage if c['raters']]
    n2 = sum(len(c['raters']) >= 2 for c in rated)
    print(f'double-rated: {n2} of {len(rated)} rated episodes'
          + (f' ({n2 / len(rated):.0%}; plan 10-20 %)' if rated else ''))
    if a.pick is not None:
        for c in pick_for_double_rating(coverage, a.pick, a.seed):
            print(f'  second rater (annotator --blind): {c["dir"]}  type {c["type"]}  rated by {c["raters"][0]}')
    rows = []
    for layer in SEGMENT_LAYERS:
        rows.append((layer, eps[layer], *stat(seg[layer].by)))
    rows.append(('gestures', eps['gestures'], *stat(ges.by)))
    for name, w in EPISODE_LABELS.items():
        rows.append((name, eps[name], *stat(epl[name].by, w)))
    print(f'{"label":16s} {"episodes":>8s} {"kappa":>7s} {"agree":>6s}  method / reading')
    out = []
    for name, n, k, pa, method in rows:
        ks = '-' if k is None else f'{k:.3f}'
        ps = '-' if pa is None else f'{pa:.0%}'
        print(f'{name:16s} {n:8d} {ks:>7s} {ps:>6s}  {method}, {AG.landis_koch(k)}')
        out.append({'label': name, 'episodes': n, 'kappa': k, 'percent_agreement': pa, 'method': method,
                    'landis_koch': AG.landis_koch(k)})
    if not any(n for _, n, *_ in rows):
        print('no episode has labels from two or more human annotators yet (annotator, different annotator ids)')
    if a.json:
        Path(a.json).write_text(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
