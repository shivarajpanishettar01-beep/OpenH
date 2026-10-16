"""Open-H annotation layers for robotic ultrasound (proposal sections 6-7), pure Python.

Layers: phase -> step -> task -> action/gesture, plus motion quality (1-5), intent, outcome,
reasoning/narration, failure class, per-step status flag g_t, and provenance + QA status for
every label. Automatic labels (provenance "automated") are derived from what the rig records:
  * phase segments from /episode/events phase events,
  * gesture segments from teleop key events on /operator/command,
  * g_t (nominal / failure / recovery) from failure-onset and recovery events.
Files per episode: <ep>/annotation.json holds the automated layers (auto_annotate rewrites it) and
any human entries written before per-annotator files existed. Every expert / trainee /
model-assisted annotator writes only <ep>/annotation.<annotator>.json (own segments, narration,
episode labels, and QA decisions on other entries). load_merged() combines them for the annotator,
the export and label_agreement, so concurrent raters never overwrite each other.
"""
import json
import re
from pathlib import Path

SCHEMA_VERSION = 1

# Proposal section 7: phases for robotic ultrasound.
PHASES = ['approach_and_contact', 'target_localisation', 'view_acquisition', 'sweep_scan',
          'reacquisition_recovery']
# Rig phase events -> proposal phases (events not listed here do not start a phase).
EVENT_PHASE_MAP = {
    'approach': 'approach_and_contact', 'free_air': 'approach_and_contact',
    'contact': 'approach_and_contact', 'search': 'target_localisation',
    'hold': 'view_acquisition', 'scan': 'sweep_scan', 'sweep': 'sweep_scan',
    'recovery': 'reacquisition_recovery', 'recovery_start': 'reacquisition_recovery',
}
GESTURES = ['slide', 'tilt', 'rock', 'rotate', 'compress', 'decompress']
# Teleop keymap actions -> gestures (slide along / across both count as "slide").
AXIS_GESTURE = {'vx': 'slide', 'vy': 'slide', 'wx': 'tilt', 'wy': 'rock', 'wz': 'rotate'}
COMMAND_GESTURE = {'force_up': 'compress', 'force_down': 'decompress'}

# Proposal section 6: four failure classes (task failure and safety state are never conflated).
FAILURE_CLASSES = ['task_performance_failure', 'observation_degradation', 'target_view_loss',
                   'safety_relevant_interaction_state']
# Staged failure modes (failure_catalog.yaml FM ids) -> failure class. PROPOSED: the catalog has no class
# field yet and the mapping waits for the failure-mode / safety protocol (OPEN_DECISIONS); a `failure_class`
# key on a catalog entry overrides this table. FM-04, 08-11 are never staged.
FM_FAILURE_CLASS = {
    'FM-01': 'observation_degradation',            # coupling loss: the image degrades in place
    'FM-02': 'safety_relevant_interaction_state',  # lift-off: the contact state changes
    'FM-03': 'safety_relevant_interaction_state',  # force at the band limit (never staged beyond it)
    'FM-05': 'target_view_loss',                   # out of plane
    'FM-06': 'target_view_loss',                   # wrong view
    'FM-07': 'observation_degradation',            # shadowing
    'FM-12': 'task_performance_failure',           # wrong / ambiguous instruction
}
FAILURE_TYPES = ('failure', 'recovery-start', 'recovery-mid')   # episode types that need a failure class
STATUS = ['nominal', 'failure', 'recovery']
PROVENANCE = ['expert', 'trainee', 'model_assisted', 'automated']
QA_STATUS = ['unreviewed', 'flagged', 'accepted', 'rejected']
QUALITY_TIERS = {5: 'expert', 4: 'expert', 3: 'intermediate', 2: 'novice', 1: 'novice'}


def failure_class_for(failure_mode, catalog=None):
    """Failure class of a plan FM id: the catalog entry's `failure_class` if it has one, else FM_FAILURE_CLASS."""
    fc = ((catalog or {}).get(failure_mode) or {}).get('failure_class')
    return fc or FM_FAILURE_CLASS.get(failure_mode)


def label(value, provenance, qa_status='unreviewed', **extra):
    if provenance not in PROVENANCE:
        raise ValueError(f'provenance must be one of {PROVENANCE}')
    if qa_status not in QA_STATUS:
        raise ValueError(f'qa_status must be one of {QA_STATUS}')
    return {'value': value, 'provenance': provenance, 'qa_status': qa_status, **extra}


def phase_segments(events, t_end_ns):
    """events: [(stamp_ns, kind, value_dict)] -> [{'phase', 't0_ns', 't1_ns'}] (non-overlapping)."""
    starts = []
    for t, kind, v in sorted(events, key=lambda e: e[0]):
        if kind != 'phase':
            continue
        ph = EVENT_PHASE_MAP.get(v.get('phase'))
        if ph and (not starts or starts[-1][1] != ph):
            starts.append((t, ph))
    segs = []
    for i, (t, ph) in enumerate(starts):
        t1 = starts[i + 1][0] if i + 1 < len(starts) else t_end_ns
        if t1 > t:
            segs.append({'phase': ph, 't0_ns': int(t), 't1_ns': int(t1)})
    return segs


def gesture_segments(key_events, keymap):
    """key_events: [(t_ns, key, down)] from /operator/command {"cmd": "key"} -> gesture segments.

    A gesture runs while at least one of its keys is held. Simultaneous gestures overlap.
    """
    key_gesture = {}
    for key, spec in (keymap.get('motion') or {}).items():
        axis = spec.split(':')[0]
        if axis in AXIS_GESTURE:
            key_gesture[str(key).upper()] = AXIS_GESTURE[axis]
    for key, action in (keymap.get('commands') or {}).items():
        if action in COMMAND_GESTURE:
            key_gesture[str(key).upper()] = COMMAND_GESTURE[action]
    held = {}            # gesture -> set of keys held
    started = {}         # gesture -> t0
    segs = []
    for t, key, down in sorted(key_events, key=lambda e: e[0]):
        g = key_gesture.get(str(key).upper())
        if g is None:
            continue
        keys = held.setdefault(g, set())
        if down:
            if not keys:
                started[g] = t
            keys.add(key)
        else:
            keys.discard(key)
            if not keys and g in started:
                segs.append({'gesture': g, 't0_ns': int(started.pop(g)), 't1_ns': int(t)})
    for g, t0 in started.items():           # still held at the end
        segs.append({'gesture': g, 't0_ns': int(t0), 't1_ns': None})
    return sorted(segs, key=lambda s: s['t0_ns'])


def status_intervals(events):
    """failure from onset (operator key "onset" / injected perturbation) until recovery_start;
    recovery from recovery_start until recovery_end. Returns [(status, t0, t1 or None)]."""
    out = []
    onset = rec = None
    for t, kind, v in sorted(events, key=lambda e: e[0]):
        ph = v.get('phase') if kind == 'phase' else None
        if (kind == 'perturbation' or ph == 'onset') and onset is None and rec is None:
            onset = v.get('onset_ns', t) if kind == 'perturbation' else t
        elif ph == 'recovery_start' and rec is None:
            if onset is not None:
                out.append(('failure', onset, t))
                onset = None
            rec = t
        elif ph == 'recovery_end' and rec is not None:
            out.append(('recovery', rec, t))
            rec = None
    if onset is not None:
        out.append(('failure', onset, None))
    if rec is not None:
        out.append(('recovery', rec, None))
    return out


def status_per_step(intervals, t_steps_ns):
    """g_t for each step time: 'nominal' unless inside a failure / recovery interval."""
    flags = ['nominal'] * len(t_steps_ns)
    for status, t0, t1 in intervals:
        for i, t in enumerate(t_steps_ns):
            if t >= t0 and (t1 is None or t < t1):
                flags[i] = status
    return flags


def segment_at(segments, key, t_steps_ns, default=''):
    out = [default] * len(t_steps_ns)
    for s in segments:
        for i, t in enumerate(t_steps_ns):
            if t >= s['t0_ns'] and (s['t1_ns'] is None or t < s['t1_ns']):
                out[i] = s[key]
    return out


# Label resolver (export, release card): which entry wins where several cover the same step.
PRIORITY = {'expert': 3, 'model_assisted': 2, 'trainee': 1, 'automated': 0}
QA_RANK = {'accepted': 2, 'unreviewed': 1, 'flagged': 0}


def usable(entry):
    """QA-rejected entries never reach the dataset."""
    return entry.get('qa_status') != 'rejected'


def _rank(entry):
    # entries without provenance (older files) count as automated
    return PRIORITY.get(entry.get('provenance') or 'automated', 0), QA_RANK.get(entry.get('qa_status'), 1)


def _covers(s, t):
    return t >= s['t0_ns'] and (s.get('t1_ns') is None or t < s['t1_ns'])


def resolve_segments(segments, key, t_steps_ns, default=''):
    """Per step (value, provenance) of the winning usable segment: highest provenance priority
    (expert > model_assisted > trainee > automated), then QA accepted > unreviewed > flagged, then the
    later-starting (more specific) segment. Steps no usable segment covers get (default, '')."""
    vals, srcs, best = [default] * len(t_steps_ns), [''] * len(t_steps_ns), [None] * len(t_steps_ns)
    for order, s in enumerate(segments or []):
        if not usable(s):
            continue
        r = (*_rank(s), s['t0_ns'], order)
        for i, t in enumerate(t_steps_ns):
            if _covers(s, t) and (best[i] is None or r > best[i]):
                best[i], vals[i], srcs[i] = r, s[key], s.get('provenance') or 'automated'
    return vals, srcs


def resolve_gestures(segments, t_steps_ns):
    """Gestures can overlap: per step the set of usable gestures from the highest-priority provenance
    that has any gesture there, and that provenance."""
    sets, srcs = [set() for _ in t_steps_ns], [''] * len(t_steps_ns)
    top = [-1] * len(t_steps_ns)
    for s in segments or []:
        if not usable(s):
            continue
        p = _rank(s)[0]
        for i, t in enumerate(t_steps_ns):
            if _covers(s, t) and p >= top[i]:
                if p > top[i]:
                    sets[i], top[i], srcs[i] = set(), p, s.get('provenance') or 'automated'
                sets[i].add(s['gesture'])
    return ['+'.join(sorted(x)) for x in sets], srcs


def resolve_steps(ann, t_steps_ns):
    """{'status' | 'phase' | 'gesture': (per-step values, per-step label source)} for the export."""
    return {'status': resolve_segments(ann.get('status_intervals'), 'status', t_steps_ns, default='nominal'),
            'phase': resolve_segments(ann.get('phases'), 'phase', t_steps_ns),
            'gesture': resolve_gestures(ann.get('gestures'), t_steps_ns)}


def new_annotation(episode):
    """Skeleton annotation.json for an episode; automated layers are filled by auto_annotate."""
    return {
        'schema_version': SCHEMA_VERSION,
        'episode_id': episode.get('episode_id'),
        'task': label(episode.get('instruction'), 'automated'),
        'phases': [], 'steps': [], 'gestures': [], 'status_intervals': [],
        'failure_class': None, 'motion_quality': None, 'intent': None, 'outcome':
            label(episode.get('outcome'), 'automated') if episode.get('outcome') else None,
        'narration': [], 'segmentation_masks': None,
    }


def validate(ann):
    """Raises ValueError listing every problem."""
    errs = []
    for s in ann.get('phases', []):
        if s.get('phase') not in PHASES:
            errs.append(f'phase {s.get("phase")!r} not in {PHASES}')
    for s in ann.get('gestures', []):
        if s.get('gesture') not in GESTURES:
            errs.append(f'gesture {s.get("gesture")!r} not in {GESTURES}')
    fc = ann.get('failure_class')
    if fc and fc.get('value') not in FAILURE_CLASSES:
        errs.append(f'failure_class {fc.get("value")!r} not in {FAILURE_CLASSES}')
    mq = ann.get('motion_quality')
    if mq and mq.get('value') not in QUALITY_TIERS:
        errs.append('motion_quality must be 1-5')
    for name in ('task', 'failure_class', 'motion_quality', 'intent', 'outcome'):
        v = ann.get(name)
        if v and (v.get('provenance') not in PROVENANCE or v.get('qa_status') not in QA_STATUS):
            errs.append(f'{name}: provenance / qa_status missing or invalid')
    for n in ann.get('narration', []):
        if n.get('provenance') not in PROVENANCE:
            errs.append('narration entry without valid provenance')
    if errs:
        raise ValueError('; '.join(errs))
    return True


def public(ann):
    """Without the merge bookkeeping (keys starting with '_')."""
    if isinstance(ann, dict):
        return {k: public(v) for k, v in ann.items() if not str(k).startswith('_')}
    if isinstance(ann, list):
        return [public(x) for x in ann]
    return ann


def dumps(ann):
    return json.dumps(public(ann), indent=2)


LAYERS = {'phases': ('phase', PHASES), 'gestures': ('gesture', GESTURES), 'status_intervals': ('status', STATUS)}
EPISODE_LABELS = ('task', 'failure_class', 'motion_quality', 'intent', 'outcome')
BASE_FILE = 'annotation.json'
_ANNOTATOR_ID = re.compile(r'^[A-Za-z0-9_-]+$')
# same-role QA disagreement on one entry: the more conservative decision wins
QA_CONFLICT = ['rejected', 'flagged', 'accepted', 'unreviewed']


def annotator_file(ep_dir, annotator_id):
    """<ep>/annotation.<annotator>.json: the only file an annotator writes."""
    if not annotator_id or not _ANNOTATOR_ID.match(str(annotator_id)):
        raise ValueError(f'annotator id {annotator_id!r}: letters, digits, _ and - only')
    return Path(ep_dir) / f'annotation.{annotator_id}.json'


def annotator_files(ep_dir):
    return sorted(p for p in Path(ep_dir).glob('annotation.*.json')
                  if _ANNOTATOR_ID.match(p.name[len('annotation.'):-len('.json')]))


def entry_ref(layer, e):
    """What identifies an entry across files and auto_annotate re-runs (its value, span and author)."""
    if layer == 'narration':
        return {'text': e.get('text'), 't_ns': e.get('t_ns'), 'annotator': e.get('annotator')}
    return {'value': e.get(LAYERS[layer][0]), 't0_ns': e.get('t0_ns'), 't1_ns': e.get('t1_ns'),
            'provenance': e.get('provenance') or 'automated', 'annotator': e.get('annotator')}


def _reviews(e):
    """{annotator: {'qa_status', 'role'}}; a QA status set in place by an older annotator (qa_by) counts too."""
    rv = dict(e.get('qa_reviews') or {})
    if not rv and e.get('qa_by') and e.get('qa_status', 'unreviewed') != 'unreviewed':
        rv[e['qa_by']] = {'qa_status': e['qa_status'], 'role': e.get('qa_role')}
    return rv


def resolve_qa(e):
    """QA status of an entry from every review: the highest-priority reviewer role decides; within that role
    rejected > flagged > accepted. No review: the entry's own status."""
    rv = e.get('qa_reviews') or {}
    if not rv:
        return e.get('qa_status', 'unreviewed')
    top = max(PRIORITY.get(r.get('role'), 0) for r in rv.values())
    return min((r['qa_status'] for r in rv.values() if PRIORITY.get(r.get('role'), 0) == top), key=QA_CONFLICT.index)


def _read(p):
    return json.loads(Path(p).read_text())


def load_merged(ep_dir, only=None, episode=None):
    """annotation.json + every annotation.<annotator>.json, as one annotation dict (what the annotator shows, the
    export resolves and label_agreement rates). only = one annotator id: the blind view (automated entries and that
    annotator's own work; nothing from other raters). Episode labels resolve by provenance priority (expert >
    model_assisted > trainee > automated), then the latest save. `ratings` keeps every annotator's own value."""
    ep_dir = Path(ep_dir)
    bp = ep_dir / BASE_FILE
    ann = _read(bp) if bp.exists() else new_annotation(episode or {})
    if not ann.get('episode_id'):
        ann['episode_id'] = (episode or {}).get('episode_id')

    def visible(e):
        return only is None or (e.get('provenance') or 'automated') == 'automated' or e.get('annotator') == only
    for layer in (*LAYERS, 'narration'):
        ann[layer] = [dict(e, _origin='base') for e in ann.get(layer) or [] if visible(e)]
        if only is not None:
            for e in ann[layer]:
                e['qa_reviews'] = {k: v for k, v in (e.get('qa_reviews') or {}).items() if k == only}
                if e.get('qa_by') not in (None, only):
                    e['qa_status'], e['qa_by'] = 'unreviewed', None
    ratings = {name: {k: v for k, v in r.items() if only is None or k == only}
               for name, r in (ann.get('ratings') or {}).items()}
    cands = {name: [(ann[name], 0)] if ann.get(name) and visible(ann[name]) else [] for name in EPISODE_LABELS}
    files = {}
    for p in annotator_files(ep_dir):
        doc = _read(p)
        who = doc.get('annotator')
        if who and (only is None or who == only):
            files[who] = doc
    for who, doc in files.items():
        saved = max((h.get('saved_ns', 0) for h in doc.get('history') or []), default=0)
        for layer in (*LAYERS, 'narration'):
            ann[layer] += [dict(e, _origin=who) for e in doc.get(layer) or []]
        for name, lab in (doc.get('labels') or {}).items():
            ratings.setdefault(name, {})[who] = None if not lab else {'value': lab['value'], 'role': doc.get('role')}
            if lab:
                cands.setdefault(name, []).append((lab, saved))
    for layer in LAYERS:
        ann[layer].sort(key=lambda s: s['t0_ns'])
    ann['narration'].sort(key=lambda n: n['t_ns'])
    unmatched = []
    for who, doc in files.items():
        for q in doc.get('qa') or []:
            e = next((x for x in ann.get(q.get('layer'), []) if entry_ref(q['layer'], x) == q.get('ref')), None)
            if e is None:                    # the entry changed (e.g. auto_annotate re-run on a new bag)
                unmatched.append(dict(q, annotator=who))
            else:
                e.setdefault('qa_reviews', {})[who] = {'qa_status': q['qa_status'], 'role': doc.get('role')}
    for layer in LAYERS:
        for e in ann[layer]:
            e['qa_reviews'] = _reviews(e)
            e['qa_status'] = resolve_qa(e)
    for name, cs in cands.items():
        ann[name] = max(cs, key=lambda c: (PRIORITY.get(c[0].get('provenance'), 0), c[1]))[0] if cs else None
    ann['ratings'] = ratings
    ann['annotators'] = sorted(files)
    if unmatched:
        ann['qa_unmatched'] = unmatched
    ann['_files'] = files
    return ann


class AnnotationEditor:
    """Edits on behalf of one annotator (role = provenance) on a load_merged() view.

    Segments are [t0_ns, t1_ns) with absolute stamps. Entries of others (automated or other raters) are never
    modified or deleted: corrections are new entries with the annotator's provenance, and any entry can get this
    annotator's QA decision (accepted / flagged / rejected). save() writes only this annotator's file
    (annotator_file()): own entries, own QA decisions, own episode labels, and its history.
    """

    def __init__(self, ann, annotator_id, role):
        if role not in PROVENANCE or role == 'automated':
            raise ValueError(f'role must be one of {[p for p in PROVENANCE if p != "automated"]}')
        if not annotator_id:
            raise ValueError('annotator_id required')
        self.ann = ann
        self.who, self.role = annotator_id, role
        for layer in LAYERS:
            ann.setdefault(layer, [])
        ann.setdefault('narration', [])
        ann.setdefault('history', [])
        self.prev = (ann.get('_files') or {}).get(annotator_id) or {}
        self.own_labels = dict(self.prev.get('labels') or {})
        self.dirty = False

    def add_segment(self, layer, value, t0_ns, t1_ns):
        key, vocab = LAYERS[layer]
        if value not in vocab:
            raise ValueError(f'{value!r} not in {vocab}')
        if t1_ns is not None and t1_ns <= t0_ns:
            raise ValueError('segment end must be after its start')
        seg = {key: value, 't0_ns': int(t0_ns), 't1_ns': None if t1_ns is None else int(t1_ns),
               'provenance': self.role, 'qa_status': 'unreviewed', 'annotator': self.who, '_origin': self.who}
        self.ann[layer].append(seg)
        self.ann[layer].sort(key=lambda s: s['t0_ns'])
        self.dirty = True
        return seg

    def delete_segment(self, layer, index):
        seg = self.ann[layer][index]
        if seg.get('provenance') == 'automated':
            raise ValueError('automated entries are not deleted: set their QA status to rejected')
        if seg.get('_origin', self.who) != self.who:
            raise ValueError(f'entry of {seg.get("annotator") or "annotation.json"} (not yours): '
                             'set its QA status to rejected instead')
        del self.ann[layer][index]
        self.dirty = True

    def set_qa(self, layer, index, qa_status):
        if qa_status not in QA_STATUS:
            raise ValueError(f'qa_status must be one of {QA_STATUS}')
        entry = self.ann[layer][index]
        entry['qa_reviews'] = _reviews(entry)
        entry['qa_reviews'][self.who] = {'qa_status': qa_status, 'role': self.role}
        entry['qa_status'] = resolve_qa(entry)
        entry['qa_by'] = self.who
        self.dirty = True

    def own_value(self, name):
        lab = self.own_labels.get(name)
        return None if not lab else lab.get('value')

    def set_label(self, name, value):
        """Episode-level label: failure_class, motion_quality, intent, outcome, task."""
        if value in (None, ''):
            lab = None
        else:
            extra = {'tier': QUALITY_TIERS[int(value)]} if name == 'motion_quality' else {}
            if name == 'motion_quality':
                value = int(value)
            lab = label(value, self.role, annotator=self.who, **extra)
        self.ann[name] = lab
        self.own_labels[name] = lab
        # every annotator's own value is kept for inter-rater agreement
        self.ann.setdefault('ratings', {}).setdefault(name, {})[self.who] = (
            None if lab is None else {'value': value, 'role': self.role})
        self.dirty = True

    def add_narration(self, t_ns, text):
        if not text.strip():
            raise ValueError('empty narration')
        n = {'t_ns': int(t_ns), 'text': text.strip(), 'provenance': self.role,
             'qa_status': 'unreviewed', 'annotator': self.who, '_origin': self.who}
        self.ann['narration'].append(n)
        self.ann['narration'].sort(key=lambda x: x['t_ns'])
        self.dirty = True
        return n

    def own_document(self, now_ns):
        def own(e):
            return {**{k: v for k, v in e.items() if k not in ('qa_reviews', 'qa_by')}, 'qa_status': 'unreviewed'}
        doc = {'schema_version': SCHEMA_VERSION, 'episode_id': self.ann.get('episode_id'),
               'annotator': self.who, 'role': self.role}
        for layer in (*LAYERS, 'narration'):
            doc[layer] = [own(e) for e in self.ann[layer] if e.get('_origin', self.who) == self.who
                          and e.get('annotator') == self.who]
        doc['qa'] = [{'layer': layer, 'ref': entry_ref(layer, e), 'qa_status': e['qa_reviews'][self.who]['qa_status']}
                     for layer in LAYERS for e in self.ann[layer] if self.who in (e.get('qa_reviews') or {})]
        doc['labels'] = self.own_labels
        doc['history'] = list(self.prev.get('history') or []) + [
            {'annotator': self.who, 'role': self.role, 'saved_ns': int(now_ns)}]
        return doc

    def save(self, path, now_ns):
        """Writes this annotator's file only (path: annotator_file(ep_dir, annotator_id)); validates what it writes."""
        doc = self.own_document(now_ns)
        validate({**doc, **doc['labels']})
        p = Path(path)
        tmp = p.with_suffix('.tmp')
        tmp.write_text(dumps(doc) + '\n')
        tmp.replace(p)
        self.prev = doc
        self.ann.setdefault('_files', {})[self.who] = doc
        self.ann['history'].append(doc['history'][-1])
        self.dirty = False
