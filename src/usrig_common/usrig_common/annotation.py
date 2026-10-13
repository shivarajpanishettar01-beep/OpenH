"""Open-H annotation layers for robotic ultrasound (proposal sections 6-7), pure Python.

Layers: phase -> step -> task -> action/gesture, plus motion quality (1-5), intent, outcome,
reasoning/narration, failure class, per-step status flag g_t, and provenance + QA status for
every label. Automatic labels (provenance "automated") are derived from what the rig records:
  * phase segments from /episode/events phase events,
  * gesture segments from teleop key events on /operator/command,
  * g_t (nominal / failure / recovery) from failure-onset and recovery events.
Expert / trainee / model-assisted labels are added later in <ep>/annotation.json.
"""
import json

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
STATUS = ['nominal', 'failure', 'recovery']
PROVENANCE = ['expert', 'trainee', 'model_assisted', 'automated']
QA_STATUS = ['unreviewed', 'flagged', 'accepted', 'rejected']
QUALITY_TIERS = {5: 'expert', 4: 'expert', 3: 'intermediate', 2: 'novice', 1: 'novice'}


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


def dumps(ann):
    return json.dumps(ann, indent=2)


LAYERS = {'phases': ('phase', PHASES), 'gestures': ('gesture', GESTURES), 'status_intervals': ('status', STATUS)}


class AnnotationEditor:
    """Edits one annotation.json on behalf of one annotator (role = provenance).

    Segments are [t0_ns, t1_ns) with absolute stamps. Automated entries are never modified by
    an annotator: corrections are added as new entries with the annotator's provenance, and the
    automated entry can be marked rejected (QA). Every save appends to `history`.
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
        self.dirty = False

    def add_segment(self, layer, value, t0_ns, t1_ns):
        key, vocab = LAYERS[layer]
        if value not in vocab:
            raise ValueError(f'{value!r} not in {vocab}')
        if t1_ns is not None and t1_ns <= t0_ns:
            raise ValueError('segment end must be after its start')
        seg = {key: value, 't0_ns': int(t0_ns), 't1_ns': None if t1_ns is None else int(t1_ns),
               'provenance': self.role, 'qa_status': 'unreviewed', 'annotator': self.who}
        self.ann[layer].append(seg)
        self.ann[layer].sort(key=lambda s: s['t0_ns'])
        self.dirty = True
        return seg

    def delete_segment(self, layer, index):
        seg = self.ann[layer][index]
        if seg.get('provenance') == 'automated':
            raise ValueError('automated entries are not deleted: set their QA status to rejected')
        del self.ann[layer][index]
        self.dirty = True

    def set_qa(self, layer, index, qa_status):
        if qa_status not in QA_STATUS:
            raise ValueError(f'qa_status must be one of {QA_STATUS}')
        entry = self.ann[layer][index]
        entry['qa_status'] = qa_status
        entry['qa_by'] = self.who
        self.dirty = True

    def set_label(self, name, value):
        """Episode-level label: failure_class, motion_quality, intent, outcome, task."""
        if value in (None, ''):
            self.ann[name] = None
        else:
            extra = {'tier': QUALITY_TIERS[int(value)]} if name == 'motion_quality' else {}
            if name == 'motion_quality':
                value = int(value)
            self.ann[name] = label(value, self.role, annotator=self.who, **extra)
        self.dirty = True

    def add_narration(self, t_ns, text):
        if not text.strip():
            raise ValueError('empty narration')
        n = {'t_ns': int(t_ns), 'text': text.strip(), 'provenance': self.role,
             'qa_status': 'unreviewed', 'annotator': self.who}
        self.ann['narration'].append(n)
        self.ann['narration'].sort(key=lambda x: x['t_ns'])
        self.dirty = True
        return n

    def save(self, path, now_ns):
        validate(self.ann)
        self.ann['history'].append({'annotator': self.who, 'role': self.role, 'saved_ns': int(now_ns)})
        from pathlib import Path
        p = Path(path)
        tmp = p.with_suffix('.tmp')
        tmp.write_text(dumps(self.ann) + '\n')
        tmp.replace(p)
        self.dirty = False
