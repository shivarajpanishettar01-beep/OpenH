import pytest

from usrig_common import annotation as A

KEYMAP = {'motion': {'W': 'vx:+1', 'S': 'vx:-1', 'R': 'wx:+1', 'Q': 'wy:+1', 'Z': 'wz:+1'},
          'commands': {'T': 'force_up', 'G': 'force_down', 'O': 'onset'}}
S = 1_000_000_000


def test_phase_segments_map_rig_events_to_proposal_phases():
    ev = [(0, 'phase', {'phase': 'free_air'}), (1 * S, 'phase', {'phase': 'contact'}),
          (2 * S, 'phase', {'phase': 'search'}), (5 * S, 'phase', {'phase': 'hold'}),
          (6 * S, 'phase', {'phase': 'fault_ack'})]
    segs = A.phase_segments(ev, 8 * S)
    assert [s['phase'] for s in segs] == ['approach_and_contact', 'target_localisation', 'view_acquisition']
    assert segs[0]['t1_ns'] == 2 * S and segs[-1]['t1_ns'] == 8 * S


def test_gestures_from_keys_including_compress_and_overlap():
    keys = [(1 * S, 'W', True), (2 * S, 'R', True), (3 * S, 'W', False), (4 * S, 'R', False),
            (5 * S, 'T', True), (6 * S, 'T', False), (7 * S, 'S', True)]
    g = A.gesture_segments(keys, KEYMAP)
    assert [(x['gesture'], x['t0_ns'], x['t1_ns']) for x in g] == [
        ('slide', 1 * S, 3 * S), ('tilt', 2 * S, 4 * S), ('compress', 5 * S, 6 * S), ('slide', 7 * S, None)]


def test_status_flags_failure_then_recovery():
    ev = [(2 * S, 'perturbation', {'onset_ns': 2 * S + 5}), (4 * S, 'phase', {'phase': 'recovery_start'}),
          (6 * S, 'phase', {'phase': 'recovery_end'})]
    iv = A.status_intervals(ev)
    assert iv == [('failure', 2 * S + 5, 4 * S), ('recovery', 4 * S, 6 * S)]
    flags = A.status_per_step(iv, [1 * S, 3 * S, 5 * S, 7 * S])
    assert flags == ['nominal', 'failure', 'recovery', 'nominal']


def test_operator_onset_without_recovery_stays_failure():
    iv = A.status_intervals([(3 * S, 'phase', {'phase': 'onset'})])
    assert A.status_per_step(iv, [2 * S, 4 * S, 9 * S]) == ['nominal', 'failure', 'failure']


def test_validate_rejects_bad_labels():
    ann = A.new_annotation({'episode_id': 'e', 'instruction': 'find the vessel'})
    A.validate(ann)
    ann['failure_class'] = A.label('lost', 'expert')
    ann['motion_quality'] = A.label(7, 'trainee')
    with pytest.raises(ValueError, match='failure_class.*motion_quality'):
        A.validate(ann)
    with pytest.raises(ValueError):
        A.label('x', 'robot')


def test_editor_rules(tmp_path):
    ann = A.new_annotation({'episode_id': 'e', 'instruction': 'x'})
    ann['gestures'] = [{'gesture': 'slide', 't0_ns': 0, 't1_ns': 5, 'provenance': 'automated',
                        'qa_status': 'unreviewed'}]
    with pytest.raises(ValueError):
        A.AnnotationEditor(ann, 'dr_x', 'automated')
    ed = A.AnnotationEditor(ann, 'dr_x', 'expert')
    with pytest.raises(ValueError, match='automated'):
        ed.delete_segment('gestures', 0)
    ed.set_qa('gestures', 0, 'rejected')
    ed.add_segment('gestures', 'rock', 1, 4)
    with pytest.raises(ValueError):
        ed.add_segment('phases', 'not_a_phase', 0, 1)
    with pytest.raises(ValueError, match='after'):
        ed.add_segment('status_intervals', 'failure', 5, 5)
    ed.set_label('failure_class', 'target_view_loss')
    ed.set_label('motion_quality', 4)
    ed.add_narration(2, 'target near centre; slide left to reacquire')
    ed.save(A.annotator_file(tmp_path, 'dr_x'), 123)
    import json
    saved = json.loads(A.annotator_file(tmp_path, 'dr_x').read_text())    # only dr_x's own work
    assert [(g['gesture'], g['provenance']) for g in saved['gestures']] == [('rock', 'expert')]
    assert saved['qa'] == [{'layer': 'gestures', 'qa_status': 'rejected', 'ref': {
        'value': 'slide', 't0_ns': 0, 't1_ns': 5, 'provenance': 'automated', 'annotator': None}}]
    assert saved['labels']['motion_quality'] == {'value': 4, 'provenance': 'expert', 'qa_status': 'unreviewed',
                                                 'annotator': 'dr_x', 'tier': 'expert'}
    assert saved['history'] == [{'annotator': 'dr_x', 'role': 'expert', 'saved_ns': 123}]
    (tmp_path / 'annotation.json').write_text(json.dumps({'gestures': [ann['gestures'][0] | {'qa_status': 'unreviewed'}]}))
    m = A.load_merged(tmp_path)
    assert m['gestures'][0]['qa_status'] == 'rejected' and m['motion_quality']['value'] == 4
    ed.delete_segment('gestures', 1)
    assert len(ann['gestures']) == 1


def test_agreement_statistics_known_values():
    from usrig_common.agreement import cohen_kappa, fleiss_kappa, landis_koch, percent_agreement
    a, b = [1, 1, 0, 0], [1, 0, 0, 0]
    assert percent_agreement(a, b) == 0.75 and abs(cohen_kappa(a, b) - 0.5) < 1e-12
    assert cohen_kappa([1, 2, 3], [1, 2, 3]) == 1.0
    # weighted: a near miss on an ordinal scale costs less than a far miss
    near = cohen_kappa([1, 2, 3, 4, 5], [1, 2, 3, 4, 4], weights='quadratic')
    far = cohen_kappa([1, 2, 3, 4, 5], [1, 2, 3, 4, 1], weights='quadratic')
    assert near > far
    # Fleiss (1971) style check: perfect agreement -> 1; two raters equals Cohen-like chance correction
    assert fleiss_kappa([['x', 'x', 'x'], ['y', 'y', 'y']]) == 1.0
    assert abs(fleiss_kappa([['a', 'a'], ['a', 'b'], ['b', 'b'], ['b', 'b']]) - 0.4666666) < 1e-6
    assert landis_koch(0.5) == 'moderate' and landis_koch(0.9) == 'almost perfect'


def test_editor_keeps_every_annotators_episode_label():
    ann = A.new_annotation({'episode_id': 'e1', 'instruction': 'x'})
    A.AnnotationEditor(ann, 'alice', 'expert').set_label('motion_quality', 4)
    A.AnnotationEditor(ann, 'bob', 'expert').set_label('motion_quality', 3)
    assert ann['motion_quality']['value'] == 3 and ann['motion_quality']['annotator'] == 'bob'
    assert {k: v['value'] for k, v in ann['ratings']['motion_quality'].items()} == {'alice': 4, 'bob': 3}


def test_concurrent_annotators_write_their_own_files(tmp_path):
    """P2-16: two raters with the same episode open must not overwrite each other; annotation.json
    (automated, written by auto_annotate) is never rewritten by the annotator; blind view hides others."""
    import json
    ep = tmp_path / 'ep_0001'
    ep.mkdir()
    base = A.new_annotation({'episode_id': 'e1', 'instruction': 'x'})
    base['gestures'] = [{'gesture': 'slide', 't0_ns': 0, 't1_ns': 5, 'provenance': 'automated',
                         'qa_status': 'unreviewed'}]
    (ep / 'annotation.json').write_text(A.dumps(base))
    a = A.AnnotationEditor(A.load_merged(ep), 'alice', 'expert')        # both open the episode
    b = A.AnnotationEditor(A.load_merged(ep), 'bob', 'trainee')
    a.add_segment('phases', 'sweep_scan', 0, 5)
    a.set_label('failure_class', 'target_view_loss')
    a.save(A.annotator_file(ep, 'alice'), 1)
    b.add_segment('phases', 'view_acquisition', 0, 5)
    b.set_qa('gestures', 0, 'accepted')
    b.set_label('failure_class', 'observation_degradation')
    b.save(A.annotator_file(ep, 'bob'), 2)
    assert json.loads((ep / 'annotation.json').read_text()) == json.loads(A.dumps(base))
    m = A.load_merged(ep)
    assert sorted((s['phase'], s['annotator']) for s in m['phases']) == [('sweep_scan', 'alice'),
                                                                         ('view_acquisition', 'bob')]
    assert m['gestures'][0]['qa_status'] == 'accepted' and m['gestures'][0]['qa_reviews']['bob']['qa_status'] == 'accepted'
    assert m['failure_class']['value'] == 'target_view_loss'           # expert over trainee
    assert {k: v['value'] for k, v in m['ratings']['failure_class'].items()} == {
        'alice': 'target_view_loss', 'bob': 'observation_degradation'}
    # an expert rejection beats a trainee acceptance; alice's earlier work stays in her file
    a2 = A.AnnotationEditor(A.load_merged(ep), 'alice', 'expert')
    a2.set_qa('gestures', 0, 'rejected')
    a2.save(A.annotator_file(ep, 'alice'), 3)
    m = A.load_merged(ep)
    assert m['gestures'][0]['qa_status'] == 'rejected'
    assert [s['phase'] for s in m['phases'] if s['annotator'] == 'alice'] == ['sweep_scan']
    blind = A.load_merged(ep, only='bob')
    assert [s['annotator'] for s in blind['phases']] == ['bob'] and blind['gestures'][0]['qa_status'] == 'accepted'
    assert 'alice' not in (blind.get('ratings') or {}).get('failure_class', {})
    ed = A.AnnotationEditor(A.load_merged(ep), 'bob', 'trainee')
    i = next(i for i, s in enumerate(ed.ann['phases']) if s['annotator'] == 'alice')
    with pytest.raises(ValueError, match='alice'):
        ed.delete_segment('phases', i)
    with pytest.raises(ValueError):
        A.annotator_file(ep, '../evil')
