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
    ed.save(tmp_path / 'annotation.json', 123)
    import json
    saved = json.loads((tmp_path / 'annotation.json').read_text())
    assert saved['gestures'][0]['qa_status'] == 'rejected' and saved['gestures'][1]['provenance'] == 'expert'
    assert saved['motion_quality'] == {'value': 4, 'provenance': 'expert', 'qa_status': 'unreviewed',
                                       'annotator': 'dr_x', 'tier': 'expert'}
    assert saved['history'] == [{'annotator': 'dr_x', 'role': 'expert', 'saved_ns': 123}]
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
