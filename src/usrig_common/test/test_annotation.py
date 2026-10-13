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
