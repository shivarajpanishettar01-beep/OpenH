import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'tools'))
import openh_fields as F  # noqa: E402
import temporal_calib as TC  # noqa: E402


def test_delta_actions_base_and_tool():
    q0 = Rotation.from_euler('z', 0, degrees=True)
    q1 = Rotation.from_euler('z', 10, degrees=True)
    P = np.array([[0, 0, 0, *q0.as_quat()], [0.01, 0, 0, *q1.as_quat()]])
    a = F.delta_actions(P, 'base')
    assert np.allclose(a[0], [0.01, 0, 0, 0, 0, np.radians(10)]) and np.allclose(a[1], 0)
    q0 = Rotation.from_euler('z', 90, degrees=True)
    P = np.array([[0, 0, 0, *q0.as_quat()], [0.01, 0, 0, *q0.as_quat()]])
    assert np.allclose(F.delta_actions(P, 'tool')[0][:3], [0, -0.01, 0])   # base +x is tool -y


def test_step_labels():
    S = 10**9
    ann = {'status_intervals': [{'status': 'failure', 't0_ns': 2 * S, 't1_ns': 3 * S}],
           'phases': [{'phase': 'sweep_scan', 't0_ns': 0, 't1_ns': None}],
           'gestures': [{'gesture': 'slide', 't0_ns': 1 * S, 't1_ns': 4 * S},
                        {'gesture': 'rock', 't0_ns': 2 * S, 't1_ns': None}]}
    st, ph, g = F.step_labels(ann, [0, int(2.5 * S), 5 * S])
    assert st == ['nominal', 'failure', 'nominal'] and ph == ['sweep_scan'] * 3
    assert g == ['', 'rock+slide', 'rock']


def test_step_labels_resolver_drops_rejected_and_prefers_expert():
    """P2-15: QA-rejected entries are ignored; expert > model_assisted > trainee > automated where they overlap,
    whatever the order in the file (auto_annotate appends the automated entries after the human ones)."""
    S = 10**9
    auto = {'provenance': 'automated', 'qa_status': 'unreviewed'}
    ann = {'status_intervals': [{'status': 'failure', 't0_ns': 0, 't1_ns': 10 * S, 'provenance': 'automated',
                                 'qa_status': 'rejected'}],
           'phases': [{'phase': 'sweep_scan', 't0_ns': 0, 't1_ns': 10 * S, 'provenance': 'expert',
                       'qa_status': 'unreviewed', 'annotator': 'dr_a'},
                      {'phase': 'target_localisation', 't0_ns': 0, 't1_ns': None, **auto}],
           'gestures': [{'gesture': 'rock', 't0_ns': 2 * S, 't1_ns': 4 * S, 'provenance': 'trainee',
                         'qa_status': 'unreviewed', 'annotator': 'tr_b'},
                        {'gesture': 'slide', 't0_ns': 0, 't1_ns': None, **auto}]}
    t = [S, 3 * S, 12 * S]
    st, ph, g = F.step_labels(ann, t)
    assert st == ['nominal'] * 3
    assert ph == ['sweep_scan', 'sweep_scan', 'target_localisation']
    assert g == ['slide', 'rock', 'slide']
    src = F.step_label_sources(ann, t)
    assert src == {'status': ['', '', ''], 'phase': ['expert', 'expert', 'automated'],
                   'gesture': ['automated', 'trainee', 'automated']}


def test_failure_class_from_failure_mode_and_missing_class_is_an_error():
    """P2-17: failure / recovery episodes need one of the four classes; the plan FM id gives it automatically,
    a human label wins, and an episode without any is reported (the export then stops)."""
    exp = {'failure_class': {'value': 'observation_degradation', 'provenance': 'expert', 'qa_status': 'unreviewed'}}
    assert F.failure_class({'type': 'recovery-mid', 'failure_mode': 'FM-05'}, {}) == ('target_view_loss', 'automated')
    assert F.failure_class({'type': 'failure', 'failure_mode': 'FM-05'}, exp) == ('observation_degradation', 'expert')
    assert F.failure_class({'type': 'success'}, {}) == ('', '')
    probs = F.failure_class_problems([
        ({'episode_id': 'a', 'type': 'recovery-start', 'failure_mode': None}, {}),
        ({'episode_id': 'b', 'type': 'failure', 'failure_mode': 'FM-SYN'}, {}),
        ({'episode_id': 'c', 'type': 'failure', 'failure_mode': None},
         {'failure_class': {'value': 'probe_slip', 'provenance': 'expert'}}),
        ({'episode_id': 'd', 'type': 'failure', 'failure_mode': 'FM-01'}, {}),
        ({'episode_id': 'e', 'type': 'success'}, {})])
    assert [p.split(':')[0] for p in probs] == ['a', 'b', 'c']


def test_export_checks_dataset_index_count_and_offsets():
    """P2-18: the v2.1 episode order must match the staging order 1:1, and every episode whose measured offsets
    differ from the ones the converter aligned the videos with is an error (not one warning per run)."""
    order = [{'index': 0, 'bag': '/s/ep_1'}, {'index': 1, 'bag': '/s/ep_2'}]
    src = [{'episode_index': 0, 'bag': '/s/ep_1'}, {'episode_index': 1, 'bag': '/s/ep_2'}]
    assert F.index_problems(order, src, [0, 1]) == []
    swapped = [{'episode_index': 0, 'bag': '/s/ep_2'}, {'episode_index': 1, 'bag': '/s/ep_1'}]
    assert len(F.index_problems(order, swapped, [0, 1])) == 2
    assert F.index_problems(order, src[:1], [0])                         # conversion stopped after one bag
    assert F.index_problems(order, src + [{'episode_index': 2, 'bag': '/s/x'}], [0, 1, 2])   # stray episode
    assert F.index_problems(order, src, [0])                             # a parquet file missing
    lr = {'images': {'observation.images.ultrasound': {'time_offset_s': -0.02},
                     'observation.images.wrist_camera': {'time_offset_s': 0.0}},
          'micrus_frame_info': {'time_offset_s': -0.02}}
    used, probs = F.converter_offsets(lr)
    assert used == {'ultrasound': -0.02, 'camera': 0.0} and probs == []
    assert F.converter_offsets({**lr, 'micrus_frame_info': {'time_offset_s': 0.0}})[1]
    assert F.offset_problems({'time_offsets_s': {'ultrasound': -0.02, 'camera': 0.0}}, used) == []
    assert F.offset_problems({'time_offsets_s': {'ultrasound': None, 'camera': None}}, used) == []
    for eid in ('a', 'b'):                                               # every episode, not only the first
        assert len(F.offset_problems({'episode_id': eid, 'time_offsets_s': {'ultrasound': -0.035}}, used)) == 1


def test_calibration_per_session_with_exact_frames():
    """P2-19: each session's calibration snapshot (session.json) is exported for that session, the exporting PC's
    rig.yaml only as a labelled fallback, and the wrist matrix is named T_flange_camera (not end-effector)."""
    I, J = np.eye(4).tolist(), (2 * np.eye(4)).tolist()
    rig_cal = {'wrist_cam_cali_mtx': J, 'prob_cali_mtx': J}
    s1 = F.session_calibration({'calibration': {'wrist_cam_cali_mtx': I, 'prob_cali_mtx': I}}, rig_cal)
    s2 = F.session_calibration({'wrist_cam_cali_mtx': I}, rig_cal)          # older session.json: wrist only
    assert s1['prob_cali_mtx'] == (I, 'session.json:calibration.prob_cali_mtx')
    assert s2['wrist_cali_mtx'] == (I, 'session.json:wrist_cam_cali_mtx')
    assert s2['prob_cali_mtx'] == (J, 'rig.yaml of the exporting PC (no session snapshot)')
    m = F.calibration_meta({'S1': s1, 'S2': s2}, 'prob_cali_mtx')
    assert m['prob_cali_mtx'] is None and m['per_session']['S1']['matrix'] == I     # sessions differ
    w = F.calibration_meta({'S1': s1, 'S2': s2}, 'wrist_cali_mtx')
    assert w['wrist_cali_mtx'] == I and 'T_flange_camera' in w['frame'] and 'end-effector' not in w['frame']


def _staged_session(tmp_path, eps):
    import json
    import yaml
    s = tmp_path / 'S1'
    for name, typ, stage, topics, extra in eps:
        d = s / name
        d.mkdir(parents=True)
        (d / 'episode.json').write_text(json.dumps({'episode_id': name, 'session_id': 'S1', 'type': typ, 'stage': stage,
                                                    'outcome': 'success', 'qc': {'pass': True}, **extra}))
        (d / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {'topics_with_message_count': [
            {'topic_metadata': {'name': t}, 'message_count': 5} for t in topics]}}))
    return s


def test_stage_bags_keeps_stage1_freedrive_and_skips_bags_missing_topics(tmp_path):
    """P2-21: freedrive episodes are kept whatever their stage; a bag without a topic the converter needs is
    skipped and reported instead of stopping bag_to_lerobot for every episode."""
    import argparse
    import stage_bags as SB
    need = ['/micrus/bmode', '/wrist/d415/color/image_raw']
    s = _staged_session(tmp_path, [('ep_0001', 'kinesthetic', 1, need, {}), ('ep_0002', 'success', 2, need[:1], {}),
                                   ('ep_0003', 'success', 2, need, {})])
    ns = argparse.Namespace(include_stage0=False, include_stage1=False, exclude_kinesthetic=False, pipeline_check=False)
    assert sorted(e['episode_id'] for e in SB.collect([s], ns)) == ['ep_0001', 'ep_0002', 'ep_0003']
    skipped = []
    got = SB.collect([s], ns, required=set(need), skipped=skipped)
    assert [e['episode_id'] for e in got] == ['ep_0003', 'ep_0001']
    assert [(x['episode_id'], x['missing']) for x in skipped] == [('ep_0002', ['/wrist/d415/color/image_raw'])]
    assert {'/micrus/bmode', '/wrist/d415/color/image_raw', '/robot/state', '/micrus/frame_info'} <= \
        SB.converter_topics(HERE / 'lerobot_rig.yaml')


def test_png16_is_lossless():
    d = np.random.default_rng(0).integers(0, 65535, (48, 64), dtype=np.uint16)
    assert np.array_equal(F.unpng16(F.png16(d)), d)


def test_sample_hold():
    assert F.sample_hold([10, 30, 20], [5, 10, 25, 99]).tolist() == [-1, 0, 2, 1]


@pytest.mark.parametrize('lag_ms,invert', [(40.0, False), (-25.0, False), (60.0, True)])
def test_temporal_calib_recovers_known_lag(lag_ms, invert):
    f = 0.5
    t_ref = np.arange(0, 20, 0.002)
    x_ref = 0.003 * np.sin(2 * np.pi * f * t_ref + 0.3)
    t_sig = np.arange(0.01, 20, 1 / 27.0)
    x_sig = np.interp(t_sig - lag_ms / 1000, t_ref, x_ref) * (-1 if invert else 1) * 1000 + 50
    lag, corr = TC.xcorr_lag(t_ref, x_ref, t_sig, x_sig, 0.2)
    assert lag * 1000 == pytest.approx(lag_ms, abs=1.5) and corr > 0.99
    assert TC.dominant_freq(t_ref, x_ref) == pytest.approx(f, rel=0.05)
    lag_s, _ = TC.sine_lag(t_ref, x_ref, t_sig, x_sig, f)
    assert lag_s * 1000 == pytest.approx(lag_ms, abs=1.5)
