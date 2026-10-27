"""Writes a synthetic MCAP episode bag and runs the offline QC on it."""
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from usrig_qc.episode_qc import _typestore, run_episode_qc

REPO = Path(__file__).resolve().parents[3]
SCFG = yaml.safe_load((REPO / 'src/usrig_bringup/config/session.yaml').read_text())
RIG = yaml.safe_load((REPO / 'src/usrig_bringup/config/rig.yaml').read_text())
T0 = 1_760_000_000_000_000_000


def write_bag(path, drop_frame=None, duration_s=4.0, drop_robot=(), source=0, mode=3, vx_mm_s=0.0,
              camera=False, camera_stop_s=None, us_stop_s=None, free_air=None, tpv=False, tpv_drop_every=None):
    from rosbags.rosbag2 import StoragePlugin, Writer
    ts = _typestore()
    Image = ts.types['sensor_msgs/msg/Image']
    String = ts.types['std_msgs/msg/String']
    Header = ts.types['std_msgs/msg/Header']
    Time = ts.types['builtin_interfaces/msg/Time']
    Pose = ts.types['geometry_msgs/msg/Pose']
    Point = ts.types['geometry_msgs/msg/Point']
    Quat = ts.types['geometry_msgs/msg/Quaternion']
    RS = ts.types['usrig_interfaces/msg/RobotState']
    EV = ts.types['usrig_interfaces/msg/EpisodeEvent']

    def hdr(ns):
        return Header(stamp=Time(sec=ns // 10**9, nanosec=ns % 10**9), frame_id='base')

    msgs = []
    # ultrasound 27 fps
    n_us = int(duration_s * 27)
    for i in range(n_us):
        if i == drop_frame:
            continue
        ns = T0 + int(i * 1e9 / 27)
        if us_stop_s is not None and ns > T0 + us_stop_s * 1e9:
            break
        img = Image(header=hdr(ns), height=64, width=64, encoding='mono8', is_bigendian=0, step=64,
                    data=np.full(64 * 64, 120, np.uint8))
        fi = {'stamp': ns * 1e-9, 'frame_index': i, 'settings': {'depth': 40, 'lines': 128}}
        msgs += [('/micrus/bmode', ns, img, Image), ('/micrus/frame_info', ns, String(data=json.dumps(fi)), String)]
    # robot 500 Hz, probe pointing down, 4 N into the phantom
    q_down = Quat(x=1.0, y=0.0, z=0.0, w=0.0)
    for k in range(int(duration_s * 500)):
        if k in drop_robot:
            continue
        ns = T0 + k * 2_000_000
        m = RS(header=hdr(ns), controller_time_s=k * 0.002, q=np.zeros(6), qd=np.zeros(6),
               tcp_base=Pose(position=Point(x=vx_mm_s * 1e-3 * k * 0.002, y=0.0, z=0.2), orientation=q_down),
               tcp_phantom=Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=q_down),
               wrench=np.array([0.0, 0.0, -4.0, 0.0, 0.0, 0.0]), force_setpoint_n=4.0, mode=mode,
               safety_flags=0, action_source=source, action_is_expert=source in (0, 1, 5))
        msgs.append(('/robot/state', ns, m, RS))
    fa_events = (((free_air[1], 'phase', {'phase': 'free_air_end'}),) if free_air else ())
    for t, kind, val in fa_events + ((0.05, 'phase', {'phase': 'free_air'}), (0.6, 'phase', {'phase': 'contact'}),
                         (0.7, 'start', {}), (1.0, 'phase', {'phase': 'search'}),
                         (1.5, 'phase', {'phase': 'onset'}), (2.0, 'phase', {'phase': 'recovery_start'}),
                         (2.5, 'phase', {'phase': 'recovery_end'}), (2.6, 'phase', {'phase': 'hold'}),
                         (3.8, 'end', {}), (3.8, 'outcome', {'outcome': 'success'})):
        ns = T0 + int(t * 1e9)
        msgs.append(('/episode/events', ns, EV(header=hdr(ns), episode_id='e1', kind=kind, value=json.dumps(val)), EV))
    for t, key, down in ((1.0, 'W', True), (1.8, 'W', False), (2.1, 'T', True), (2.3, 'T', False)):
        ns = T0 + int(t * 1e9)
        cmd = {'cmd': 'key', 'key': key, 'down': down, 't_ns': ns}
        msgs.append(('/operator/command', ns, String(data=json.dumps(cmd)), String))
    usb = {'speed_mbps': 5000, 'reconnects': 0} if camera else None
    for k in range(int(duration_s * 2)):
        ns = T0 + int(k * 0.5e9)
        msgs.append(('/qc/status', ns, String(data=json.dumps({'wrist_usb': usb})), String))
    if camera:
        # D415 colour + aligned depth at 30 Hz over the whole bag (it records before start and after end)
        ct = SCFG['wrist_camera']['topics']
        for k in range(int(duration_s * 30)):
            ns = T0 + int(k * 1e9 / 30)
            if camera_stop_s is not None and ns > T0 + camera_stop_s * 1e9:
                break
            col = Image(header=hdr(ns), height=4, width=4, encoding='rgb8', is_bigendian=0, step=12,
                        data=np.full(48, 100, np.uint8))
            dep = Image(header=hdr(ns), height=4, width=4, encoding='16UC1', is_bigendian=0, step=8,
                        data=np.full(32, 7, np.uint8))
            msgs += [(ct['color'], ns, col, Image), (ct['depth'], ns, dep, Image)]
    if tpv:
        # TPV colour at 30 Hz (colour only); tpv_drop_every: every n-th frame missing
        for k in range(int(duration_s * 30)):
            if tpv_drop_every and k % tpv_drop_every == 0:
                continue
            ns = T0 + int(k * 1e9 / 30)
            col = Image(header=hdr(ns), height=4, width=4, encoding='rgb8', is_bigendian=0, step=12,
                        data=np.full(48, 90, np.uint8))
            msgs.append((SCFG['tpv_camera']['topics']['color'], ns, col, Image))

    with Writer(path, version=8, storage_plugin=StoragePlugin.MCAP) as w:
        conns = {}
        for topic, _, _, cls in msgs:
            if topic not in conns:
                conns[topic] = w.add_connection(topic, cls.__msgtype__, typestore=ts)
        for topic, ns, m, cls in sorted(msgs, key=lambda x: x[1]):
            w.write(conns[topic], ns, ts.serialize_cdr(m, cls.__msgtype__))


def config(min_free=0):
    scfg = json.loads(json.dumps(SCFG))
    scfg['wrist_camera']['enabled'] = False
    scfg['session']['min_free_disk_gb'] = min_free
    rig = json.loads(json.dumps(RIG))
    rig['qc']['coupling_floor'] = 20
    rig['limits']['force_band_n'] = [2.0, 8.0]
    return scfg, rig


EP = {'stage': 0, 'type': 'shakedown', 't_start_ns': T0 + int(0.7e9), 't_end_ns': T0 + int(3.8e9)}


def test_clean_episode_passes(tmp_path):
    write_bag(tmp_path / 'ep_0001')
    r = run_episode_qc(tmp_path / 'ep_0001', EP, *config())
    assert r['pass'], r['failed']
    c = r['checks']
    assert c['micrus_frame_gaps']['gaps'] == 0
    assert 26 <= c['micrus_fps']['fps'] <= 29
    assert c['robot_rate']['hz'] == pytest.approx(500, rel=0.01)
    assert c['us_robot_stamp_gap']['max_gap_ms'] <= 2.0
    assert c['force_band']['frac_in_band'] == 1.0
    assert c['failure_labels']['status'] == 'n/a'
    assert (tmp_path / 'ep_0001' / 'qc.json').exists()


def test_dropped_frame_fails(tmp_path):
    write_bag(tmp_path / 'ep_0002', drop_frame=40)
    r = run_episode_qc(tmp_path / 'ep_0002', EP, *config())
    assert not r['pass']
    assert r['checks']['micrus_frame_gaps']['gaps'] == 1
    assert 'micrus_frame_gaps' in r['failed']


def test_null_thresholds_are_unset_not_pass(tmp_path):
    write_bag(tmp_path / 'ep_0003')
    scfg, rig = config()
    rig['qc']['coupling_floor'] = None
    r = run_episode_qc(tmp_path / 'ep_0003', EP, scfg, rig)
    assert r['checks']['coupling']['status'] == 'unset' and not r['pass']


def test_auto_annotate_from_bag(tmp_path):
    from usrig_qc.auto_annotate import auto_annotate
    write_bag(tmp_path / 'ep_0004')
    keymap = yaml.safe_load((REPO / 'src/usrig_bringup/config/keymap.yaml').read_text())
    ep = dict(EP, episode_id='e4', instruction='centre the vessel', outcome='success')
    # a human label already in the file must survive re-running the automated pass
    (tmp_path / 'ep_0004' / 'annotation.json').write_text(json.dumps(
        {'phases': [], 'gestures': [{'gesture': 'rock', 't0_ns': 1, 't1_ns': 2, 'provenance': 'expert',
                                     'qa_status': 'accepted'}], 'status_intervals': [], 'narration': []}))
    ann = auto_annotate(tmp_path / 'ep_0004', ep, keymap)
    S = 10**9
    assert [(g['gesture'], g['provenance']) for g in ann['gestures']] == [
        ('rock', 'expert'), ('slide', 'automated'), ('compress', 'automated')]
    st = [(x['status'], (x['t0_ns'] - T0) / S, (x['t1_ns'] - T0) / S) for x in ann['status_intervals']]
    assert st == [('failure', 1.5, 2.0), ('recovery', 2.0, 2.5)]
    assert [p['phase'] for p in ann['phases']] == ['approach_and_contact', 'target_localisation',
                                                   'reacquisition_recovery', 'view_acquisition']
    assert ann['outcome']['value'] == 'success' and ann['task']['value'] == 'centre the vessel'


def test_auto_annotate_failure_class_from_plan_failure_mode(tmp_path):
    """P2-17: a failure / recovery episode gets its failure class from the plan's FM id (automated)."""
    from usrig_qc.auto_annotate import auto_annotate
    write_bag(tmp_path / 'ep_0010')
    keymap = yaml.safe_load((REPO / 'src/usrig_bringup/config/keymap.yaml').read_text())
    ep = dict(EP, episode_id='e10', instruction='x', type='recovery-mid', failure_mode='FM-05')
    ann = auto_annotate(tmp_path / 'ep_0010', ep, keymap)
    assert ann['failure_class']['value'] == 'target_view_loss' and ann['failure_class']['provenance'] == 'automated'
    # re-run after the episode was corrected to a success: the automated class goes away
    ann = auto_annotate(tmp_path / 'ep_0010', dict(ep, type='success', failure_mode=None), keymap)
    assert ann['failure_class'] is None


def test_lost_robot_samples_fail(tmp_path):
    write_bag(tmp_path / 'ep_0005', drop_robot=(800, 801, 802, 1500))
    r = run_episode_qc(tmp_path / 'ep_0005', EP, *config())
    c = r['checks']['robot_sample_gaps']
    assert c['status'] == 'fail' and c['gaps'] == 2 and c['samples_lost'] == 4


FD = dict(EP, type='kinesthetic', collection_mode='freedrive', grip='wrist_handle', force_label_valid=True)


def freedrive_config():
    scfg, rig = config()
    rig['limits']['max_linear_speed_mm_s'] = 50.0
    return scfg, rig


def test_freedrive_episode_passes(tmp_path):
    write_bag(tmp_path / 'ep_0006', source=5, mode=7, vx_mm_s=20.0)
    r = run_episode_qc(tmp_path / 'ep_0006', FD, *freedrive_config())
    assert r['pass'], r['failed']
    c = r['checks']
    assert c['freedrive_engaged']['share'] == 1.0 and c['expert_flag']['status'] == 'pass'
    assert c['freedrive_speed']['max_mm_s'] == pytest.approx(20.0, rel=0.02)


def test_freedrive_pedal_never_used_or_too_fast_fails(tmp_path):
    write_bag(tmp_path / 'ep_0007', source=4, mode=0)
    r = run_episode_qc(tmp_path / 'ep_0007', FD, *freedrive_config())
    assert 'freedrive_engaged' in r['failed']
    write_bag(tmp_path / 'ep_0008', source=5, mode=7, vx_mm_s=80.0)
    r = run_episode_qc(tmp_path / 'ep_0008', FD, *freedrive_config())
    assert 'freedrive_speed' in r['failed'] and r['checks']['freedrive_speed']['max_mm_s'] > 75


def test_grip_on_probe_makes_force_band_not_applicable(tmp_path):
    write_bag(tmp_path / 'ep_0009', source=5, mode=7)
    r = run_episode_qc(tmp_path / 'ep_0009', dict(FD, grip='probe', force_label_valid=False), *freedrive_config())
    assert r['checks']['force_band']['status'] == 'n/a'


def test_rig_status_with_nothing_configured():
    from usrig_qc.rig_status import CONNECT, SET, rows
    km = yaml.safe_load((REPO / 'src/usrig_bringup/config/keymap.yaml').read_text())
    table = rows(SCFG, RIG, km)
    status = {item: st for _, item, st, _ in table}
    assert status['robot.ip'] == SET                       # null ip: no network probe, a SET row
    assert status['rig.yaml limits (robot_control)'] == SET
    assert all(st in ('ok', SET, CONNECT, 'info') for _, _, st, _ in table)


def test_collection_plan_capacity_and_schedule():
    import datetime as dt
    from usrig_qc.collection_plan import available_days, plan
    days = available_days(dt.date(2026, 10, 19), dt.date(2026, 11, 2), {'mon', 'tue', 'wed', 'thu', 'fri'},
                          {dt.date(2026, 10, 22)})
    assert len(days) == 9 and dt.date(2026, 10, 22) not in days            # 2 weeks of weekdays minus 1
    prog = {'phantoms': {'a': {'hours': 0.5}, 'b': {'hours': 0.0}}}
    r = plan(prog, {'a': 1.5, 'b': 1.0}, days, 2.5, 0.8, 0.4, 25.0, 0.2)
    assert abs(r['remaining_recorded_h']['a'] - 1.0) < 1e-9
    assert abs(r['rig_hours_total'] - 2.0 / 0.32) < 1e-6 and r['on_track']   # 6.25 rig h of 22.5
    assert abs(sum(s['recorded_h'] for s in r['schedule']) - 2.0) < 1e-6
    assert {s['phantom'] for s in r['schedule']} == {'a', 'b'} and not r['unscheduled_recorded_h']
    short = plan(prog, {'a': 20.0, 'b': 0.0}, days[:2], 2.5, 0.8, 0.4, 25.0, 0.2)
    assert not short['on_track'] and short['unscheduled_recorded_h']['a'] > 0


def test_collection_plan_measures_rates_from_real_episodes(tmp_path):
    from usrig_qc.collection_plan import measured_rates
    s = tmp_path / 'sessions' / 'S1'
    S = 10**9
    for i in range(12):
        d = s / f'ep_{i:04d}'
        d.mkdir(parents=True)
        kept = i % 4 != 0                                            # 9 of 12 kept
        d.joinpath('episode.json').write_text(json.dumps({
            'outcome': 'success' if kept else 'discard', 'qc': {'pass': kept},
            't_start_ns': i * 100 * S, 't_end_ns': i * 100 * S + 25 * S}))
    y, rec, n = measured_rates(tmp_path)
    assert n == 12 and abs(y - 0.75) < 1e-9
    assert abs(rec - 9 * 25 / (11 * 100 + 25)) < 1e-9


def test_label_agreement_two_raters(tmp_path, capsys):
    from usrig_common import annotation as A
    from usrig_qc.label_agreement import main
    S = 10**9
    for i, (bob_onset, q_alice, q_bob) in enumerate(((2.0, 4, 4), (2.5, 3, 4), (2.0, 5, 5))):
        d = tmp_path / 'S1' / f'ep_{i:04d}'
        d.mkdir(parents=True)
        (d / 'episode.json').write_text(json.dumps({'t_start_ns': 0, 't_end_ns': 10 * S}))
        ann = A.new_annotation({'episode_id': f'e{i}'})
        a, b = A.AnnotationEditor(ann, 'alice', 'expert'), A.AnnotationEditor(ann, 'bob', 'expert')
        a.add_segment('status_intervals', 'failure', 2 * S, 5 * S)
        b.add_segment('status_intervals', 'failure', int(bob_onset * S), 5 * S)
        a.set_label('motion_quality', q_alice)
        b.set_label('motion_quality', q_bob)
        if i == 0:                                 # an automated label never counts
            ann['status_intervals'].append({'status': 'recovery', 't0_ns': 0, 't1_ns': S, 'provenance': 'automated'})
        (d / 'annotation.json').write_text(A.dumps(ann))
    main([str(tmp_path / 'S1'), '--json', str(tmp_path / 'a.json')])
    res = {r['label']: r for r in json.loads((tmp_path / 'a.json').read_text())}
    st = res['status_intervals']
    assert st['episodes'] == 3 and abs(st['percent_agreement'] - (300 - 5) / 300) < 1e-9 and st['kappa'] > 0.9
    assert res['motion_quality']['episodes'] == 3 and 0 < res['motion_quality']['kappa'] < 1
    assert res['phases']['episodes'] == 0 and res['phases']['kappa'] is None


def camera_config():
    scfg, rig = config()
    scfg['wrist_camera']['enabled'] = True
    scfg['recording']['include_wrist_camera'] = True
    return scfg, rig


def test_camera_rate_judged_inside_the_episode_window(tmp_path):
    """The bag records before start (readiness, free-air hold, descent) and after end: depth was counted over
    the whole bag but divided by the window, so every camera episode under ~35 s failed wrist_rate."""
    write_bag(tmp_path / 'ep_c1', camera=True)
    r = run_episode_qc(tmp_path / 'ep_c1', EP, *camera_config())
    c = r['checks']['wrist_rate']
    assert c['status'] == 'pass', c
    assert c['hz']['depth'] == pytest.approx(30, rel=0.02) and c['hz']['color'] == pytest.approx(30, rel=0.02)


def test_stream_that_stops_mid_episode_fails(tmp_path):
    """A camera or scanner that stalls part way used to pass: rates were taken between the samples present."""
    write_bag(tmp_path / 'ep_c2', camera=True, camera_stop_s=2.5)        # window 0.7 .. 3.8 s
    r = run_episode_qc(tmp_path / 'ep_c2', EP, *camera_config())
    assert r['checks']['stream_coverage']['status'] == 'fail'
    assert 'wrist.color' in r['checks']['stream_coverage']['short']
    write_bag(tmp_path / 'ep_c3', us_stop_s=3.0)
    r = run_episode_qc(tmp_path / 'ep_c3', EP, *config())
    assert r['checks']['stream_coverage']['status'] == 'fail' and 'ultrasound' in r['checks']['stream_coverage']['short']
    write_bag(tmp_path / 'ep_c4', camera=True)
    r = run_episode_qc(tmp_path / 'ep_c4', EP, *camera_config())
    assert r['checks']['stream_coverage']['status'] == 'pass', r['checks']['stream_coverage']


def test_bag_recorder_error_fails(tmp_path):
    write_bag(tmp_path / 'ep_c5')
    r = run_episode_qc(tmp_path / 'ep_c5', dict(EP, bag_returncode=1), *config())
    assert r['checks']['bag_closed']['status'] == 'fail' and not r['pass']
    r = run_episode_qc(tmp_path / 'ep_c5', dict(EP, bag_returncode=0), *config())
    assert r['checks']['bag_closed']['status'] == 'pass'


def test_low_disk_does_not_fail_a_recorded_episode(tmp_path):
    """Free disk was a QC check on the already-recorded episode: below the floor good episodes were discarded."""
    write_bag(tmp_path / 'ep_d1')
    r = run_episode_qc(tmp_path / 'ep_d1', EP, *config(min_free=10**9))
    assert r['pass'], r['failed']
    assert r['checks']['free_disk']['status'] == 'n/a' and r['checks']['free_disk']['free_gb'] > 0


def test_label_agreement_counts_an_accepted_automated_segment(tmp_path):
    """P2-16: a rater who accepts the automated segment has rated it; that is agreement data."""
    from usrig_common import annotation as A
    from usrig_qc.label_agreement import main
    S = 10**9
    d = tmp_path / 'S1' / 'ep_0000'
    d.mkdir(parents=True)
    (d / 'episode.json').write_text(json.dumps({'t_start_ns': 0, 't_end_ns': 10 * S}))
    ann = A.new_annotation({'episode_id': 'e0'})
    ann['status_intervals'] = [{'status': 'failure', 't0_ns': 2 * S, 't1_ns': 5 * S, 'provenance': 'automated',
                                'qa_status': 'unreviewed'}]
    a = A.AnnotationEditor(ann, 'alice', 'expert')
    a.add_segment('status_intervals', 'failure', 2 * S, 5 * S)
    b = A.AnnotationEditor(ann, 'bob', 'expert')
    b.set_qa('status_intervals', 0, 'accepted')       # bob accepts the automated interval (index 0)
    (d / 'annotation.json').write_text(A.dumps(ann))
    main([str(tmp_path / 'S1'), '--json', str(tmp_path / 'a.json')])
    res = {r['label']: r for r in json.loads((tmp_path / 'a.json').read_text())}
    assert res['status_intervals']['episodes'] == 1 and res['status_intervals']['percent_agreement'] == 1.0


def test_pick_for_double_rating_covers_every_type():
    from usrig_qc.label_agreement import pick_for_double_rating
    cov = [{'dir': f'e{i}', 'type': t, 'sim': False, 'raters': ['alice']}
           for i, t in enumerate(['success'] * 16 + ['failure'] * 2 + ['recovery-mid'] * 2)]
    cov.append({'dir': 'done', 'type': 'success', 'sim': False, 'raters': ['alice', 'bob']})
    pick = pick_for_double_rating(cov, 0.15)            # 15 % of 21 = 3, one already double-rated
    assert len(pick) == 2 and {c['type'] for c in pick} <= {'success', 'failure', 'recovery-mid'}
    assert len({c['type'] for c in pick_for_double_rating(cov, 0.2)}) == 3
    assert pick == pick_for_double_rating(cov, 0.15)    # seeded


def test_rerun_qc_over_a_session_updates_episode_json(tmp_path, monkeypatch):
    """Episodes recorded before the QC thresholds were set (unset = fail) must be re-checkable later; the CLI
    wrote qc.json only, while progress and the export read episode.json."""
    import sys as _sys
    from usrig_qc import episode_qc as EQ
    sdir = tmp_path / 'S1'
    for name, outcome in (('ep_0001', 'success'), ('ep_0002', 'discard')):
        write_bag(sdir / name)
        (sdir / name / 'episode.json').write_text(json.dumps(dict(EP, outcome=outcome, qc={'pass': False, 'old': 1})))
    scfg, rig = config()
    monkeypatch.setattr(EQ.config, 'load_config', lambda name, d=None: scfg if name == 'session' else rig)
    monkeypatch.setattr(_sys, 'argv', ['episode_qc', str(sdir), '--update'])
    with pytest.raises(SystemExit):
        EQ.main()
    e1 = json.loads((sdir / 'ep_0001' / 'episode.json').read_text())
    assert e1['qc']['pass'] is True and e1['qc_previous'] == {'pass': False, 'old': 1}
    e2 = json.loads((sdir / 'ep_0002' / 'episode.json').read_text())
    assert e2['qc'] == {'pass': False, 'old': 1}                     # discards are left alone


def test_episode_length_is_reported_against_the_target(tmp_path):
    """The proposal's 20-30 s episodes were not visible anywhere; reported (information, not a fail)."""
    write_bag(tmp_path / 'ep_l1')
    scfg, rig = config()
    r = run_episode_qc(tmp_path / 'ep_l1', EP, scfg, rig)                 # 3.1 s window
    c = r['checks']['episode_length']
    assert c['status'] == 'n/a' and c['within_target'] is False and c['target_s'] == [20, 30]
    assert c['recorded_s'] == pytest.approx(3.1, abs=0.01) and r['pass']


def test_free_air_offset_is_measured_per_episode(tmp_path):
    """The free-air window records the F/T offset (freedrive: nothing zeroes the sensor) but nothing measured it."""
    write_bag(tmp_path / 'ep_f1', free_air=(0.05, 0.6))
    scfg, rig = config()
    c = run_episode_qc(tmp_path / 'ep_f1', EP, scfg, rig)['checks']['free_air_window']
    assert c['status'] == 'pass' and c['samples'] > 200
    assert c['wrench_mean'][2] == pytest.approx(-4.0) and c['wrench_std'][2] == pytest.approx(0.0)


def test_arm_moving_while_labelled_idle_is_flagged(tmp_path):
    """Freedrive by the pendant button (not the pedal input) is recorded as IDLE / none / not expert: the
    demonstration would be lost from the expert data without any warning."""
    write_bag(tmp_path / 'ep_m1', mode=0, source=4, vx_mm_s=50.0)        # moving, labelled idle
    r = run_episode_qc(tmp_path / 'ep_m1', EP, *config())
    assert r['checks']['unlabelled_motion']['status'] == 'fail'
    write_bag(tmp_path / 'ep_m2', mode=7, source=5, vx_mm_s=50.0)        # moving, labelled freedrive
    assert run_episode_qc(tmp_path / 'ep_m2', EP, *config())['checks']['unlabelled_motion']['status'] == 'pass'
    write_bag(tmp_path / 'ep_m3', mode=0, source=4)                       # still, idle
    assert run_episode_qc(tmp_path / 'ep_m3', EP, *config())['checks']['unlabelled_motion']['status'] == 'pass'


def test_tpv_camera_rate_and_coverage(tmp_path):
    """The third-person view camera (colour only) is checked like the wrist colour: rate in the window and
    coverage; not recorded -> n/a, so wrist-only sessions are unchanged."""
    scfg, rig = config()
    assert run_episode_qc(write_tpv(tmp_path / 'ep_t0', tpv=False), EP, scfg, rig)['checks']['tpv_rate']['status'] == 'n/a'
    scfg['tpv_camera']['enabled'] = True
    r = run_episode_qc(write_tpv(tmp_path / 'ep_t1'), EP, scfg, rig)['checks']
    assert r['tpv_rate']['status'] == 'pass' and r['tpv_rate']['hz'] == pytest.approx(30, rel=0.02), r['tpv_rate']
    assert r['stream_coverage']['status'] == 'pass', r['stream_coverage']
    r = run_episode_qc(write_tpv(tmp_path / 'ep_t2', tpv_drop_every=4), EP, scfg, rig)['checks']
    assert r['tpv_rate']['status'] == 'fail'
    r = run_episode_qc(write_tpv(tmp_path / 'ep_t3', tpv=False), EP, scfg, rig)['checks']   # enabled, nothing recorded
    assert r['tpv_rate']['status'] == 'fail' and 'tpv.color' in r['stream_coverage']['short']


def write_tpv(path, tpv=True, tpv_drop_every=None):
    write_bag(path, tpv=tpv, tpv_drop_every=tpv_drop_every)
    return path
