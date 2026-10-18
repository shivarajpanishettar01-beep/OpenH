"""Tests that need no ROS, no robot and no display (Qt tests use the offscreen platform)."""
import os
import time
from types import SimpleNamespace

import numpy as np
import pytest

from usrig_demo_gui import config, kinematics as kin, metrics as M
from usrig_demo_gui.models import (ContactState, Health, Outcome, PlaneState, RecordingStatus, StreamStatus,
                                   SystemState, TaskPhase)
from usrig_demo_gui.sources.base import rate_health
from usrig_demo_gui.sources.episode_tracker import EpisodeTracker
from usrig_demo_gui.sources.mock import SCENARIOS, MockSource

CFG = config.load()


def at(src, t):
    """Snapshot the mock as if t seconds had passed (two calls so the F/T buffer fills)."""
    src.t0 = time.monotonic() - t + 0.05
    src.snapshot()
    src.t0 = time.monotonic() - t
    return src.snapshot()


# ------------------------------------------------------------------ kinematics
def test_seed_points_down_in_front():
    T = kin.tcp(kin.Q_SEED)
    assert T[0, 3] > 0.3 and T[2, 2] < -0.99


def test_ik_reaches_target():
    p = np.array([0.42, 0.13, 0.058])
    q = kin.ik_point_down(p, [0, 0, -1], kin.Q_SEED)
    T = kin.tcp(q)
    assert np.allclose(T[:3, 3], p, atol=1e-4)
    assert T[2, 2] < -0.999


def test_rotvec_roundtrip():
    for v in ([0.1, 0.2, 0.3], [np.pi - 1e-3, 0, 0], [0, 0, 0]):
        v = np.array(v)
        th = np.linalg.norm(v)
        if th == 0:
            R = np.eye(3)
        else:
            k = v / th
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K
        assert np.allclose(kin.rotvec_from_matrix(R), v, atol=1e-6)


# ------------------------------------------------------------------ metrics
def test_metrics_none_not_zero():
    assert M.speckle_snr(None, (0, 10, 0, 10)) is None
    assert M.cnr(np.zeros((50, 50), np.uint8), None, (0, 10, 0, 10)) is None
    flat = np.full((50, 50), 100, np.uint8)
    assert M.speckle_snr(flat, (0, 20, 0, 20)) is None        # undefined, not infinite / zero


def test_rate_health():
    assert rate_health(27.0, [26, 29]) == Health.OK
    assert rate_health(18.0, [26, 29]) == Health.WARN
    assert rate_health(None, [26, 29]) == Health.DOWN
    assert rate_health(None, None) == Health.OK
    assert rate_health(30.0, [26, 29], node_up=False) == Health.DOWN


# ------------------------------------------------------------------ mock scenarios
@pytest.mark.parametrize('name', list(SCENARIOS))
def test_every_scenario_runs(name):
    src = MockSource(CFG, name)
    for t in (0.5, 6, 13, 18, 25, 31, 36):
        d = at(src, t)
        assert d.is_mock
        assert len(d.nodes) == len(CFG['nodes'])


def test_normal_episode_sequence():
    src = MockSource(CFG, 'normal')
    assert at(src, 1.0).task.phase == TaskPhase.APPROACH
    d = at(src, 14.0)
    assert d.task.phase == TaskPhase.SWEEP and d.contact == ContactState.IN_CONTACT
    assert d.plane == PlaneState.IN_PLANE and d.task.outcome == Outcome.IN_PROGRESS
    assert d.recording.status == RecordingStatus.RECORDING
    assert at(src, 28.5).task.outcome == Outcome.SUCCESS
    assert d.system == SystemState.RUNNING


def test_lost_contact_is_not_failure():
    src = MockSource(CFG, 'recovery')
    d = at(src, 17.5)
    assert d.contact == ContactState.NO_CONTACT
    assert d.plane == PlaneState.UNKNOWN                   # no image contact -> plane not judged
    assert d.task.phase == TaskPhase.RECOVERY
    assert d.task.outcome == Outcome.IN_PROGRESS
    assert at(src, 29.0).task.outcome == Outcome.SUCCESS


def test_failure_and_aborted_outcomes():
    assert at(MockSource(CFG, 'failure'), 30.5).task.outcome == Outcome.FAILURE
    assert at(MockSource(CFG, 'aborted'), 10.5).task.outcome == Outcome.ABORTED


def test_unavailable_metrics_are_none():
    d = at(MockSource(CFG, 'idle'), 2.0)
    m = {x.name: x.value for x in d.metrics.items}
    assert m['In-plane score'] is None and m['Probe-tissue angle'] is None
    assert m['Contact force'] is not None                  # a small real reading, not "unavailable"
    d = at(MockSource(CFG, 'disconnected'), 2.0)
    assert all(x.value is None for x in d.metrics.items)
    assert d.system == SystemState.DISCONNECTED
    assert d.ultrasound.status == StreamStatus.DISCONNECTED
    assert all(n.health == Health.DOWN for n in d.nodes)


def test_degraded_flags_slow_streams():
    d = at(MockSource(CFG, 'degraded'), 3.0)
    assert d.system == SystemState.DEGRADED
    slow = {n.name for n in d.nodes if n.health == Health.WARN}
    assert {'micrus', 'd415 depth'} <= slow
    assert d.recording.warnings


# ------------------------------------------------------------------ episode events (ROS mapping)
def test_episode_tracker():
    tr = EpisodeTracker(CFG['phase_map'], CFG['outcome_map'])
    ns = 1_000_000_000
    eid = '20261013_vascular_4vein_op1_s01_ep_0003'
    tr.handle('phase', '{"phase": "free_air", "hold_s": 1.5}', 10 * ns, eid)
    assert tr.phase == TaskPhase.APPROACH and tr.outcome == Outcome.IN_PROGRESS
    tr.handle('phase', '{"phase": "free_air_end"}', 12 * ns, eid)
    tr.handle('start', '{"type": "nominal", "instruction": "find the bifurcation"}', 14 * ns, eid)
    tr.handle('phase', '{"phase": "search"}', 14 * ns, eid)
    assert tr.phase == TaskPhase.VIEW_ACQUISITION
    tr.handle('phase', '{"phase": "perturbation"}', 20 * ns, eid)
    assert tr.phase == TaskPhase.RECOVERY and tr.outcome == Outcome.IN_PROGRESS
    tr.handle('end', '{"outcome": "recovered"}', 30 * ns, eid)
    assert tr.outcome == Outcome.SUCCESS and tr.phase is None and not tr.active
    assert tr.recording_status(31 * ns, True) == RecordingStatus.FINALIZING
    assert tr.recording_status(40 * ns, False) == RecordingStatus.IDLE
    assert abs(tr.elapsed_s(99 * ns) - 20.0) < 1e-6
    # free_air_end: the probe is still in the air
    tr2 = EpisodeTracker(CFG['phase_map'], CFG['outcome_map'])
    tr2.handle('phase', '{"phase": "free_air_end"}', 1 * ns, eid)
    assert tr2.phase == TaskPhase.APPROACH


def test_episode_tracker_robot_control_phases():
    """Phase names robot_control publishes (robot_control.py), incl. a path run as the sweep."""
    tr = EpisodeTracker(CFG['phase_map'], CFG['outcome_map'])
    ns = 1_000_000_000
    eid = 's01_ep_0001'
    tr.handle('start', '{}', 1 * ns, eid)
    tr.handle('phase', '{"phase": "approach", "legs": 2}', 2 * ns, eid)
    assert tr.phase == TaskPhase.APPROACH
    tr.handle('phase', '{"phase": "contact", "force_setpoint_n": 3.0}', 4 * ns, eid)
    assert tr.phase == TaskPhase.CONTACT
    tr.handle('phase', '{"phase": "search", "path_length_m": 0.06, "speed_mm_s": 10}', 5 * ns, eid)
    assert tr.phase == TaskPhase.SWEEP
    tr.handle('phase', '{"phase": "contact_lost", "force_axis_n": 0.1}', 6 * ns, eid)
    assert tr.phase == TaskPhase.RECOVERY
    tr.handle('phase', '{"phase": "recontact"}', 7 * ns, eid)
    assert tr.phase == TaskPhase.REACQUISITION
    tr.handle('phase', '{"phase": "retract"}', 8 * ns, eid)       # unmapped: phase kept
    assert tr.phase == TaskPhase.REACQUISITION
    tr.handle('phase', '{"phase": "fault", "reason": "force_max"}', 9 * ns, eid)
    tr.handle('pause', '{"state": "paused"}', 10 * ns, eid)
    texts = [e.text for e in tr.events]
    assert {'Contact lost', 'Contact regained', 'Path run started', 'Paused'} <= set(texts)
    assert any(t.startswith('Fault: force_max') for t in texts)
    tr.handle('phase', '{"phase": "search"}', 11 * ns, eid)        # Stage 2 search: no path_length_m
    assert tr.phase == TaskPhase.VIEW_ACQUISITION


def test_ros_snapshot_mapping_without_ros():
    """Feed fake messages into RosSource and check the mapping (rclpy is never imported)."""
    from usrig_demo_gui.sources.ros_source import RosSource
    src = RosSource(CFG)
    pose = SimpleNamespace(position=SimpleNamespace(x=0.4, y=0.1, z=0.06),
                           orientation=SimpleNamespace(x=1.0, y=0.0, z=0.0, w=0.0))   # tool z down
    robot = SimpleNamespace(q=[3.1, -1.4, 2.2, -2.4, -1.57, 0.0], tcp_base=pose,
                            wrench=[0, 0, -4.0, 0, 0, 0], mode=3, safety_flags=0)
    img = np.random.default_rng(0).integers(0, 255, (400, 380), np.uint8)
    bmode = SimpleNamespace(encoding='mono8', height=400, width=380, step=380, data=img.tobytes())
    now = time.monotonic()
    src.latest.update(robot=robot, bmode=bmode, frame_info={'frame_index': 7, 'settings': {'depth': 40},
                                                            'image': {'pixel_spacing_mm': 0.1}})
    for k in ('robot', 'bmode', 'frame_info'):
        src.stamp[k] = now
    d = src.snapshot()
    assert d.robot.connected and d.robot.mode == 'TELEOP'
    assert d.contact == ContactState.UNKNOWN                # threshold null in the config -> Unknown
    assert d.plane == PlaneState.UNKNOWN
    assert d.ultrasound.status == StreamStatus.LIVE and d.ultrasound.depth_mm == 40
    assert abs(d.metrics.items[0].value - 4.0) < 1e-9
    assert d.rgb.status == StreamStatus.DISCONNECTED
    assert set(d.robot.frames_missing) == {'flange', 'tip', 'camera'}     # rig.yaml not loaded: nothing guessed
    src.rig = {'robot': {'tcp_setting': [0, 0, 0.13, 0, 0, 0]}, 'probe': {'tcp_already_includes_probe': True}}
    from usrig_demo_gui.frames import pose_from_pq
    src.latest['tcp_pose'] = pose_from_pq([0.4, 0.1, 0.06], [1.0, 0.0, 0.0, 0.0])
    src.stamp['tcp_pose'] = now
    d = src.snapshot()
    assert np.allclose(d.robot.T_tip[:3, 3], (0.4, 0.1, 0.06))
    assert np.allclose(d.robot.T_flange[:3, 3], (0.4, 0.1, 0.19))         # 0.13 m up along the tool axis
    assert d.robot.T_camera is None and 'camera' in d.robot.frames_missing
    # frame_info as micrus --simulate publishes it (captured 15 Oct 2026)
    src.latest['frame_info'] = {'frame_index': 254, 'image': {'pixel_spacing_mm': [0.1, 0.1], 'height': 400},
                                'settings': {'depth_mm': 40, 'gain': 64, 'frequency_hz': 12000000}}
    u = src.snapshot().ultrasound
    assert u.depth_mm == 40 and u.gain == 64 and abs(u.freq_mhz - 12.0) < 1e-9
    # rates measured in the GUI for rows qc_monitor does not report (its own /qc/status, control status)
    for k, hz in (('qc', 2.0), ('control', 125.0)):
        src.stamp[k] = now
        src.arrivals[k].extend(now - i / hz for i in range(10, -1, -1))
    rows = {n.name: n for n in src.snapshot().nodes}
    assert abs(rows['qc_monitor'].rate_hz - 2.0) < 0.01 and abs(rows['robot_control'].rate_hz - 125.0) < 0.5
    CFG2 = dict(CFG, contact_force_threshold_n=1.0)
    src.cfg = CFG2
    assert src.snapshot().contact == ContactState.IN_CONTACT


# ------------------------------------------------------------------ GUI smoke test
def test_window_builds_offscreen():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PySide6 import QtWidgets
    from usrig_demo_gui.main_window import MainWindow, apply_theme
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(app)
    for name, cfg in (('normal', CFG), ('disconnected', CFG), ('normal', dict(CFG, show_recording_panel=True))):
        w = MainWindow(MockSource(cfg, name), cfg, robot_view='simple')   # no OpenGL offscreen
        assert (w.rec is not None) == bool(cfg.get('show_recording_panel'))
        assert w.episode is not None and w.timeline is not None
        assert (w.rec in [x for x, _ in w.rail_items]) if w.rec is not None else True
        w.timer.stop()
        for size in ((1920, 1080), (1200, 800)):
            w.resize(*size)
            w.show()
            w.refresh()
            app.processEvents()
            assert not w.grab().isNull()
        w.close()


def test_urdf_chain_matches_dh():
    """The mesh view (URDF frames) and the DH chain agree at the flange for any joint values."""
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, 6)
        assert np.allclose(kin.urdf_frames(q)['tool0'], kin.fk_frames(q)[6], atol=1e-9)


def test_mesh_asset_complete():
    from pathlib import Path
    d = np.load(Path(kin.__file__).parent / 'assets' / 'ur7e_meshes.npz')
    for link in ('base', 'shoulder', 'upper_arm', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3'):
        v, f, n = d[f'{link}_v'], d[f'{link}_f'], d[f'{link}_n']
        assert len(v) == len(n) == len(d[f'{link}_c']) and f.max() < len(v)


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='needs a display with OpenGL (e.g. xvfb-run)')
def test_gl_view_renders():
    pytest.importorskip('OpenGL', reason='PyOpenGL not installed (the GUI falls back to the line drawing)')
    from PySide6 import QtWidgets
    from usrig_demo_gui.widgets.robot_view_gl import RobotViewGL
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = RobotViewGL()
    w.resize(400, 300)
    w.show()
    w.set_state(kin.Q_SEED, True)
    w._apply()
    app.processEvents()
    img = w.grab_image()
    assert img.width() > 0
    w.close()


# ------------------------------------------------------------------ tool frames and trajectory map
def test_tool_frames_chain():
    from usrig_demo_gui.frames import ToolCalibration, pose_from_ur, tool_frames
    q = kin.ik_point_down((0.42, 0.10, 0.06), (0, 0, -1), kin.Q_SEED)
    T_flange = kin.urdf_frames(q)['tool0']
    # controller TCP = probe tip (tcp_setting 0.13 m along z): flange recovered, tip = TCP
    rig = {'robot': {'tcp_setting': [0, 0, 0.13, 0, 0, 0]}, 'probe': {'tcp_already_includes_probe': True},
           'calibration': {'wrist_cam_cali_mtx': [0.09, 0, 0.045, 0, -0.349, 0]}}
    cal = ToolCalibration.from_rig(rig)
    T_tcp = T_flange @ pose_from_ur([0, 0, 0.13, 0, 0, 0])
    fr = tool_frames(T_tcp, cal)
    assert np.allclose(fr.flange, T_flange, atol=1e-9)
    assert np.allclose(fr.tip, T_tcp)
    assert np.allclose(fr.camera, T_flange @ pose_from_ur([0.09, 0, 0.045, 0, -0.349, 0]))
    assert cal.missing() == {} and cal.camera_source == 'rig.yaml'
    # controller TCP = flange, tip from probe.tip_offset_flange_m; camera from /tf_static wins over rig.yaml
    T_cam_tf = pose_from_ur([0.05, 0.01, 0.03, 0, 0, 0])
    rig2 = {'robot': {'tcp_setting': [0, 0, 0, 0, 0, 0]},
            'probe': {'tcp_already_includes_probe': False, 'tip_offset_flange_m': [0, 0, 0.13, 0, 0, 0]},
            'calibration': rig['calibration']}
    cal2 = ToolCalibration.from_rig(rig2, camera_from_tf=T_cam_tf)
    fr2 = tool_frames(T_flange, cal2)
    assert np.allclose(fr2.tip[:3, 3], fr.tip[:3, 3]) and np.allclose(fr2.camera, T_flange @ T_cam_tf)
    assert cal2.camera_source == '/tf_static'
    # repo defaults (all null): nothing is guessed
    cal3 = ToolCalibration.from_rig({'robot': {'tcp_setting': None}, 'probe': {'tcp_already_includes_probe': None},
                                     'calibration': {'wrist_cam_cali_mtx': None}})
    fr3 = tool_frames(T_tcp, cal3)
    assert fr3.flange is None and fr3.tip is None and fr3.camera is None
    assert set(cal3.missing()) == {'flange', 'tip', 'camera'}


def _frames_at(z, y=0.10):
    from usrig_demo_gui.frames import ToolFrames
    T = np.eye(4)
    T[:3, 3] = (0.42, y, z)
    F = T.copy()
    F[2, 3] += 0.13
    return ToolFrames(flange=F, tip=T, camera=None)


def test_trajectory_map_runs_and_markers():
    from usrig_demo_gui.trajectory import TrajectoryMap
    tm = TrajectoryMap(rest_s=1.0, min_step_m=0.001, merge_m=0.005)
    t = 0.0
    for _ in range(30):                                  # parked at the rest pose, no episode
        tm.update(t, _frames_at(0.20), False, None)
        t += 0.1
    assert tm.at_rest
    for k in range(1, 15):                               # approach down to the surface
        tm.update(t, _frames_at(0.20 - k * 0.01), False, 'Approach')
        t += 0.1
    assert [m.label for m in tm.markers][:2] == ['Rest', 'Approach']   # 1 cm apart > merge_m: two markers
    assert np.allclose(tm.tip()[0], (0.42, 0.10, 0.20))  # run starts at the rest position
    for k in range(20):                                  # hold still in contact during the episode
        tm.update(t, _frames_at(0.06), True, 'Contact')
        t += 0.1
    assert not tm.at_rest                                # time inside an episode is not a rest
    for k in range(1, 30):                               # sweep 2.9 cm
        tm.update(t, _frames_at(0.06, 0.10 + k * 0.001), True, 'Sweep')
        t += 0.1
    tm.update(t, _frames_at(0.06, 0.129), True, None)    # episode ends
    labels = [m.label for m in tm.markers]
    assert labels[2] == 'Contact / Sweep' and labels[-1] == 'End'   # sweep starts 1 mm away: merged
    in_air = tm.tip_in_air()
    assert in_air[0] and not in_air[-1]
    n = len(tm)
    for _ in range(30):                                  # retract finished, parked: map kept
        tm.update(t, _frames_at(0.20, 0.129), False, None)
        t += 0.1
    assert len(tm) >= n and tm.at_rest
    tm.update(t, _frames_at(0.19, 0.129), False, None)   # moves off: new run from this rest
    assert len(tm) == 2 and tm.markers[0].label == 'Rest'


def test_mock_frames_match_drawn_arm():
    d = at(MockSource(CFG, 'normal'), 14.0)
    r = d.robot
    assert np.allclose(r.T_tip[:3, 3], kin.tcp(r.joints_rad)[:3, 3], atol=1e-9)
    assert np.allclose(r.T_flange, kin.urdf_frames(r.joints_rad)['tool0'], atol=1e-9)
    assert r.T_camera is not None and r.frames_missing == {}


def test_timeline_history_and_events():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PySide6 import QtWidgets
    from usrig_demo_gui.widgets.timeline import TimelinePanel
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tl = TimelinePanel(window_s=30.0)
    src = MockSource(CFG, 'recovery')
    for t in (2.0, 6.0, 9.0, 13.0, 17.5, 20.0):
        tl.update_state(at(src, t))
    phases = [p for _, p in tl.phase_hist if p is not None]
    assert TaskPhase.RECOVERY in phases and phases[0] == TaskPhase.APPROACH
    texts = [v[2] for v in tl.events.values()]
    assert 'Contact lost' in texts and 'Contact detected' in texts
    by_text = {v[2]: v[0] for v in tl.events.values()}
    # 'Staged perturbation' (16.0 s) and 'Contact lost' (16.35 s) first arrive in the same update:
    # they keep their episode-time spacing instead of both being stamped "now"
    assert abs((by_text['Contact lost'] - by_text['Staged perturbation: lift (FM-02)']) - 0.35) < 0.02
    assert sum(r.isVisible() for r in tl._rects) >= 4     # one band segment per phase seen
    app.processEvents()


# ------------------------------------------------------------------ demo only: paused during collection
def test_collection_guard_rule():
    from usrig_demo_gui.collection_guard import collection_lock
    real = {'session': {'session_id': '2026-10-15_x', 'sim': False}}
    sim = {'session': {'session_id': 'sim_1', 'sim': True}}
    assert collection_lock(None, None, None)                                   # graph not read yet
    assert collection_lock({'robot_state', 'micrus'}, None, None) is None      # no episode_manager
    assert collection_lock({'episode_manager'}, sim, 0.5) is None              # simulation session
    assert '2026-10-15_x' in collection_lock({'episode_manager'}, real, 0.5)   # real collection
    assert collection_lock({'episode_manager'}, None, None)                    # sim flag not known: paused
    assert collection_lock({'episode_manager'}, sim, 60.0)                     # stale status: paused
    assert collection_lock(set(), real, 0.5) is None                           # episode_manager gone


def test_paused_snapshot_and_window():
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PySide6 import QtWidgets
    from usrig_demo_gui.main_window import MainWindow, apply_theme
    from usrig_demo_gui.sources.ros_source import RosSource
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    apply_theme(app)
    src = RosSource(CFG)
    src.paused_reason = 'Data collection in progress (session s01).'
    d = src.snapshot()
    assert d.paused_reason and d.robot.joints_rad is None and d.ultrasound.image is None
    win = MainWindow(src, CFG, robot_view='simple')
    win.resize(1920, 1080)
    win.show()
    win.refresh()
    assert win.stack.currentIndex() == 1 and 's01' in win.paused_lbl.text()
    h = win.header
    assert h.system.val.text() == 'Paused' and h.badge.text() == 'Paused'      # not a made-up "Idle"
    assert all(f.val.text() != 'Idle' for f in (h.contact, h.plane, h.phase, h.outcome))
    src.paused_reason = None                                  # session ended: live view again
    win.refresh()
    assert win.stack.currentIndex() == 0
    assert h.badge.text() == 'Live, read-only' and 'synthetic' not in h.badge.toolTip()
    win.close()


def test_episode_output_path():
    from usrig_demo_gui.sources.ros_source import episode_paths
    sim = {'session': {'session_id': '2026-10-16_s01', 'sim': True,
                       'session_dir': '/home/u/data_sim/sessions/2026-10-16_s01'}}
    # episode_manager's own session_dir (sim sessions live under ~/data_sim)
    assert episode_paths('2026-10-16_s01_ep_0003', sim) == \
        ('2026-10-16_s01', 'ep_0003', '/home/u/data_sim/sessions/2026-10-16_s01/ep_0003/')
    # no status (or a different session): <data_root>/sessions/<session>/ep_NNNN/ as usrig_common.session
    assert episode_paths('2026-10-16_s01_ep_0003', None, '~/data/')[2] == '~/data/sessions/2026-10-16_s01/ep_0003/'
    assert episode_paths('other_ep_0001', sim)[2] == '~/data/sessions/other/ep_0001/'
    assert episode_paths(None, sim) == (None, None, None)


def test_demo_gui_pauses_during_test_recordings():
    """test1-3 record:=true and temporal-calibration bags run without episode_manager; the demo GUI's load
    (about one core) belongs to the recorder then."""
    from usrig_demo_gui.collection_guard import collection_lock
    assert collection_lock({'robot_state', 'rosbag2_recorder'}, None, None)
    assert collection_lock({'robot_state', 'micrus'}, None, None) is None
