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


def write_bag(path, drop_frame=None, duration_s=4.0):
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
        img = Image(header=hdr(ns), height=64, width=64, encoding='mono8', is_bigendian=0, step=64,
                    data=np.full(64 * 64, 120, np.uint8))
        fi = {'stamp': ns * 1e-9, 'frame_index': i, 'settings': {'depth': 40, 'lines': 128}}
        msgs += [('/micrus/bmode', ns, img, Image), ('/micrus/frame_info', ns, String(data=json.dumps(fi)), String)]
    # robot 500 Hz, probe pointing down, 4 N into the phantom
    q_down = Quat(x=1.0, y=0.0, z=0.0, w=0.0)
    for k in range(int(duration_s * 500)):
        ns = T0 + k * 2_000_000
        m = RS(header=hdr(ns), controller_time_s=k * 0.002, q=np.zeros(6), qd=np.zeros(6),
               tcp_base=Pose(position=Point(x=0.0, y=0.0, z=0.2), orientation=q_down),
               tcp_phantom=Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=q_down),
               wrench=np.array([0.0, 0.0, -4.0, 0.0, 0.0, 0.0]), force_setpoint_n=4.0, mode=3,
               safety_flags=0, action_source=0, action_is_expert=True)
        msgs.append(('/robot/state', ns, m, RS))
    for t, kind, val in ((0.05, 'phase', {'phase': 'free_air'}), (0.6, 'phase', {'phase': 'contact'}),
                         (0.7, 'start', {}), (3.8, 'end', {}), (3.8, 'outcome', {'outcome': 'success'})):
        ns = T0 + int(t * 1e9)
        msgs.append(('/episode/events', ns, EV(header=hdr(ns), episode_id='e1', kind=kind, value=json.dumps(val)), EV))
    for k in range(int(duration_s * 2)):
        ns = T0 + int(k * 0.5e9)
        msgs.append(('/qc/status', ns, String(data=json.dumps({'wrist_usb': None})), String))

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
