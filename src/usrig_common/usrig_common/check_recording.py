"""Test recordings for the staged rig tests (test1-test4 launch files, docs/TESTING_STEPS.md).

A test recording is a plain MCAP bag of the streams under test, in <root>/checks/<test>_<YYYYmmdd_HHMMSS>/
(real root, or sim root with sim:=true). It is not a collection episode: no episode.json, never exported.
`ros2 run usrig_qc bag_check <bag>` checks it.
"""
import tempfile
import time
from pathlib import Path

import yaml

from .session import data_dir

ROBOT = ['/robot/state', '/joint_states', '/robot/tcp_pose', '/robot/wrench', '/robot/cmd_executed',
         '/robot/control_status', '/tf_static']
MICRUS = ['/micrus/bmode', '/micrus/frame_info']
OPERATOR = ['/teleop/twist', '/operator/command', '/episode/events', '/qc/status']


def camera_topics(scfg, name='wrist_camera'):
    t = (scfg.get(name) or {}).get('topics') or {}
    return [t[k] for k in ('color', 'depth', 'color_info', 'depth_info', 'metadata') if t.get(k)]


def topics_for(scfg, parts):
    """Topics for a set of parts: robot, micrus, camera (wrist), tpv (operator topics always)."""
    out = []
    if 'robot' in parts:
        out += ROBOT
    if 'micrus' in parts:
        out += MICRUS
    if 'camera' in parts:
        out += camera_topics(scfg)
    if 'tpv' in parts:
        out += camera_topics(scfg, 'tpv_camera')
    return out + OPERATOR


def check_dir(scfg, test_name, sim=False):
    return data_dir(scfg, 'checks', sim) / f'{test_name}_{time.strftime("%Y%m%d_%H%M%S")}'


def recording_rate(bag_dir):
    """(MB/s, seconds) of a recorded bag from its size and metadata.yaml duration, or None."""
    meta = Path(bag_dir) / 'metadata.yaml'
    if not meta.exists():
        return None
    info = yaml.safe_load(meta.read_text()).get('rosbag2_bagfile_information', {})
    dur = (info.get('duration') or {}).get('nanoseconds', 0) * 1e-9
    if dur < 5.0:
        return None
    size = sum(f.stat().st_size for f in Path(bag_dir).rglob('*') if f.is_file())
    return size / 1e6 / dur, dur


def estimate_episode_rate(scfg, sim=False, camera=True):
    """Conservative MB/s of one episode (bag + micrus HDF5) from the latest test recordings, for the disk check
    before any episode exists: test2 (robot + ultrasound) counted twice (its ultrasound part again stands in for
    the HDF5), plus test3 (robot + camera) when the camera is recorded. Returns (MB/s, explanation) or None."""
    root = data_dir(scfg, 'checks', sim)

    def latest(name):
        for d in sorted(root.glob(f'{name}_*'), reverse=True):
            r = recording_rate(d)
            if r:
                return r[0], d.name
        return None
    t2 = latest('test2_micrus_robot')
    if t2 is None:
        return None
    rate, used = 2 * t2[0], [f'{t2[1]} x2']
    if camera:
        t3 = latest('test3_camera_robot')
        if t3 is None:
            return None
        rate += t3[0]
        used.append(t3[1])
    return rate, 'estimated (conservative) from ' + ' + '.join(used)


def record_cmd(scfg, topics, out_dir):
    """ros2 bag record command (MCAP, the session.yaml writer options, micrus topics recorded reliably)."""
    rec = scfg['recording']
    d = Path(tempfile.mkdtemp(prefix='usrig_check_'))
    storage = d / 'mcap_writer.yaml'
    storage.write_text(yaml.safe_dump(rec.get('mcap_storage_config') or {}))
    qos = d / 'qos_overrides.yaml'
    qos.write_text(yaml.safe_dump({t: {'reliability': 'reliable', 'history': 'keep_last', 'depth': 100}
                                   for t in topics if t.startswith('/micrus/')}))
    Path(out_dir).parent.mkdir(parents=True, exist_ok=True)
    return ['ros2', 'bag', 'record', '-s', rec.get('storage', 'mcap'), '--storage-config-file', str(storage),
            '--qos-profile-overrides-path', str(qos), '-o', str(out_dir), *topics]
