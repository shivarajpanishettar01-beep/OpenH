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


def camera_topics(scfg):
    t = scfg['wrist_camera']['topics']
    return [t[k] for k in ('color', 'depth', 'color_info', 'depth_info', 'metadata') if t.get(k)]


def topics_for(scfg, parts):
    """Topics for a set of parts: robot, micrus, camera (operator topics always)."""
    out = []
    if 'robot' in parts:
        out += ROBOT
    if 'micrus' in parts:
        out += MICRUS
    if 'camera' in parts:
        out += camera_topics(scfg)
    return out + OPERATOR


def check_dir(scfg, test_name, sim=False):
    return data_dir(scfg, 'checks', sim) / f'{test_name}_{time.strftime("%Y%m%d_%H%M%S")}'


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
