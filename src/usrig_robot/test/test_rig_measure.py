import shutil
from pathlib import Path

import yaml

from usrig_robot.rig_measure import write_key

RIG = Path(__file__).resolve().parents[2] / 'usrig_bringup/config/rig.yaml'


def test_write_key_keeps_comments_and_parses(tmp_path):
    p = tmp_path / 'rig.yaml'
    shutil.copy(RIG, p)
    write_key(p, 'min', [-0.5, -0.6, 0.05])
    write_key(p, 'max', [0.1, -0.2, 0.4])
    write_key(p, 'tilt_reference_axis_base', [0.0, 0.0, -1.0])
    cfg = yaml.safe_load(p.read_text())
    box = cfg['limits']['workspace_box_base_m']
    assert box == {'min': [-0.5, -0.6, 0.05], 'max': [0.1, -0.2, 0.4]}
    assert cfg['limits']['tilt_reference_axis_base'] == [0.0, 0.0, -1.0]
    assert '# [x, y, z] in the robot base frame' in p.read_text()
    # nothing else changed
    orig = yaml.safe_load(RIG.read_text())
    orig['limits']['workspace_box_base_m'] = box
    orig['limits']['tilt_reference_axis_base'] = [0.0, 0.0, -1.0]
    assert cfg == orig
