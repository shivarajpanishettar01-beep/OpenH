import shutil
from pathlib import Path

import yaml

import numpy as np

from usrig_robot.rig_measure import force_tilt_table, write_key

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


def test_force_tilt_table_spread():
    def T(tilt_deg):
        a = np.radians(tilt_deg)
        M = np.eye(4)
        M[:3, :3] = np.array([[1, 0, 0], [0, -np.cos(a), -np.sin(a)], [0, np.sin(a), -np.cos(a)]])
        return M
    rows = [(T(0), np.array([0, 0, 0.0, 0, 0, 0])), (T(20), np.array([0, 0, -1.0, 0, 0, 0]))]
    out, sm = force_tilt_table(rows, np.array([0, 0, -1.0]))
    assert abs(out[1]['tilt_deg'] - 20) < 1e-6
    assert abs(sm['f_axis_spread_n'] - np.cos(np.radians(20))) < 1e-6 and abs(sm['f_norm_max_n'] - 1.0) < 1e-9
