"""rig_setup: every item exists in the config files, the checks catch contradictions, a scripted session
writes exactly what was typed (comments kept, record saved) and refuses to write on an ERROR."""
import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml

from usrig_common import config
from usrig_robot import rig_setup as rs

CFG = Path(__file__).resolve().parents[2] / 'usrig_bringup/config'


@pytest.fixture
def cfg_dir(tmp_path):
    d = tmp_path / 'config'
    shutil.copytree(CFG, d)
    s = yaml.safe_load((d / 'session.yaml').read_text())
    (d / 'session.yaml').write_text((d / 'session.yaml').read_text().replace(
        f"root: {s['session']['root']}", f"root: {tmp_path / 'data'}", 1))
    return d


def test_every_item_is_in_its_file(cfg_dir):
    for it in rs.ITEMS:
        config.write_yaml_path(cfg_dir / f'{it.file}.yaml', it.key, None)     # KeyError if missing
    missing = set(config.MOTION_LIMITS) - {it.key for it in rs.ITEMS} - {'robot.control_backend'}
    assert not missing, f'required by robot_control but not asked by rig_setup: {missing}'


def test_parse():
    assert rs.parse('vec3', '[0.1, -0.2 0.3]') == [0.1, -0.2, 0.3]
    assert rs.parse('bool', 'yes') is True and rs.parse('float_or_null', 'null') is None
    with pytest.raises(ValueError):
        rs.parse('pair', '1 2 3')
    with pytest.raises(ValueError):
        rs.parse('float', 'null')


def good_values():
    rig = {'robot': {'tcp_setting': [0, 0, 0.21, 0, 0, 0], 'payload_kg': 0.8, 'payload_cog_m': [0, 0, 0.06]},
           'probe': {'tcp_already_includes_probe': True},
           'limits': {'workspace_box_base_m': {'min': [0.2, -0.3, 0.0], 'max': [0.6, 0.3, 0.4]},
                      'tilt_reference_axis_base': [0, 0, -1], 'tilt_cone_deg': 30, 'local_tilt_cone_deg': 10,
                      'max_linear_speed_mm_s': 30, 'max_angular_speed_deg_s': 15, 'max_linear_accel_mm_s2': 200,
                      'max_angular_accel_deg_s2': 100, 'stop_decel_mm_s2': 500, 'deadman_timeout_s': 0.2,
                      'contact_detect_threshold_n': 1.0, 'contact_lost_threshold_n': 0.5, 'contact_lost_time_s': 0.2,
                      'force_band_n': [2, 8], 'force_setpoint_default_n': 4, 'max_contact_force_n': 15},
           'rtde': {'force_mode_limits': [0.02, 0.02, 0.02, 0.1, 0.1, 0.1]},
           'motion': {'transit_speed_mm_s': 20, 'descent_speed_mm_s': 5, 'descent_max_travel_mm': 60,
                      'approach_offset_mm': 50}}
    km = {'speeds': {'slide_along_mm_s': 10, 'slide_across_mm_s': 10, 'rock_deg_s': 5, 'tilt_deg_s': 5,
                     'rotate_deg_s': 5}, 'force_step_n': 0.5}
    return rig, km


def test_consistent_values_pass():
    rig, km = good_values()
    assert rs.validate(rig, km) == []


@pytest.mark.parametrize('path,value,needle', [
    ('limits.force_band_n', [2, 20], 'below limits.max_contact_force_n'),
    ('limits.force_setpoint_default_n', 9, 'inside limits.force_band_n'),
    ('limits.local_tilt_cone_deg', 40, 'must not exceed'),
    ('limits.workspace_box_base_m.max', [0.1, 0.3, 0.4], 'every min must be below'),
    ('robot.tcp_setting', [0, 0, 210, 0, 0, 0], 'entered in mm'),
    ('probe.tcp_already_includes_probe', False, 'must be yes'),
    ('robot.payload_kg', 9.0, 'UR7e'),
    ('limits.deadman_timeout_s', 0.005, 'two control periods'),
])
def test_contradictions_are_errors(path, value, needle):
    rig, km = good_values()
    node = rig
    *parents, last = path.split('.')
    for k in parents:
        node = node[k]
    node[last] = value
    errs = [m for lvl, m in rs.validate(rig, km) if lvl == 'ERROR']
    assert any(needle in m for m in errs), errs


def test_key_speed_above_cap_and_noise_warning():
    rig, km = good_values()
    km['speeds']['slide_along_mm_s'] = 50
    assert any('keyboard_teleop refuses' in m for lvl, m in rs.validate(rig, km) if lvl == 'ERROR')
    rig, km = good_values()
    warns = [m for lvl, m in rs.validate(rig, km, {'axis_std_n': 0.5}) if lvl == 'WARN']
    assert any('3x the measured free-air noise' in m for m in warns)


class FakeRtde:
    """Read-only receive interface: a probe pointing down, moved to the box corners in turn."""

    def __init__(self):
        self.corners = [[0.25, -0.20, 0.05], [0.55, 0.20, 0.30]]
        self.k = 0

    def getActualTCPPose(self):
        p = self.corners[min(self.k, 1)]
        return [*p, np.pi, 0.0, 0.0]

    def getActualTCPForce(self):
        return [0.0, 0.0, 0.1, 0.0, 0.0, 0.0]

    def getPayload(self):
        return 0.85

    def getPayloadCog(self):
        return [0.0, 0.01, 0.06]

    def disconnect(self):
        pass


def scripted(answers, fake=None):
    it = iter(answers)

    def ask(prompt):
        a = next(it)
        if fake is not None and prompt.startswith('[1 points]'):
            fake.k = 1                               # the operator moved to the second corner
        return a
    return ask


def test_scripted_session_writes_typed_and_measured_values(cfg_dir):
    fake = FakeRtde()
    answers = ['192.168.10.2', '0 0 0.21 0 0 0', 'yes', 'MPRO-L40S', 'm', '',     # robot (m reads the payload)
               'm', '', '', 'q', '',                                              # box: 2 corners, q, then keep max
               'q',                                                               # finish
               'yes']                                                             # write
    rc = rs.main(['--config-dir', str(cfg_dir), '--section', 'robot', 'workspace'], ask=scripted(answers, fake),
                 connect=lambda ip: fake)
    assert rc == 0
    rig = yaml.safe_load((cfg_dir / 'rig.yaml').read_text())
    assert rig['robot']['ip'] == '192.168.10.2' and rig['robot']['tcp_setting'] == [0, 0, 0.21, 0, 0, 0]
    assert rig['robot']['payload_kg'] == 0.85 and rig['robot']['payload_cog_m'] == [0.0, 0.01, 0.06]
    assert rig['limits']['workspace_box_base_m'] == {'min': [0.25, -0.2, 0.05], 'max': [0.55, 0.2, 0.3]}
    assert rig['limits']['max_contact_force_n'] is None                             # not asked: untouched
    assert '# [x, y, z] in the robot base frame' in (cfg_dir / 'rig.yaml').read_text()
    rec = sorted((cfg_dir.parent / 'data' / 'calibration').glob('rig_setup_*'))
    assert rec and (rec[0] / 'rig.yaml.before').exists() and (rec[0] / 'changes.json').exists()


def test_error_blocks_the_write(cfg_dir):
    before = (cfg_dir / 'rig.yaml').read_text()
    answers = ['', '0 0 210 0 0 0', 'q']                                         # TCP typed in mm
    rc = rs.main(['--config-dir', str(cfg_dir), '--section', 'robot'], ask=scripted(answers))
    assert rc == 1 and (cfg_dir / 'rig.yaml').read_text() == before


def test_review_reports_nulls(cfg_dir, capsys):
    assert rs.main(['--config-dir', str(cfg_dir), '--review']) == 0
    out = capsys.readouterr().out
    assert 'still null' in out and 'rtde.force_mode_limits' in out
