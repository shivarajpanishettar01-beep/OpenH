import copy

import pytest
import yaml

from usrig_episode import plan_generator as pg

CATALOG = yaml.safe_load(open(__file__.replace('test/test_plan.py', 'config/failure_catalog.yaml')))


def filled_catalog():
    c = copy.deepcopy(CATALOG)
    for spec in c.values():
        for m in spec['methods'].values():
            if m.get('kind', 'none') != 'none':
                m['delta'] = [0.001 if v is None else v for v in m['delta']]
                m['duration_s'] = 1.0
    return c


TARGETS = {f't{i}': {'instructions': [f'find view {i} a', f'find view {i} b', f'find view {i} c']}
           for i in range(3)}


def test_type_quotas():
    c = pg.type_counts(400)
    assert c['success'] == 220 and c['failure'] == 60
    assert c['recovery-start'] + c['recovery-mid'] == 120
    assert c['recovery-start'] <= 120 / 3


def test_generate_is_balanced_and_reproducible():
    rows = pg.generate(TARGETS, 200, 7, ['op1', 'op2'], filled_catalog())
    again = pg.generate(TARGETS, 200, 7, ['op1', 'op2'], filled_catalog())
    assert rows == again
    assert sum(r['operator_id'] == 'op2' for r in rows) >= 0.2 * 200
    for r in rows:
        if r['type'] in ('recovery-start', 'recovery-mid', 'failure'):
            assert r['failure_mode'] and r['staging_method']
        assert abs(r['off_rock_deg']) <= 10 and abs(r['off_tilt_deg']) <= 10
    fms = {r['failure_mode'] for r in rows if r['failure_mode']}
    assert fms == set(CATALOG)


def test_null_magnitudes_refuse():
    with pytest.raises(ValueError, match='magnitudes are null'):
        pg.generate(TARGETS, 100, 1, ['op1'], CATALOG)


def test_targets_need_3_to_5_instructions():
    with pytest.raises(ValueError, match='3-5 instructions'):
        pg.generate({'t': {'instructions': ['a']}}, 10, 1, ['op1'], filled_catalog())
