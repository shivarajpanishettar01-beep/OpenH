"""Stage gates must be recoverable: one old bad episode must not block collection forever."""
from usrig_common import gates

CFG = {'gate12': {'max_speed_error_pct': 5.0, 'min_paths': 3},
       'gate01': {'min_consecutive_qc_pass': 3, 'key_speeds_chosen': True, 'force_band_chosen': True}}


def st1(i, target, err=2.0, gap_fail=False, **kw):
    checks = {'micrus_frame_gaps': {'status': 'fail' if gap_fail else 'pass'}}
    return {'stage': 1, 'outcome': 'success', 't_start_ns': i, 'target': target,
            'speed_tracking': {'max_error_pct': err}, 'qc': {'pass': not gap_fail, 'checks': checks}, **kw}


def test_gate12_recovers_after_an_old_gap_failure():
    eps = [st1(0, 'p1', gap_fail=True), st1(1, 'p1'), st1(2, 'p2'), st1(3, 'p3')]
    assert gates.gate12(CFG, eps)['passed'] is True
    assert gates.gate12(CFG, eps + [st1(4, 'p1', gap_fail=True)])['passed'] is False   # a recent one still blocks


def test_gate12_counts_distinct_paths():
    eps = [st1(i, 'p1') for i in range(5)]
    r = gates.gate12(CFG, eps)
    assert r['passed'] is False and r['paths_within_error'] == 1


def test_reviewed_exclusion_and_freedrive_do_not_count():
    excl = {'by': 'op1', 'reason': 'cable pulled'}
    eps = [st1(0, 'p1'), st1(1, 'p2'), st1(2, 'p3'), st1(3, 'p1', gap_fail=True, gate_excluded=excl)]
    assert gates.gate12(CFG, eps)['passed'] is True
    fd = [{'stage': 0, 'outcome': 'success', 'collection_mode': 'freedrive', 'qc': {'pass': True}}] * 3
    assert gates.gate01(CFG, fd)['passed'] is False                      # kinesthetic demos are not teleop shakedown
