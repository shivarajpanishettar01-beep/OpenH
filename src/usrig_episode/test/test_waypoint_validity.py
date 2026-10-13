"""A waypoint runs only if verified with the current probe / TCP / payload (Stage E)."""
import copy

from usrig_episode.waypoints import executable_problem, rig_fingerprint

RIG = {'robot': {'tcp_setting': [0, 0, 0.21, 0, 0, 0], 'payload_kg': 0.62, 'payload_cog_m': [0, 0, 0.06]},
       'probe': {'type': 'MPRO-L40S'}}


def wp(**kw):
    return {'name': 'liver_long', 'verified': True, 'verified_config': rig_fingerprint(RIG), **kw}


def test_verified_with_current_config_runs():
    assert executable_problem(wp(), RIG) is None


def test_unverified_or_legacy_refused():
    assert 'not verified' in executable_problem(wp(verified=False), RIG)
    legacy = wp()
    del legacy['verified_config']
    assert 'no verified_config' in executable_problem(legacy, RIG)


def test_config_change_invalidates():
    for path, value in (('robot.tcp_setting', [0, 0, 0.215, 0, 0, 0]), ('probe.type', 'other'),
                        ('robot.payload_kg', 0.7), ('robot.payload_cog_m', [0, 0.01, 0.06])):
        rig = copy.deepcopy(RIG)
        sec, key = path.split('.')
        rig[sec][key] = value
        msg = executable_problem(wp(), rig)
        assert msg and 'verify it again' in msg, path


def test_yaml_float_round_trip_is_not_a_change():
    w = wp(verified_config={**rig_fingerprint(RIG), 'tcp_setting': [0.0, 0.0, 0.21000000001, 0.0, 0.0, 0.0]})
    assert executable_problem(w, RIG) is None
