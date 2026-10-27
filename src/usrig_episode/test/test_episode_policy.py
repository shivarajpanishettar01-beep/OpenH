"""episode_manager records robot_control's policy (GR00T-H) runs in episode.json: which policy, and when."""
import json
import threading
import types

from usrig_common.timeutil import stamp_from_ns
from usrig_episode.episode_manager import EpisodeManager
from usrig_interfaces.msg import EpisodeEvent

INFO = {'model': 'gr00t-h-test', 'checkpoint': 'ckpt-0001'}


def ev(kind, value, t_ns):
    e = EpisodeEvent()
    e.header.stamp = stamp_from_ns(t_ns)
    e.kind, e.value = kind, json.dumps(value)
    return e


def fake(ep):
    f = types.SimpleNamespace(ep=ep, ep_lock=threading.Lock())
    f._on_policy_event = lambda msg: EpisodeManager._on_policy_event(f, msg)
    return f


def test_policy_runs_recorded_with_model_and_reason():
    ep = {'policy': None, 'policy_runs': [], 'disturbance': None}
    f = fake(ep)
    EpisodeManager._on_event(f, ev('policy', {'enabled': True, **INFO}, 1_000))
    EpisodeManager._on_event(f, ev('policy', {'enabled': False, 'reason': 'preempted'}, 2_000))
    EpisodeManager._on_event(f, ev('policy', {'enabled': True, **INFO}, 3_000))
    assert ep['policy'] == INFO and 'policy_changed' not in ep
    assert ep['policy_runs'] == [{'start_ns': 1_000, 'end_ns': 2_000, 'reason': 'preempted'},
                                 {'start_ns': 3_000, 'end_ns': None, 'reason': None}]
    assert ep['disturbance'] is None                  # policy events are not perturbations


def test_second_policy_in_one_episode_is_flagged():
    ep = {'policy': None, 'policy_runs': []}
    f = fake(ep)
    EpisodeManager._on_event(f, ev('policy', {'enabled': True, **INFO}, 1_000))
    EpisodeManager._on_event(f, ev('policy', {'enabled': True, **INFO, 'checkpoint': 'ckpt-0002'}, 2_000))
    assert ep['policy'] == INFO and ep['policy_changed'] is True


def test_policy_events_between_episodes_are_ignored():
    f = fake(None)
    EpisodeManager._on_event(f, ev('policy', {'enabled': True, **INFO}, 1_000))
    assert f.ep is None
