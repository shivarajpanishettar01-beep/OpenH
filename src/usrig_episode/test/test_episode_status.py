"""Pause accounting and /episode/status payload (pure)."""
import types

from usrig_episode import episode_status as es

S = 10 ** 9


def test_pause_log_spans_and_total():
    p = es.PauseLog()
    assert p.pause(10 * S) and not p.pause(11 * S)          # second pause ignored
    assert p.paused and abs(p.total_s(13 * S) - 3.0) < 1e-9
    assert p.resume(14 * S) and not p.resume(15 * S)
    p.pause(20 * S)
    spans = p.closed(22 * S)                                 # open span closed at the end
    assert spans == [[10 * S, 14 * S], [20 * S, 22 * S]] and not p.paused
    rec = es.episode_record(spans, 5 * S)
    assert rec['pauses'] == [[5.0, 9.0], [15.0, 17.0]] and rec['paused_s'] == 6.0


def test_state_and_summaries():
    assert es.episode_state(None) == 'idle'
    assert es.episode_state(None, finishing=True) == 'finishing'
    assert es.episode_state({'t_bag_ready_ns': None}) == 'starting'
    assert es.episode_state({'t_bag_ready_ns': 1, 't_start_ns': None}) == 'preparing'
    assert es.episode_state({'t_bag_ready_ns': 1, 't_start_ns': 2}) == 'recording'
    assert es.episode_number({'episode_dir': 'ep_0012'}) == 12
    eps = [{'outcome': 'success', 'qc': {'pass': True}, 'duration_s': 30, 'paused_s': 6},
           {'outcome': 'discard', 'qc': {'pass': True}, 'duration_s': 30},
           {'outcome': 'fail', 'qc': {'pass': False, 'failed': ['gaps']}, 'duration_s': 30}]
    c = es.summarize_episodes(eps)
    assert c['episodes'] == 3 and c['qc_failed'] == 1 and c['kept_minutes'] == 0.4    # 24 s kept
    assert c['by_outcome'] == {'success': 1, 'discard': 1, 'fail': 1}


def test_status_payload_has_the_episode_details():
    ep = {'episode_id': 's_ep_0003', 'episode_dir': 'ep_0003', 'type': 'shakedown', 't_bag_ready_ns': 1,
          't_start_ns': 1_760_000_000 * S, 'view_reached_ns': [1, 2]}
    p = es.PauseLog()
    p.pause(1_760_000_010 * S)
    out = es.status_payload(session={'session_id': 's'}, ep=ep, pause_log=p, waiting_for={'start_stop', 'outcome'},
                            finishing=False, last_ep=None, counts={}, plan=None, now_ns=1_760_000_012 * S)
    e = out['episode']
    assert out['state'] == 'recording' and out['waiting_for'] == ['outcome', 'start_stop']
    assert e['number'] == 3 and e['elapsed_s'] == 12.0 and e['paused'] and e['paused_s'] == 2.0
    assert e['date'] and e['start_time'] and e['view_reached'] == 2


def test_episode_manager_pause_marks_and_holds_robot():
    """_set_paused: a pause event in the bag and a robot hold; resume releases it; nothing before the start."""
    import json
    from usrig_episode.episode_manager import EpisodeManager
    sent, events = [], []
    fake = types.SimpleNamespace(pause_log=es.PauseLog(), control=True,
                                 event=lambda kind, v, t=None: events.append((kind, v['state'])),
                                 pub_cmd=types.SimpleNamespace(publish=lambda m: sent.append(json.loads(m.data))))
    ep = {'t_start_ns': 1}
    EpisodeManager._set_paused(fake, ep, True, 5 * S)
    EpisodeManager._set_paused(fake, ep, True, 6 * S)                   # already paused: nothing new
    EpisodeManager._set_paused(fake, ep, False, 8 * S)
    assert events == [('pause', 'paused'), ('pause', 'resumed')]
    assert sent == [{'cmd': 'hold', 'value': True}, {'cmd': 'hold', 'value': False}]
    events.clear()
    EpisodeManager._set_paused(fake, {'t_start_ns': None}, True, 9 * S)  # not recording yet: ignored
    assert events == []


def test_time_cap_does_not_count_paused_time():
    """A pause must not end the episode: Stage 2 cap_s equals the longest planned scan."""
    import threading
    from usrig_episode.episode_manager import EpisodeManager
    from usrig_common.timeutil import now_ns
    t0 = now_ns() - 25 * S                                   # scanning for 25 s ...
    p = es.PauseLog()
    p.pause(t0 + 5 * S)
    p.resume(t0 + 15 * S)                                    # ... of which 10 s paused
    fake = types.SimpleNamespace(pause_log=p, ep_lock=threading.Lock())
    left = EpisodeManager._cap_left(fake, 30.0, t0)
    assert 14.5 < left < 15.5                                # 30 - (25 - 10)
