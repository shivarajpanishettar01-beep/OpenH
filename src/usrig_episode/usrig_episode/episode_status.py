"""Episode status for the operator console, and pause accounting. Pure: no ROS.

episode_manager publishes ``status_payload(...)`` as JSON on /episode/status (2 Hz, latched); the operator
console and any other viewer read it instead of reconstructing state from the event stream.

Pause (operator button): recording CONTINUES. A bag with a hole would no longer line up with the micrus
HDF5 (which keeps every frame) and QC would fail the episode on sample gaps. Instead the paused span is
marked (``pause`` events in the bag, ``pauses`` / ``paused_s`` in episode.json) and the robot holds still
(robot_control operator hold), so held samples are labelled not expert; the export can drop the spans.
"""
import datetime

# What episode_manager is waiting for -> the button the operator should press (console prompt).
COMMAND_LABELS = {
    'start_stop': 'Start / Stop', 'outcome': 'an outcome (Success / Recovered / Fail)', 'discard': 'Discard',
    'next': 'Next row',
}


class PauseLog:
    """Paused spans of one episode, in ns. ``closed()`` closes an open span at the given time."""

    def __init__(self):
        self.spans = []          # [[t0_ns, t1_ns or None], ...]

    @property
    def paused(self):
        return bool(self.spans) and self.spans[-1][1] is None

    def pause(self, t_ns):
        if self.paused:
            return False
        self.spans.append([int(t_ns), None])
        return True

    def resume(self, t_ns):
        if not self.paused:
            return False
        self.spans[-1][1] = max(int(t_ns), self.spans[-1][0])
        return True

    def total_s(self, now_ns):
        return sum(((t1 if t1 is not None else now_ns) - t0) for t0, t1 in self.spans) * 1e-9

    def closed(self, t_end_ns):
        if self.paused:
            self.resume(t_end_ns)
        return [list(s) for s in self.spans]


def episode_record(spans, t_start_ns):
    """episode.json fields for the paused spans: absolute ns and seconds from the episode start."""
    rel = [[round((t0 - t_start_ns) * 1e-9, 3), round((t1 - t_start_ns) * 1e-9, 3)] for t0, t1 in spans] \
        if t_start_ns else []
    return {'pauses_ns': spans, 'pauses': rel,
            'paused_s': round(sum(t1 - t0 for t0, t1 in spans) * 1e-9, 3)}


def episode_state(ep, finishing=False):
    """idle | starting (approach, bag readiness) | preparing (free air, contact) | recording | finishing."""
    if finishing:
        return 'finishing'
    if ep is None:
        return 'idle'
    if ep.get('t_bag_ready_ns') is None:
        return 'starting'
    if ep.get('t_start_ns') is None:
        return 'preparing'
    return 'recording'


def wall_time(t_ns):
    """Local date and time of an ns timestamp ('2026-10-14', '17:42:05'), or (None, None)."""
    if not t_ns:
        return None, None
    d = datetime.datetime.fromtimestamp(t_ns * 1e-9)
    return d.strftime('%Y-%m-%d'), d.strftime('%H:%M:%S')


def episode_number(ep):
    """ep_0007 -> 7."""
    d = (ep or {}).get('episode_dir') or ''
    return int(d[3:]) if d[3:].isdigit() else None


def summarize_episodes(episodes):
    """Counts over finished episode.json dicts: outcomes, QC, kept recording time."""
    by_outcome, qc_fail, kept_s = {}, 0, 0.0
    for e in episodes:
        o = e.get('outcome') or 'none'
        by_outcome[o] = by_outcome.get(o, 0) + 1
        qc = e.get('qc') or {}
        if qc and not qc.get('pass'):
            qc_fail += 1
        if o not in ('discard', 'none') and qc.get('pass'):
            kept_s += float(e.get('duration_s') or 0.0) - float(e.get('paused_s') or 0.0)
    return {'episodes': len(episodes), 'by_outcome': by_outcome, 'qc_failed': qc_fail,
            'kept_minutes': round(kept_s / 60.0, 1)}


def last_summary(ep):
    if not ep:
        return None
    qc = ep.get('qc') or {}
    return {'episode_id': ep.get('episode_id'), 'number': episode_number(ep), 'outcome': ep.get('outcome'),
            'duration_s': ep.get('duration_s'), 'paused_s': ep.get('paused_s'), 'qc_pass': qc.get('pass'),
            'qc_failed': qc.get('failed', [])}


def status_payload(*, session, ep, pause_log, waiting_for, finishing, last_ep, counts, plan, now_ns):
    """The /episode/status message (a dict; published as JSON)."""
    cur = None
    if ep is not None:
        t0 = ep.get('t_start_ns')
        date, clock = wall_time(t0 or ep.get('t_created_ns'))
        cur = {'episode_id': ep.get('episode_id'), 'number': episode_number(ep), 'type': ep.get('type'),
               'target': ep.get('target'), 'instruction': ep.get('instruction'), 'plan_row': ep.get('plan_row'),
               'operator_id': ep.get('operator_id'), 'date': date, 'start_time': clock,
               'elapsed_s': round((now_ns - t0) * 1e-9, 1) if t0 else None,
               'paused': pause_log.paused if pause_log else False,
               'paused_s': round(pause_log.total_s(now_ns), 1) if pause_log else 0.0,
               'outcome': ep.get('outcome'), 'gel_state': ep.get('gel_state'), 'notes': ep.get('notes'),
               'view_reached': len(ep.get('view_reached_ns') or []),
               'onset_t': ep.get('onset_t'), 'recovery_start_t': ep.get('recovery_start_t'),
               'recovery_end_t': ep.get('recovery_end_t')}
    return {'stamp_ns': int(now_ns), 'session': session, 'state': episode_state(ep, finishing),
            'waiting_for': sorted(waiting_for or []), 'episode': cur, 'last_episode': last_ep,
            'counts': counts, 'plan': plan}
