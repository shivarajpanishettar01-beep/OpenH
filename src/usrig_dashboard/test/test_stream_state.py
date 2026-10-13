"""The dashboard never shows an old value as current (Stage I)."""
from collections import deque

from usrig_dashboard.dashboard import MOTION_BUTTONS, rate, stream_state


def test_stream_state():
    assert stream_state(None, 10.0, 1.0) == 'NO DATA'
    assert stream_state(9.5, 10.0, 1.0) == 'live'
    assert stream_state(7.0, 10.0, 1.0) == 'STALE 3.0 s'


def test_rate_drops_to_zero_when_stream_stops():
    dq = deque([0.0, 0.1, 0.2, 0.3])
    assert abs(rate(dq, 0.35) - 10.0) < 1e-9
    assert rate(dq, 2.0) == 0.0


def test_motion_buttons_are_confirmed():
    assert MOTION_BUTTONS == {'start_stop', 'next'}
