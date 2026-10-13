import numpy as np
import pytest

from usrig_teleop.ramp import TwistRamp

KEYS = {'W': (0, 1.0), 'S': (0, -1.0), 'A': (1, 1.0), 'Q': (4, 1.0)}
SPEEDS = [0.01, 0.01, 0.0, 0.1, 0.1, 0.1]


def run(r, held, fine, t, dt=0.01):
    out = None
    for _ in range(int(round(t / dt))):
        out = r.step(held, fine, dt)
    return out


def test_ramps_up_in_ramp_time_and_back_to_zero():
    r = TwistRamp(KEYS, SPEEDS, 0.25, 0.25)
    assert run(r, {'W'}, False, 0.10)[0] == pytest.approx(0.004)
    assert run(r, {'W'}, False, 0.20)[0] == pytest.approx(0.01)
    assert np.all(run(r, set(), False, 0.30) == 0)


def test_diagonal_and_opposing_keys():
    r = TwistRamp(KEYS, SPEEDS, 0.25, 0.25)
    out = run(r, {'W', 'A', 'Q'}, False, 0.5)
    assert out[0] == pytest.approx(0.01) and out[1] == pytest.approx(0.01) and out[4] == pytest.approx(0.1)
    assert run(r, {'W', 'S'}, False, 0.5)[0] == pytest.approx(0.0)


def test_fine_mode():
    r = TwistRamp(KEYS, SPEEDS, 0.25, 0.25)
    assert run(r, {'W'}, True, 0.5)[0] == pytest.approx(0.0025)


def test_enable_key_required():
    r = TwistRamp(KEYS, SPEEDS, 0.25, 0.25, enable_keys=['LEFTCTRL'])
    assert np.all(run(r, {'W'}, False, 0.5) == 0)
    assert run(r, {'W', 'LEFTCTRL'}, False, 0.5)[0] == pytest.approx(0.01)
    assert np.all(run(r, {'W'}, False, 0.5) == 0)      # enable released mid-motion: ramps back to zero
