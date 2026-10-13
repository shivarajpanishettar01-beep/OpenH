import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'tools'))
import openh_fields as F  # noqa: E402
import temporal_calib as TC  # noqa: E402


def test_delta_actions_base_and_tool():
    q0 = Rotation.from_euler('z', 0, degrees=True)
    q1 = Rotation.from_euler('z', 10, degrees=True)
    P = np.array([[0, 0, 0, *q0.as_quat()], [0.01, 0, 0, *q1.as_quat()]])
    a = F.delta_actions(P, 'base')
    assert np.allclose(a[0], [0.01, 0, 0, 0, 0, np.radians(10)]) and np.allclose(a[1], 0)
    q0 = Rotation.from_euler('z', 90, degrees=True)
    P = np.array([[0, 0, 0, *q0.as_quat()], [0.01, 0, 0, *q0.as_quat()]])
    assert np.allclose(F.delta_actions(P, 'tool')[0][:3], [0, -0.01, 0])   # base +x is tool -y


def test_step_labels():
    S = 10**9
    ann = {'status_intervals': [{'status': 'failure', 't0_ns': 2 * S, 't1_ns': 3 * S}],
           'phases': [{'phase': 'sweep_scan', 't0_ns': 0, 't1_ns': None}],
           'gestures': [{'gesture': 'slide', 't0_ns': 1 * S, 't1_ns': 4 * S},
                        {'gesture': 'rock', 't0_ns': 2 * S, 't1_ns': None}]}
    st, ph, g = F.step_labels(ann, [0, int(2.5 * S), 5 * S])
    assert st == ['nominal', 'failure', 'nominal'] and ph == ['sweep_scan'] * 3
    assert g == ['', 'rock+slide', 'rock']


def test_png16_is_lossless():
    d = np.random.default_rng(0).integers(0, 65535, (48, 64), dtype=np.uint16)
    assert np.array_equal(F.unpng16(F.png16(d)), d)


def test_sample_hold():
    assert F.sample_hold([10, 30, 20], [5, 10, 25, 99]).tolist() == [-1, 0, 2, 1]


@pytest.mark.parametrize('lag_ms,invert', [(40.0, False), (-25.0, False), (60.0, True)])
def test_temporal_calib_recovers_known_lag(lag_ms, invert):
    f = 0.5
    t_ref = np.arange(0, 20, 0.002)
    x_ref = 0.003 * np.sin(2 * np.pi * f * t_ref + 0.3)
    t_sig = np.arange(0.01, 20, 1 / 27.0)
    x_sig = np.interp(t_sig - lag_ms / 1000, t_ref, x_ref) * (-1 if invert else 1) * 1000 + 50
    lag, corr = TC.xcorr_lag(t_ref, x_ref, t_sig, x_sig, 0.2)
    assert lag * 1000 == pytest.approx(lag_ms, abs=1.5) and corr > 0.99
    assert TC.dominant_freq(t_ref, x_ref) == pytest.approx(f, rel=0.05)
    lag_s, _ = TC.sine_lag(t_ref, x_ref, t_sig, x_sig, f)
    assert lag_s * 1000 == pytest.approx(lag_ms, abs=1.5)
