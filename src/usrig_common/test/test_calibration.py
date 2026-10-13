import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from usrig_common import calibration as C


def rand_pose(rng, rot_deg=40, trans=0.3):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.radians(rot_deg) * rng.uniform(-1, 1, 3)).as_matrix()
    T[:3, 3] = rng.uniform(-trans, trans, 3)
    return T


def test_pivot_recovers_tip_and_pivot_with_noise():
    rng = np.random.default_rng(1)
    tip = np.array([0.012, -0.004, 0.185])          # tip in TCP frame
    pivot = np.array([0.40, -0.20, 0.05])           # divot in base
    T = []
    for _ in range(20):
        R = Rotation.from_rotvec(np.radians(35) * rng.uniform(-1, 1, 3)).as_matrix()
        t = pivot - R @ tip + rng.normal(0, 0.0002, 3)    # 0.2 mm robot noise
        P = np.eye(4); P[:3, :3] = R; P[:3, 3] = t
        T.append(P)
    r = C.pivot_calibration(T)
    assert np.linalg.norm(r['tip_in_tcp_m'] - tip) * 1000 < 0.5
    assert np.linalg.norm(r['pivot_in_base_m'] - pivot) * 1000 < 0.5
    assert 0.1 < r['rms_mm'] < 1.0 and r['orientation_spread_deg'] > 30


def test_pivot_needs_four_poses():
    with pytest.raises(ValueError):
        C.pivot_calibration([np.eye(4)] * 3)


def test_handeye_recovers_extrinsics():
    rng = np.random.default_rng(2)
    X = rand_pose(rng, 20, 0.08)                    # true T_tcp_cam
    B_base = rand_pose(rng, 10, 0.5)                # board fixed in base
    A_list, B_list = [], []
    for _ in range(15):
        A = rand_pose(rng, 35, 0.3)
        A_list.append(A)
        B_list.append(np.linalg.inv(A @ X) @ B_base)    # board seen from the camera
    best, allm = C.solve_handeye(A_list, B_list)
    assert np.allclose(best['X'], X, atol=1e-6) and best['rms_mm'] < 1e-3
    assert len(allm) == 5 and C.is_rigid(best['X'])


def test_checkerboard_points():
    g = C.board_points('checkerboard', 9, 7, 0.025)
    assert g.shape == (48, 3) and np.isclose(g[-1, 0], 0.175) and np.isclose(g[-1, 1], 0.125)
