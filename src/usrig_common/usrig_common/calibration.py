"""Calibration solvers (pure numpy / OpenCV, no ROS), shared by the calibration tools.

pivot_calibration  tool-tip offset from poses that pivot about a fixed point (arm / pointer)
solve_handeye      eye-in-hand camera extrinsics from TCP poses and board poses (wrist camera)
"""
import numpy as np
from scipy.spatial.transform import Rotation


def pivot_calibration(T_base_tcp):
    """Poses T_base_tcp[i] (4x4) with the tool tip resting in one fixed divot.

    Solves R_i p_tip + t_i = p_pivot for p_tip (tip in the TCP frame, m) and p_pivot (divot in the
    base frame, m). Returns dict with tip, pivot, per-pose residuals (mm), rms, and the orientation
    spread (deg), which must be large (tens of degrees) for a well-conditioned solution.
    """
    T = [np.asarray(x, float) for x in T_base_tcp]
    if len(T) < 4:
        raise ValueError('pivot calibration needs at least 4 poses')
    A = np.vstack([np.hstack([t[:3, :3], -np.eye(3)]) for t in T])
    b = -np.concatenate([t[:3, 3] for t in T])
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    tip, pivot = x[:3], x[3:]
    res = np.array([np.linalg.norm(t[:3, :3] @ tip + t[:3, 3] - pivot) for t in T]) * 1000.0
    rots = Rotation.from_matrix(np.array([t[:3, :3] for t in T]))
    mean = rots.mean()
    spread = float(np.degrees(np.max((rots * mean.inv()).magnitude())) * 2)
    cond = float(np.linalg.cond(A))
    return {'tip_in_tcp_m': tip, 'pivot_in_base_m': pivot, 'residuals_mm': res,
            'rms_mm': float(np.sqrt(np.mean(res ** 2))), 'max_mm': float(res.max()),
            'orientation_spread_deg': spread, 'condition': cond, 'n_poses': len(T)}


def board_points(kind, squares_x, squares_y, square_m):
    """3D inner-corner coordinates of a checkerboard (z = 0, board frame)."""
    if kind != 'checkerboard':
        raise ValueError('board_points only for checkerboard')
    nx, ny = squares_x - 1, squares_y - 1
    g = np.zeros((nx * ny, 3), np.float32)
    g[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2) * square_m
    return g


def handeye_residual(T_base_tcp, T_cam_board, X):
    """Consistency of X = T_tcp_cam: the board must sit still in the base frame.
    Returns (rms mm, max mm, mean rotation deviation deg)."""
    boards = [A @ X @ B for A, B in zip(T_base_tcp, T_cam_board)]
    P = np.array([b[:3, 3] for b in boards])
    t_err = np.linalg.norm(P - P.mean(0), axis=1) * 1000.0
    R0 = Rotation.from_matrix(np.array([b[:3, :3] for b in boards])).mean()
    r_err = [np.degrees((Rotation.from_matrix(b[:3, :3]) * R0.inv()).magnitude()) for b in boards]
    return float(np.sqrt(np.mean(t_err ** 2))), float(t_err.max()), float(np.mean(r_err))


HANDEYE_METHODS = ('TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS')


def solve_handeye(T_base_tcp, T_cam_board):
    """All OpenCV hand-eye methods; returns (best, all) where each entry is
    {'method', 'X' (T_tcp_cam), 'rms_mm', 'max_mm', 'rot_deg'} and best has the lowest rms."""
    import cv2
    if len(T_base_tcp) < 3:
        raise ValueError('hand-eye needs at least 3 poses (12+ recommended)')
    out = []
    for name in HANDEYE_METHODS:
        R, t = cv2.calibrateHandEye([a[:3, :3] for a in T_base_tcp], [a[:3, 3] for a in T_base_tcp],
                                    [b[:3, :3] for b in T_cam_board], [b[:3, 3] for b in T_cam_board],
                                    method=getattr(cv2, f'CALIB_HAND_EYE_{name}'))
        X = np.eye(4)
        X[:3, :3], X[:3, 3] = R, np.ravel(t)
        rms, mx, rot = handeye_residual(T_base_tcp, T_cam_board, X)
        out.append({'method': name, 'X': X, 'rms_mm': rms, 'max_mm': mx, 'rot_deg': rot})
    best = min(out, key=lambda r: r['rms_mm'])
    return best, out


def is_rigid(T, tol=1e-6):
    T = np.asarray(T, float)
    if T.shape != (4, 4):
        return False
    R = T[:3, :3]
    return (np.allclose(R.T @ R, np.eye(3), atol=tol * 100) and abs(np.linalg.det(R) - 1) < tol * 100
            and np.allclose(T[3], [0, 0, 0, 1], atol=tol))
