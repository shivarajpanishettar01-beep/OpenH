"""UR7e kinematics for drawing only.

The recorded robot data never uses this: the TCP pose shown in the GUI comes from the controller
(/robot/tcp_pose) when running on ROS. Here FK only places the drawn links, and the small IK is
used by the mock source to produce believable joint angles.

ASSUMPTION: the UR7e uses the UR5e arm geometry (same 850 mm reach); DH values below are the
published UR5e ones. Replace with the calibrated values from the controller / URDF if needed.
"""
from __future__ import annotations

import numpy as np

DH_D = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
DH_A = np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0])
DH_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

# Probe tip offset along the flange z axis. rig.yaml robot.tcp_setting is still null in the repo,
# so this is a display guess (probe + holder length).
PROBE_LEN_M = 0.13


def _dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_frames(q, probe_len=PROBE_LEN_M):
    """Return 8 homogeneous transforms: base, joint frames 1..6 (flange last), probe tip."""
    T = np.eye(4)
    out = [T.copy()]
    for i in range(6):
        T = T @ _dh(q[i], DH_D[i], DH_A[i], DH_ALPHA[i])
        out.append(T.copy())
    tip = T.copy()
    tip[:3, 3] = T[:3, 3] + T[:3, 2] * probe_len
    out.append(tip)
    return out


def tcp(q, probe_len=PROBE_LEN_M):
    return fk_frames(q, probe_len)[-1]


def rotvec_from_matrix(R):
    """Axis-angle vector (rad), as on the UR pendant."""
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    if np.pi - th < 1e-6:
        # 180 deg: axis from the diagonal
        ax = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0))
        i = int(np.argmax(ax))
        if ax[i] > 0:
            for j in range(3):
                if j != i:
                    ax[j] = np.copysign(ax[j], R[i, j] + R[j, i])
        return ax / np.linalg.norm(ax) * th
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / (2 * np.sin(th)) * th


def ik_point_down(p, z_dir, q0, iters=40, probe_len=PROBE_LEN_M):
    """Damped least squares for tip position p and tool z axis z_dir; joint 6 held near q0[5].

    Good enough for the mock trajectory (small moves from a nearby seed). Not for control.
    """
    q = np.array(q0, dtype=float)
    z_dir = np.asarray(z_dir, float) / np.linalg.norm(z_dir)
    lam = 0.05

    def err(qq):
        T = tcp(qq, probe_len)
        ep = np.asarray(p) - T[:3, 3]
        er = np.cross(T[:3, 2], z_dir)
        return np.concatenate([ep, 0.3 * er, [0.05 * (q0[5] - qq[5])]])

    for _ in range(iters):
        e = err(q)
        if np.linalg.norm(e[:3]) < 1e-5 and np.linalg.norm(e[3:6]) < 1e-5:
            break
        J = np.zeros((7, 6))
        h = 1e-6
        for k in range(6):
            dq = np.zeros(6)
            dq[k] = h
            J[:, k] = -(err(q + dq) - e) / h
        dq = np.linalg.solve(J.T @ J + lam ** 2 * np.eye(6), J.T @ e)
        q = q + dq
    return q


# A seed with the probe pointing straight down in front of the robot (checked by the tests).
Q_SEED = np.radians([180.0, -100.0, 110.0, -100.0, -90.0, 0.0])


# ---------------------------------------------------------------- URDF chain (for drawing meshes)
# Joint origins from ur_description config/ur7e/default_kinematics.yaml. Frames are expressed in
# base_link_inertia, which is the controller's base frame (same frame as the DH chain above).
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _T(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _Rz(q):
    return _T(rpy=(0, 0, q))


_PI = np.pi
URDF_JOINTS = [   # (child link, joint origin)
    ('shoulder', _T((0, 0, 0.1625))),
    ('upper_arm', _T((0, 0, 0), (_PI / 2, 0, 0))),
    ('forearm', _T((-0.425, 0, 0))),
    ('wrist_1', _T((-0.3922, 0, 0.1333))),
    ('wrist_2', _T((0, -0.0997, 0), (_PI / 2, 0, 0))),
    ('wrist_3', _T((0, 0.0996, 0), (_PI / 2, _PI, _PI))),
]
_TOOL0 = _T(rpy=(0, -_PI / 2, -_PI / 2)) @ _T(rpy=(_PI / 2, 0, _PI / 2))   # wrist_3 -> flange -> tool0


def urdf_frames(q):
    """{'base': I, 'shoulder': T, ..., 'wrist_3': T, 'tool0': T} in the controller base frame."""
    out = {'base': np.eye(4)}
    T = np.eye(4)
    for (name, origin), qi in zip(URDF_JOINTS, q):
        T = T @ origin @ _Rz(qi)
        out[name] = T
    out['tool0'] = T @ _TOOL0
    return out
