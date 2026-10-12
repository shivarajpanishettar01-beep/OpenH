"""Pose maths. No forward kinematics anywhere: poses come from the controller's TCP.

Conventions
-----------
* Poses are 4x4 homogeneous matrices, or (position[3], quaternion xyzw[4]).
* UR reports orientation as a rotation vector; convert with :func:`pose_from_ur`.
* Probe frame: x along the array, y across it, z along the beam into tissue.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def pose_from_ur(p6) -> np.ndarray:
    """UR [x, y, z, rx, ry, rz] (rotation vector) -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(np.asarray(p6[3:6], float)).as_matrix()
    T[:3, 3] = p6[:3]
    return T


def ur_from_pose(T) -> list:
    rv = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return [float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), *map(float, rv)]


def pose_from_pq(p, q_xyzw) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(q_xyzw).as_matrix()
    T[:3, 3] = p
    return T


def pq_from_pose(T):
    return T[:3, 3].copy(), Rotation.from_matrix(T[:3, :3]).as_quat()


def pose_from_msg(pose_msg) -> np.ndarray:
    p, o = pose_msg.position, pose_msg.orientation
    return pose_from_pq([p.x, p.y, p.z], [o.x, o.y, o.z, o.w])


def fill_pose_msg(pose_msg, T):
    p, q = pq_from_pose(T)
    pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = map(float, p)
    (pose_msg.orientation.x, pose_msg.orientation.y,
     pose_msg.orientation.z, pose_msg.orientation.w) = map(float, q)
    return pose_msg


def inv(T) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def twist_probe_to_base(T_base_probe, v_probe, w_probe):
    """Rotate a probe-frame twist into the base frame (same reference point)."""
    R = T_base_probe[:3, :3]
    return R @ np.asarray(v_probe, float), R @ np.asarray(w_probe, float)


def twist_base_to_probe(T_base_probe, v_base, w_base):
    R = T_base_probe[:3, :3]
    return R.T @ np.asarray(v_base, float), R.T @ np.asarray(w_base, float)


def tilt_deg(T_base_probe, reference_axis_base) -> float:
    """Angle between the probe beam axis (+z) and a reference axis in the base frame."""
    z = T_base_probe[:3, 2]
    r = np.asarray(reference_axis_base, float)
    r = r / np.linalg.norm(r)
    return float(np.degrees(np.arccos(np.clip(z @ r, -1.0, 1.0))))


def kabsch(points_src, points_dst):
    """Rigid transform T (4x4) minimising |T*src - dst|. Returns (T, rms_residual)."""
    A = np.asarray(points_src, float)
    B = np.asarray(points_dst, float)
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = cb - R @ ca
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    res = (R @ A.T).T + t - B
    return T, float(np.sqrt((res ** 2).sum(1).mean()))


class ConstantSpeedPath:
    """Piecewise-linear position, slerped orientation, constant speed along the path.

    Path length combines translation and rotation: a segment that moves dp and rotates by
    angle dθ has length sqrt(|dp|^2 + (rot_radius * dθ)^2), i.e. rotation counts as the motion
    of a point rot_radius from the probe tip. Line sweeps are pure translation (tangential
    speed = speed); fans (rotation about the tip) move at speed / rot_radius rad/s.
    Acceleration ramps (trapezoidal profile) only at the start and the end of the whole path.
    """

    def __init__(self, poses, speed, accel, rot_radius=0.05):
        if len(poses) < 2:
            raise ValueError('a path needs at least two waypoints')
        if speed <= 0 or accel <= 0 or rot_radius <= 0:
            raise ValueError('speed, accel and rot_radius must be positive')
        self.poses = [np.asarray(T, float) for T in poses]
        self.r = float(rot_radius)
        self.seg_dp, self.seg_rv = [], []
        lengths = []
        for A, B in zip(self.poses[:-1], self.poses[1:]):
            dp = B[:3, 3] - A[:3, 3]
            rv = Rotation.from_matrix(B[:3, :3] @ A[:3, :3].T).as_rotvec()
            L = float(np.sqrt(dp @ dp + (self.r ** 2) * (rv @ rv)))
            if L <= 1e-9:
                raise ValueError('consecutive waypoints coincide')
            self.seg_dp.append(dp)
            self.seg_rv.append(rv)
            lengths.append(L)
        self.cum = np.concatenate([[0.0], np.cumsum(lengths)])
        self.length = float(self.cum[-1])
        self.v = float(speed)
        self.a = float(accel)
        t_acc = self.v / self.a
        d_acc = 0.5 * self.a * t_acc ** 2
        if 2 * d_acc > self.length:          # triangular profile: never reaches v
            t_acc = np.sqrt(self.length / self.a)
            self.v = self.a * t_acc
            d_acc = self.length / 2
        self.t_acc = t_acc
        self.d_acc = d_acc
        self.t_cruise = (self.length - 2 * d_acc) / self.v
        self.duration = 2 * t_acc + self.t_cruise

    def s_of_t(self, t):
        if t <= 0:
            return 0.0, 0.0
        if t < self.t_acc:
            return 0.5 * self.a * t * t, self.a * t
        if t < self.t_acc + self.t_cruise:
            return self.d_acc + self.v * (t - self.t_acc), self.v
        if t < self.duration:
            td = self.duration - t
            return self.length - 0.5 * self.a * td * td, self.a * td
        return self.length, 0.0

    def _segment(self, s):
        s = float(np.clip(s, 0.0, self.length))
        i = int(np.clip(np.searchsorted(self.cum, s, side='right') - 1, 0, len(self.poses) - 2))
        u = (s - self.cum[i]) / (self.cum[i + 1] - self.cum[i])
        return i, u

    def pose_at_s(self, s):
        i, u = self._segment(s)
        A = self.poses[i]
        T = np.eye(4)
        T[:3, 3] = A[:3, 3] + u * self.seg_dp[i]
        T[:3, :3] = Rotation.from_rotvec(u * self.seg_rv[i]).as_matrix() @ A[:3, :3]
        return T, i

    def tangent_at_s(self, s):
        """Unit tangent split into (linear m per m of path, angular rad per m of path)."""
        i, _ = self._segment(s)
        L = self.cum[i + 1] - self.cum[i]
        return self.seg_dp[i] / L, self.seg_rv[i] / L

    def sample(self, t):
        s, sd = self.s_of_t(t)
        T, _ = self.pose_at_s(s)
        return T, sd, t >= self.duration


def pose_error(T_cur, T_des):
    """(position error in base [m], rotation error as rotvec in base [rad]) of des relative to cur."""
    dp = T_des[:3, 3] - T_cur[:3, 3]
    dR = T_des[:3, :3] @ T_cur[:3, :3].T
    return dp, Rotation.from_matrix(dR).as_rotvec()
