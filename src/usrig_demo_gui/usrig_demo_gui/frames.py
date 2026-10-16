"""Tool frames for the trajectory map: flange (end effector), probe tip and wrist camera.

Everything starts from the controller's TCP pose (/robot/tcp_pose, the same sample as
/robot/state tcp_base, frame `base`) and the rig calibrations, following the repo's rig.yaml:

  robot.tcp_setting                 controller TCP offset [x,y,z,rx,ry,rz]   -> T_flange_tcp
  probe.tcp_already_includes_probe  true: the controller TCP *is* the probe tip
  probe.tip_offset_flange_m         probe tip in the flange frame, used when the TCP is the flange
  calibration.wrist_cam_cali_mtx    4x4 T_flange_camera (hand-eye), also published by rig_static_tf
                                    on /tf_static as flange -> wrist_camera

  flange = T_base_tcp @ inv(T_flange_tcp)
  tip    = T_base_tcp                         if tcp_already_includes_probe
         = flange @ T_flange_tip              otherwise
  camera = flange @ T_flange_camera

A frame whose calibration is still null is returned as None (not guessed), so the GUI can say
"not calibrated" instead of drawing a made-up path.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


def rotvec_to_matrix(rv) -> np.ndarray:
    rv = np.asarray(rv, float)
    th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def pose_from_ur(p6) -> np.ndarray:
    """UR [x, y, z, rx, ry, rz] (rotation vector) -> 4x4 (same as usrig_common.geometry.pose_from_ur)."""
    T = np.eye(4)
    T[:3, :3] = rotvec_to_matrix(p6[3:6])
    T[:3, 3] = np.asarray(p6[:3], float)
    return T


def pose_from_pq(p, q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    T = np.eye(4)
    T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    T[:3, 3] = [float(v) for v in p]
    return T


def as_matrix(v) -> Optional[np.ndarray]:
    """Config value -> 4x4: accepts a 4x4 (nested list), 16 numbers, a UR 6-vector or [x, y, z]."""
    if v is None:
        return None
    a = np.asarray(v, float).ravel()
    if a.size == 16:
        return a.reshape(4, 4)
    if a.size == 6:
        return pose_from_ur(a)
    if a.size == 3:
        T = np.eye(4)
        T[:3, 3] = a
        return T
    raise ValueError(f'expected 4x4, 16, 6 or 3 numbers, got {a.size}')


@dataclass
class ToolCalibration:
    """What is needed to go from the controller TCP to the three tracked frames."""
    T_flange_tcp: Optional[np.ndarray] = None       # None = not known yet (tcp_setting null)
    tcp_is_probe_tip: Optional[bool] = None         # None = not known yet
    T_flange_tip: Optional[np.ndarray] = None
    T_flange_camera: Optional[np.ndarray] = None
    camera_source: str = ''                         # '/tf_static', 'rig.yaml', 'mock' ...

    @classmethod
    def from_rig(cls, rig: dict, camera_from_tf: Optional[np.ndarray] = None):
        robot, probe, cal = rig.get('robot') or {}, rig.get('probe') or {}, rig.get('calibration') or {}
        cam, src = camera_from_tf, '/tf_static'
        if cam is None and cal.get('wrist_cam_cali_mtx') is not None:
            cam, src = as_matrix(cal['wrist_cam_cali_mtx']), 'rig.yaml'
        return cls(T_flange_tcp=as_matrix(robot.get('tcp_setting')),
                   tcp_is_probe_tip=probe.get('tcp_already_includes_probe'),
                   T_flange_tip=as_matrix(probe.get('tip_offset_flange_m')),
                   T_flange_camera=cam, camera_source=src if cam is not None else '')

    def missing(self):
        """Names of the rig.yaml entries still needed, per frame."""
        out = {}
        if self.T_flange_tcp is None:
            out['flange'] = 'robot.tcp_setting'
        if self.tcp_is_probe_tip is None:
            out['tip'] = 'probe.tcp_already_includes_probe'
        elif not self.tcp_is_probe_tip and self.T_flange_tip is None:
            out['tip'] = 'probe.tip_offset_flange_m'
        if self.T_flange_camera is None:
            out['camera'] = 'calibration.wrist_cam_cali_mtx (or flange -> wrist_camera on /tf_static)'
        return out


@dataclass
class ToolFrames:
    flange: Optional[np.ndarray] = None
    tip: Optional[np.ndarray] = None
    camera: Optional[np.ndarray] = None


def tool_frames(T_base_tcp: Optional[np.ndarray], cal: ToolCalibration) -> ToolFrames:
    if T_base_tcp is None:
        return ToolFrames()
    out = ToolFrames()
    if cal.T_flange_tcp is not None:
        out.flange = T_base_tcp @ np.linalg.inv(cal.T_flange_tcp)
    if cal.tcp_is_probe_tip is True:
        out.tip = T_base_tcp.copy()
    elif cal.tcp_is_probe_tip is False and out.flange is not None and cal.T_flange_tip is not None:
        out.tip = out.flange @ cal.T_flange_tip
    if out.flange is not None and cal.T_flange_camera is not None:
        out.camera = out.flange @ cal.T_flange_camera
    return out
