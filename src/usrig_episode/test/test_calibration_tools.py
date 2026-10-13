import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from usrig_common import geometry
from usrig_episode import handeye

REPO = Path(__file__).resolve().parents[3]
BOARD = {'type': 'checkerboard', 'squares_x': 9, 'squares_y': 7, 'square_mm': 25.0, 'marker_mm': None, 'dictionary': None}
K = np.array([[615.0, 0, 320.0], [0, 615.0, 240.0], [0, 0, 1]])


def render_checkerboard(T_cam_board, size=(640, 480)):
    """Render the board seen by a pinhole camera (planar homography of a fine board image)."""
    px = 40                                             # pixels per square in the source image
    nx, ny = BOARD['squares_x'], BOARD['squares_y']
    src = np.full(((ny + 2) * px, (nx + 2) * px), 255, np.uint8)
    for j in range(ny):
        for i in range(nx):
            if (i + j) % 2 == 0:
                src[(j + 1) * px:(j + 2) * px, (i + 1) * px:(i + 2) * px] = 0
    s = BOARD['square_mm'] / 1000.0
    # source pixel (u, v) -> board point ((u/px - 1 - 1) * s, ...) : inner corner (0,0) is at square (1,1)
    S = np.array([[s / px, 0, -2 * s], [0, s / px, -2 * s], [0, 0, 1]])
    R, t = T_cam_board[:3, :3], T_cam_board[:3, 3]
    H = K @ np.column_stack([R[:, 0], R[:, 1], t]) @ S
    return cv2.warpPerspective(src, H, size, borderValue=255)


def test_board_pose_from_rendered_image():
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler('xyz', [15, -10, 5], degrees=True).as_matrix()
    T[:3, 3] = [-0.08, -0.05, 0.45]
    bgr = cv2.cvtColor(render_checkerboard(T), cv2.COLOR_GRAY2BGR)
    det = handeye.BoardDetector(BOARD)
    T_est, n, rms = det.pose(bgr, K, np.zeros(5))
    assert n == 48 and rms < 0.5
    assert np.linalg.norm(T_est[:3, 3] - T[:3, 3]) * 1000 < 2.0
    assert np.degrees(Rotation.from_matrix(T_est[:3, :3] @ T[:3, :3].T).magnitude()) < 0.5


def test_unset_board_refuses():
    from usrig_common.config import UnsetLimitError
    with pytest.raises(UnsetLimitError, match='calib_board.square_mm'):
        handeye.BoardDetector({'type': 'checkerboard', 'squares_x': 9, 'squares_y': 7, 'square_mm': None})


def test_resolve_writes_rig_yaml(tmp_path):
    rng = np.random.default_rng(3)
    tcp = [0.0, 0.0, 0.18, 0.0, 0.0, 0.0]              # active TCP 18 cm along flange z
    X = np.eye(4)
    X[:3, :3] = Rotation.from_euler('xyz', [180, 0, 90], degrees=True).as_matrix()
    X[:3, 3] = [0.05, 0.0, -0.12]                       # true T_tcp_camera
    B_base = np.eye(4)
    B_base[:3, 3] = [0.5, -0.1, 0.0]
    caps = []
    for _ in range(14):
        A = np.eye(4)
        A[:3, :3] = Rotation.from_rotvec(np.radians(30) * rng.uniform(-1, 1, 3)).as_matrix() @ \
            Rotation.from_euler('x', 180, degrees=True).as_matrix()
        A[:3, 3] = [0.5, -0.1, 0.55] + rng.uniform(-0.1, 0.1, 3)
        caps.append({'T_base_tcp': A.tolist(), 'T_cam_board': (np.linalg.inv(A @ X) @ B_base).tolist()})
    cap_dir = tmp_path / 'handeye_x'
    cap_dir.mkdir()
    (cap_dir / 'captures.json').write_text(json.dumps({'board': BOARD, 'tcp_setting': tcp, 'captures': caps}))
    cfg = tmp_path / 'config'
    shutil.copytree(REPO / 'src/usrig_bringup/config', cfg)
    sys.argv = ['handeye_calib', '--resolve', str(cap_dir), '--write-rig', '--config-dir', str(cfg)]
    handeye.main()
    rig = yaml.safe_load((cfg / 'rig.yaml').read_text())
    T_fc = np.array(rig['calibration']['wrist_cam_cali_mtx'])
    assert np.allclose(T_fc, geometry.pose_from_ur(tcp) @ X, atol=1e-6)
    info = rig['calibration']['wrist_cam_cali_info']
    assert info['n_poses'] == 14 and info['residual_rms_mm'] < 0.01 and info['captures'] == str(cap_dir)


def test_pointer_tip_in_base():
    tcp = [0.0, 0.0, 0.18, 0.0, 0.0, 0.0]
    T_base_tcp = geometry.pose_from_ur([0.4, 0.1, 0.3, 0.0, np.pi, 0.0])
    tip = geometry.tool_tip_in_base(T_base_tcp, tcp, [0.03, 0.0, 0.10])
    T_base_flange = T_base_tcp @ geometry.inv(geometry.pose_from_ur(tcp))
    assert np.allclose(tip[:3, 3], T_base_flange[:3, :3] @ [0.03, 0, 0.10] + T_base_flange[:3, 3])
