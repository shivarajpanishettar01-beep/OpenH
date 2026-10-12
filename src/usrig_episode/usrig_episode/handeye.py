"""Eye-in-hand calibration of the wrist D415 with a ChArUco board.

  ros2 run usrig_episode handeye_calib <session_dir> [--min-poses 12]

The user moves the robot (freedrive or pendant) to varied poses that see the board; this tool
only reads. At each pose press Enter: it grabs the colour image, camera_info and the TCP pose,
detects the board and stores the pair. 'q' + Enter solves cv2.calibrateHandEye (all methods),
reports the residual (spread of the board pose in the base frame across captures) and saves the
best result as wrist_cam_cali_mtx (T_tcp_camera, 4x4) in session.json.
Board geometry: session.yaml wrist_camera.charuco (null until set). Redo whenever the mount moves.
"""
import argparse
import threading
import time
from pathlib import Path

import numpy as np
from sensor_msgs.msg import CameraInfo, Image

from usrig_common import config, geometry
from usrig_common.session import update_session_json
from usrig_common.timeutil import now_ns

from .tool_node import run_tool

METHODS = {'TSAI': 'CALIB_HAND_EYE_TSAI', 'PARK': 'CALIB_HAND_EYE_PARK', 'HORAUD': 'CALIB_HAND_EYE_HORAUD',
           'ANDREFF': 'CALIB_HAND_EYE_ANDREFF', 'DANIILIDIS': 'CALIB_HAND_EYE_DANIILIDIS'}


def image_to_bgr(m):
    import cv2
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)
    ch = 3 if m.encoding in ('rgb8', 'bgr8') else 1
    a = a[:, :m.width * ch].reshape(m.height, m.width, ch) if ch == 3 else a[:, :m.width]
    if m.encoding == 'rgb8':
        a = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    return a


def residual(T_base_tcp, T_cam_board, X):
    boards = [A @ X @ B for A, B in zip(T_base_tcp, T_cam_board)]
    P = np.array([b[:3, 3] for b in boards])
    t_err = np.linalg.norm(P - P.mean(0), axis=1) * 1000.0
    from scipy.spatial.transform import Rotation
    R0 = boards[0][:3, :3]
    r_err = [np.degrees(np.linalg.norm(Rotation.from_matrix(b[:3, :3] @ R0.T).as_rotvec())) for b in boards]
    return float(np.sqrt((t_err ** 2).mean())), float(np.max(t_err)), float(np.mean(r_err))


def _calib(node, a):
    import cv2
    ch = config.get_path(node.scfg, 'wrist_camera.charuco')
    if not ch or any(v is None for v in ch.values()):
        raise config.UnsetLimitError(['wrist_camera.charuco.*'], 'session.yaml')
    topics = node.scfg['wrist_camera']['topics']
    latest = {}
    lock = threading.Lock()
    node.create_subscription(Image, topics['color'], lambda m: latest.update(img=m), 2)
    node.create_subscription(CameraInfo, topics['color_info'], lambda m: latest.update(info=m), 2)
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ch['dictionary']))
    board = cv2.aruco.CharucoBoard((int(ch['squares_x']), int(ch['squares_y'])),
                                   ch['square_mm'] / 1000.0, ch['marker_mm'] / 1000.0, dictionary)
    detector = cv2.aruco.CharucoDetector(board)
    A, B = [], []
    while True:
        cmd = input(f'[{len(A)} poses] move the robot, Enter = capture, q = solve: ').strip().lower()
        if cmd == 'q':
            break
        time.sleep(0.3)   # let the image settle after the motion stops
        with lock:
            img, info = latest.get('img'), latest.get('info')
        if img is None or info is None:
            print('  no image / camera_info yet')
            continue
        T_tcp = geometry.pose_from_msg(node.state().tcp_base)
        bgr = image_to_bgr(img)
        corners, ids, _, _ = detector.detectBoard(bgr)
        if ids is None or len(ids) < 6:
            print('  board not detected (need >= 6 corners)')
            continue
        obj, imgp = board.matchImagePoints(corners, ids)
        K = np.array(info.k).reshape(3, 3)
        D = np.array(info.d)
        ok, rvec, tvec = cv2.solvePnP(obj, imgp, K, D)
        if not ok:
            print('  solvePnP failed')
            continue
        Tcb = np.eye(4)
        Tcb[:3, :3] = cv2.Rodrigues(rvec)[0]
        Tcb[:3, 3] = tvec.ravel()
        A.append(T_tcp)
        B.append(Tcb)
        print(f'  captured {len(ids)} corners')
    if len(A) < a.min_poses:
        raise ValueError(f'only {len(A)} poses, need {a.min_poses}')
    best = None
    for name, flag in METHODS.items():
        R, t = cv2.calibrateHandEye([x[:3, :3] for x in A], [x[:3, 3] for x in A],
                                    [x[:3, :3] for x in B], [x[:3, 3] for x in B], method=getattr(cv2, flag))
        X = np.eye(4)
        X[:3, :3], X[:3, 3] = R, t.ravel()
        rms, mx, rot = residual(A, B, X)
        print(f'{name:10s} residual rms {rms:.2f} mm, max {mx:.2f} mm, rot {rot:.2f} deg')
        if best is None or rms < best[1]:
            best = (name, rms, mx, rot, X)
    name, rms, mx, rot, X = best
    update_session_json(a.session, wrist_cam_cali_mtx=X.tolist(), wrist_cam_cali={
        'frame': 'T_tcp_camera (camera colour optical frame in the controller TCP frame)',
        'method': name, 'residual_rms_mm': rms, 'residual_max_mm': mx, 'residual_rot_deg': rot,
        'n_poses': len(A), 'stamp_ns': now_ns()})
    print(f'saved {name} result to {Path(a.session) / "session.json"} (wrist_cam_cali_mtx)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--min-poses', type=int, default=12)
    a, _ = ap.parse_known_args()
    run_tool(lambda n: _calib(n, a), 'handeye_calib')
