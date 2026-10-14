"""Eye-in-hand calibration of the wrist D415 (camera w.r.t. the arm's flange).

  ros2 run usrig_episode handeye_calib [--session <dir>] [--write-rig] [--min-poses 12]
  ros2 run usrig_episode handeye_calib --resolve <capture_dir> [--write-rig]   # re-solve saved captures

Board: session.yaml wrist_camera.calib_board (checkerboard or ChArUco; null until set). Fix the board
on the table. You move the robot (freedrive or pendant) to 12-20 poses that see the whole board from
different angles (tilt 20-40 deg, rotate about the camera axis, vary distance); this tool only reads.
At each pose press Enter: it grabs the colour image, camera_info and the TCP pose, detects the board
and stores the pair. `q` + Enter solves with every OpenCV hand-eye method and keeps the one with the
lowest residual (spread of the board pose in the base frame).

Every capture (PNG + poses + intrinsics) is saved in <root>/calibration/handeye_<date>/ so the result
can be re-solved or audited later. The result T_tcp_camera is converted to T_flange_camera with
rig.yaml robot.tcp_setting (the end-effector frame of the dataset). --write-rig stores it in rig.yaml
calibration.wrist_cam_cali_mtx with a provenance record; --session also stores it in that
session.json. rig_static_tf publishes it on /tf_static. Redo whenever the camera mount is touched.
"""
import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np

from usrig_common import calibration as C
from usrig_common import config, geometry
from usrig_common.session import update_session_json
from usrig_common.timeutil import now_ns


def image_to_bgr(m):
    import cv2
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)
    if m.encoding in ('rgb8', 'bgr8'):
        a = a[:, :m.width * 3].reshape(m.height, m.width, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()
    return cv2.cvtColor(a[:, :m.width].copy(), cv2.COLOR_GRAY2BGR)


class BoardDetector:
    """Board pose in the camera frame from one image (checkerboard or ChArUco)."""

    def __init__(self, board_cfg):
        import cv2
        need = ['type', 'squares_x', 'squares_y', 'square_mm'] + (
            ['marker_mm', 'dictionary'] if board_cfg.get('type') == 'charuco' else [])
        missing = [k for k in need if board_cfg.get(k) is None]
        if missing:
            raise config.UnsetLimitError([f'wrist_camera.calib_board.{k}' for k in missing], 'session.yaml')
        self.cfg = board_cfg
        self.kind = board_cfg['type']
        sq = board_cfg['square_mm'] / 1000.0
        nx, ny = int(board_cfg['squares_x']), int(board_cfg['squares_y'])
        if self.kind == 'checkerboard':
            self.pattern = (nx - 1, ny - 1)
            self.obj = C.board_points('checkerboard', nx, ny, sq)
        elif self.kind == 'charuco':
            d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, board_cfg['dictionary']))
            self.board = cv2.aruco.CharucoBoard((nx, ny), sq, board_cfg['marker_mm'] / 1000.0, d)
            self.detector = cv2.aruco.CharucoDetector(self.board)
        else:
            raise ValueError("calib_board.type must be 'checkerboard' or 'charuco'")

    def pose(self, bgr, K, D):
        """Returns (T_cam_board 4x4, n_points, reprojection rms px) or (None, reason)."""
        import cv2
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.kind == 'checkerboard':
            ok, corners = cv2.findChessboardCorners(gray, self.pattern,
                                                    cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ok:
                return None, 'checkerboard not found (whole board must be visible)'
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4))
            obj, img = self.obj, corners.reshape(-1, 2)
        else:
            corners, ids, _, _ = self.detector.detectBoard(gray)
            if ids is None or len(ids) < 6:
                return None, 'ChArUco board not found (need >= 6 corners)'
            obj, img = self.board.matchImagePoints(corners, ids)
            obj, img = obj.reshape(-1, 3), img.reshape(-1, 2)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return None, 'solvePnP failed'
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.ravel()
        return T, len(obj), rms


def solve_and_report(A, B, tcp_setting):
    best, allm = C.solve_handeye(A, B)
    for r in allm:
        print(f'{r["method"]:10s} residual rms {r["rms_mm"]:.2f} mm, max {r["max_mm"]:.2f} mm, rot {r["rot_deg"]:.2f} deg')
    X = best['X']
    T_flange_cam = geometry.pose_from_ur(tcp_setting) @ X if tcp_setting is not None else None
    print(f'\nbest: {best["method"]}, rms {best["rms_mm"]:.2f} mm over {len(A)} poses')
    print('T_tcp_camera:\n' + np.array2string(X, precision=5, suppress_small=True))
    if T_flange_cam is not None:
        print('T_flange_camera (stored as wrist_cam_cali_mtx):\n' +
              np.array2string(T_flange_cam, precision=5, suppress_small=True))
    else:
        print('rig.yaml robot.tcp_setting is null: cannot express the result in the flange frame')
    return best, T_flange_cam


def store(a, rig_file, best, T_flange_cam, n, capture_dir, board_cfg):
    info = {'date': time.strftime('%Y-%m-%d %H:%M'), 'method': best['method'], 'n_poses': n,
            'residual_rms_mm': round(best['rms_mm'], 3), 'residual_max_mm': round(best['max_mm'], 3),
            'residual_rot_deg': round(best['rot_deg'], 3), 'board': board_cfg,
            'frame': 'T_flange_camera (colour optical frame in the flange frame)', 'captures': str(capture_dir)}
    if a.write_rig:
        if T_flange_cam is None:
            raise SystemExit('not written: robot.tcp_setting is null')
        config.write_yaml_key(rig_file, 'wrist_cam_cali_mtx', np.round(T_flange_cam, 7).tolist())
        config.write_yaml_key(rig_file, 'wrist_cam_cali_info', info)
        print(f'written to {rig_file}: calibration.wrist_cam_cali_mtx + wrist_cam_cali_info')
    if a.session:
        update_session_json(a.session, wrist_cam_cali_mtx=None if T_flange_cam is None else T_flange_cam.tolist(),
                            wrist_cam_cali={**info, 'T_tcp_camera': best['X'].tolist(), 'stamp_ns': now_ns()})
        print(f'stored in {Path(a.session) / "session.json"}')


def resolve(a, rig, rig_file):
    d = Path(a.resolve).expanduser()
    meta = json.loads((d / 'captures.json').read_text())
    A = [np.array(c['T_base_tcp']) for c in meta['captures']]
    B = [np.array(c['T_cam_board']) for c in meta['captures']]
    best, Tf = solve_and_report(A, B, meta.get('tcp_setting'))
    store(a, rig_file, best, Tf, len(A), d, meta.get('board'))


def capture(a, rig, rig_file, scfg):
    import cv2
    import rclpy
    from sensor_msgs.msg import CameraInfo, Image
    from usrig_interfaces.msg import RobotState
    det = BoardDetector(config.get_path(scfg, 'wrist_camera.calib_board') or {})
    topics = scfg['wrist_camera']['topics']
    rclpy.init()
    node = rclpy.create_node('handeye_calib')
    latest, lock = {}, threading.Lock()

    def keep(k):
        def f(m):
            with lock:
                latest[k] = m
        return f
    node.create_subscription(Image, topics['color'], keep('img'), 2)
    node.create_subscription(CameraInfo, topics['color_info'], keep('info'), 2)
    node.create_subscription(RobotState, '/robot/state', keep('state'), 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    stamp = time.strftime('%Y%m%d_%H%M%S')
    from usrig_common.session import data_dir, session_is_sim
    sim = session_is_sim(a.session) if a.session else a.sim
    out = data_dir(scfg, 'calibration', sim) / f'handeye_{stamp}'
    out.mkdir(parents=True, exist_ok=True)
    caps = []
    try:
        while True:
            cmd = input(f'[{len(caps)} poses] move the robot, Enter = capture, q = solve: ').strip().lower()
            if cmd == 'q':
                break
            time.sleep(0.5)   # let the arm and the image settle
            with lock:
                img, info, st = latest.get('img'), latest.get('info'), latest.get('state')
            if img is None or info is None or st is None:
                print('  missing image / camera_info / robot state')
                continue
            T_tcp = geometry.pose_from_msg(st.tcp_base)
            K, D = np.array(info.k).reshape(3, 3), np.array(info.d)
            bgr = image_to_bgr(img)
            T_cb, n, rms = det.pose(bgr, K, D)
            if T_cb is None:
                print(f'  {n}')
                continue
            i = len(caps)
            cv2.imwrite(str(out / f'pose_{i:02d}.png'), bgr)
            caps.append({'T_base_tcp': T_tcp.tolist(), 'T_cam_board': T_cb.tolist(), 'points': n,
                         'reproj_rms_px': rms, 'image': f'pose_{i:02d}.png',
                         'robot_stamp': [st.header.stamp.sec, st.header.stamp.nanosec]})
            print(f'  captured: {n} points, reprojection {rms:.2f} px, camera-board distance '
                  f'{np.linalg.norm(T_cb[:3, 3]) * 1000:.0f} mm')
            (out / 'captures.json').write_text(json.dumps({
                'board': det.cfg, 'tcp_setting': config.get_path(rig, 'robot.tcp_setting'),
                'K': K.tolist(), 'D': D.tolist(), 'image_size': [img.width, img.height],
                'camera_frame': info.header.frame_id, 'captures': caps}, indent=1))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if len(caps) < a.min_poses:
        raise SystemExit(f'only {len(caps)} poses (need {a.min_poses}); captures kept in {out}')
    A = [np.array(c['T_base_tcp']) for c in caps]
    B = [np.array(c['T_cam_board']) for c in caps]
    best, Tf = solve_and_report(A, B, config.get_path(rig, 'robot.tcp_setting'))
    store(a, rig_file, best, Tf, len(caps), out, det.cfg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--session', help='also store the result in <session>/session.json')
    ap.add_argument('--write-rig', action='store_true', help='store the result in rig.yaml (all sessions)')
    ap.add_argument('--resolve', help='re-solve a saved capture directory (no robot / camera needed)')
    ap.add_argument('--min-poses', type=int, default=12)
    ap.add_argument('--sim', action='store_true', help='simulation (captures under session.sim_root)')
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    d = Path(a.config_dir) if a.config_dir else config.default_config_dir()
    rig_file = d / 'rig.yaml'
    rig = config.load_yaml(rig_file)
    scfg = config.load_config('session', a.config_dir)
    try:
        if a.resolve:
            resolve(a, rig, rig_file)
        else:
            capture(a, rig, rig_file, scfg)
    except (config.UnsetLimitError, ValueError) as e:
        raise SystemExit(f'[handeye_calib] {e}')


if __name__ == '__main__':
    main()
