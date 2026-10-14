"""rig_status: what is set up, connected and running, per staged test (docs/TESTING_STEPS.md).

  ros2 run usrig_qc rig_status            # read-only: never commands anything, never changes a file

Run it any time, also with nothing connected. It lists, per test, what is ready (ok), what is missing
in the configuration (SET: the key to fill), what is not connected (CONNECT), and rig nodes that are
already running (left over from an earlier launch, they would mix with a new one).
"""
import os
import shutil
import socket
import subprocess
import time
from pathlib import Path

from usrig_common import config
from usrig_common.session import data_root

from .usb import realsense_usb

OK, SET, CONNECT, INFO = 'ok', 'SET', 'CONNECT', 'info'


def tcp_open(ip, port, timeout=1.5):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def remote_control(ip):
    try:
        with socket.create_connection((ip, 29999), timeout=1.5) as s:
            s.recv(1024)
            s.sendall(b'is in remote control\n')
            return s.recv(1024).decode().strip().lower().startswith('true')
    except OSError:
        return None


def running_rig_nodes(wait_s=1.5):
    """Names of usrig / micrus / camera nodes already on the ROS graph (rclpy, read-only)."""
    try:
        import rclpy
        rclpy.init()
        n = rclpy.create_node('rig_status')
        time.sleep(wait_s)
        names = sorted({nm for nm, _ in n.get_node_names_and_namespaces()} - {'rig_status'})
        n.destroy_node()
        rclpy.shutdown()
    except Exception:  # noqa: BLE001
        return None
    keys = ('robot', 'qc_monitor', 'rig_static_tf', 'episode_manager', 'keyboard_teleop', 'spacemouse', 'micrus',
            'd415', 'camera', 'dashboard', 'rosbag2', 'status_console', 'teleop')
    return [x for x in names if any(k in x for k in keys)]


def nulls(cfg, keys):
    return [k for k in keys if config.get_path(cfg, k) is None]


def rows(scfg, rig, km):
    out = []

    def add(test, item, status, detail=''):
        out.append((test, item, status, detail))

    # ---- software
    prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    add('software', 'workspace sourced', OK if 'usrig_bringup' in prefix else SET,
        'run: cd ~/OpenH && source scripts/env.sh' if 'usrig_bringup' not in prefix else '')
    add('software', 'conda env openh', OK if os.environ.get('CONDA_DEFAULT_ENV') == 'openh' else SET,
        os.environ.get('CONDA_DEFAULT_ENV', 'none'))
    try:
        from ament_index_python.packages import get_package_prefix
        cpp = (Path(get_package_prefix('usrig_robot_cpp')) / 'lib/usrig_robot_cpp/robot_state_cpp').exists()
    except Exception:  # noqa: BLE001
        cpp = False
    add('software', 'robot_state_cpp built', OK if cpp else SET, '' if cpp else 'bash scripts/build.sh')

    # ---- test 1: robot
    ip = config.get_path(rig, 'robot.ip')
    if ip is None:
        add('1 robot', 'robot.ip', SET, 'rig.yaml robot.ip (controller address)')
    else:
        reach = tcp_open(ip, 30004)
        add('1 robot', f'robot {ip} reachable (RTDE 30004)', OK if reach else CONNECT,
            '' if reach else 'cable / PC address on the robot subnet / controller on')
        if reach:
            rc = remote_control(ip)
            add('1 robot', 'Remote Control', OK if rc else INFO,
                'on' if rc else 'off (needed only for robot_control; reading works without)')
    for k, where in (('robot.tcp_setting', 'pendant: Installation > TCP'), ('robot.payload_kg', 'pendant: Payload'),
                     ('robot.payload_cog_m', 'pendant: Payload'), ('probe.type', 'probe model, e.g. MPRO-L40S')):
        v = config.get_path(rig, k)
        add('1 robot', k, OK if v is not None else SET,
            '' if v is not None else f'{where} (set before saving poses: they store it)')
    fd = config.get_path(rig, 'freedrive.input')
    add('1 robot', 'freedrive.input (pedal, optional)', OK if fd is not None else INFO,
        '' if fd is not None else 'only for freedrive collection')

    # ---- test 2: micrus
    mc = Path(os.path.expanduser(scfg['micrus']['command']))
    found = shutil.which(str(mc)) or (mc.exists() and str(mc))
    add('2 ultrasound', 'micrus command', OK if found else SET, str(found or mc))
    lic = config.get_path(scfg, 'micrus.licence_file')
    add('2 ultrasound', 'micrus.licence_file', SET if lic is None else (OK if Path(lic).expanduser().exists() else SET),
        '' if lic and Path(lic).expanduser().exists() else 'session.yaml (micrus machine-id)')
    vid = config.get_path(scfg, 'micrus.usb_vendor_id')
    if vid is None:
        add('2 ultrasound', 'micrus.usb_vendor_id', SET, 'session.yaml, from lsusb with the scanner plugged in')
    else:
        lsusb = subprocess.run(['lsusb'], capture_output=True, text=True).stdout
        add('2 ultrasound', 'MicrUs on USB', OK if f'ID {vid}:' in lsusb else CONNECT, vid)
    pid = config.get_path(scfg, 'session.phantom_id')
    add('2 ultrasound', 'session.phantom_id', OK if pid else SET, pid or f'one of {sorted(scfg.get("phantoms") or {})}')
    if pid:
        pf = data_root(scfg) / 'phantoms' / f'{pid}.yaml'
        add('2 ultrasound', 'phantom fiducial file', OK if pf.exists() else SET, str(pf))
    for k in ('registration_max_residual_mm', 'pointer.tip_offset_flange_m'):
        v = config.get_path(rig, k)
        add('2 ultrasound', k, OK if v is not None else SET, '' if v is not None else
            ('rig.yaml' if k.startswith('registration') else 'pivot_calib --tool pointer --write'))
    add('2 ultrasound', 'calibration.prob_cali_mtx', OK if config.get_path(rig, 'calibration.prob_cali_mtx') is not None
        else INFO, 'pending the lab calibration file (needed for export only)')

    # ---- test 3: camera
    cam = scfg['wrist_camera']
    for k in ('serial', 'exposure_us', 'white_balance_k'):
        v = cam.get(k)
        add('3 camera', f'wrist_camera.{k}', OK if v is not None else SET,
            '' if v is not None else ('camera_autolock --write' if k != 'serial' else 'rs-enumerate-devices -s'))
    board = config.get_path(scfg, 'wrist_camera.calib_board.type')
    add('3 camera', 'wrist_camera.calib_board', OK if board else SET, board or 'for hand-eye')
    devs = realsense_usb(cam.get('serial'))
    if not devs:
        add('3 camera', 'D415 on USB', CONNECT, 'not on the bus')
    else:
        sp = devs[0]['speed_mbps']
        add('3 camera', 'D415 on USB 3', OK if sp >= 5000 else CONNECT, f'{sp} Mbit/s' + ('' if sp >= 5000 else ': use a USB 3 port'))
    add('3 camera', 'hand-eye (calibration.wrist_cam_cali_mtx)',
        OK if config.get_path(rig, 'calibration.wrist_cam_cali_mtx') is not None else INFO, 'made in test 3')

    # ---- test 4: everything, moving the arm
    miss = nulls(rig, config.MOTION_LIMITS)
    add('4 full', 'rig.yaml limits (robot_control)', OK if not miss else SET,
        f'{len(miss)} null: {", ".join(miss[:4])}{" ..." if len(miss) > 4 else ""}' if miss else '')
    sp = [k for k, v in (km.get('speeds') or {}).items() if v is None]
    add('4 full', 'keymap.yaml speeds (keyboard teleop)', OK if not sp else SET, ', '.join(sp))
    root = data_root(scfg)
    probe = root if root.exists() else root.parent
    while not probe.exists():
        probe = probe.parent
    free = shutil.disk_usage(probe).free / 1e9
    need = float(scfg['session'].get('min_free_disk_gb', 50))
    add('4 full', f'free disk for {root}', OK if free >= need else SET, f'{free:.0f} GB (QC needs > {need:g} GB)')
    return out


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config-dir')
    ap.add_argument('--no-graph', action='store_true', help='skip the running-node check')
    a = ap.parse_args(argv)
    scfg = config.load_config('session', a.config_dir)
    rig = config.load_config('rig', a.config_dir)
    km = config.load_config('keymap', a.config_dir)
    table = rows(scfg, rig, km)
    w = max(len(r[1]) for r in table)
    last = None
    for test, item, status, detail in table:
        if test != last:
            print(f'\n[{test}]')
            last = test
        print(f'  {item.ljust(w)}  {status:8s} {detail}')
    if not a.no_graph:
        nodes = running_rig_nodes()
        print('\n[running now]')
        if nodes is None:
            print('  could not read the ROS graph (workspace sourced?)')
        elif nodes:
            print(f'  rig nodes already running: {", ".join(nodes)}  <- stop them (Ctrl+C in their terminal) '
                  'before starting a test')
        else:
            print('  no rig nodes running')
    blocking = {}
    for test, _, status, _ in table:
        blocking[test] = blocking.get(test, 0) + (status in (SET, CONNECT))
    needs = {'1 robot': ['software', '1 robot'], '2 ultrasound': ['software', '1 robot', '2 ultrasound'],
             '3 camera': ['software', '1 robot', '3 camera'],
             '4 full': ['software', '1 robot', '2 ultrasound', '3 camera', '4 full']}
    print('\nReady to start (no SET / CONNECT left in what the test needs):')
    for t, req in needs.items():
        left = sum(blocking.get(x, 0) for x in req)
        print(f'  test {t:13s} {"yes" if not left else f"not yet ({left} item(s) above)"}')

if __name__ == '__main__':
    main()
