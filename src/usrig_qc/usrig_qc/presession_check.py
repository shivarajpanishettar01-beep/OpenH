"""Pre-session check: one PASS/FAIL table before a collection session.

  ros2 run usrig_qc presession_check --stage 0 --session <session_dir> \
      [--planned-hours 2] [--mb-per-s 45] [--control]

--mb-per-s: measured rate of one test episode (bag + HDF5); without it the most recent episode
under the session root is used, or the disk check reports "unset".

The result (every row, PASS / FAIL, with or without --control) is written to <session>/session.json
under presession.stage<N>. episode_manager refuses to start a real (non-sim) session without a
PASS for that stage and control mode younger than session.presession_max_age_h.
"""
import argparse
import json
import shutil
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

from usrig_common import config, gates
from usrig_common.session import read_json, update_session_json
from usrig_common.timeutil import now_ns, ns_from_stamp

from .usb import realsense_usb


class Table:

    def __init__(self):
        self.rows = []

    def add(self, name, status, detail=''):
        self.rows.append((name, status, detail))

    def print(self):
        w = max(len(r[0]) for r in self.rows)
        print(f'\n{"CHECK".ljust(w)}  RESULT  DETAIL')
        print('-' * (w + 60))
        for n, s, d in self.rows:
            print(f'{n.ljust(w)}  {s:6s}  {d}')
        ok = all(s in ('PASS', 'N/A') for _, s, _ in self.rows)
        print('-' * (w + 60))
        print(f'PRE-SESSION: {"PASS" if ok else "FAIL"}\n')
        return ok


def expected_rates(scfg, rig, control):
    lo, hi = scfg['micrus']['expected_fps']
    cam = scfg['wrist_camera']
    r = {'/micrus/bmode': (lo, hi), '/micrus/frame_info': (lo, hi)}
    hz = float(rig['robot']['state_rate_hz'])
    for t in ('/robot/state', '/joint_states', '/robot/tcp_pose', '/robot/wrench'):
        r[t] = (0.98 * hz, 1.02 * hz)
    if control:
        r['/robot/cmd_executed'] = (0.95 * rig['robot']['control_rate_hz'], 1.05 * rig['robot']['control_rate_hz'])
    r['/qc/status'] = (1.5, 2.5)
    if cam.get('enabled') and scfg['recording'].get('include_wrist_camera'):
        f = float(cam['fps'])
        for k in ('color', 'depth', 'color_info', 'depth_info'):
            if cam['topics'].get(k):
                r[cam['topics'][k]] = (0.95 * f, 1.05 * f)
    return r


class Probe(Node):

    def __init__(self, topics):
        super().__init__('presession_check')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        types = dict(self.get_topic_names_and_types())
        self.samples = {t: [] for t in topics}
        self.missing = []
        for t in topics:
            if t not in types:
                self.missing.append(t)
                continue
            cls = get_message(types[t][0])

            def cb(m, t=t):
                st = ns_from_stamp(m.header.stamp) if hasattr(m, 'header') else None
                self.samples[t].append((now_ns(), st))
            self.create_subscription(cls, t, cb, qos)

    def get_param(self, node_fqn, names, timeout=3.0):
        c = self.create_client(GetParameters, f'{node_fqn}/get_parameters')
        if not c.wait_for_service(timeout_sec=timeout):
            return None
        fut = c.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            return None
        from rclpy.parameter import parameter_value_to_python
        return [parameter_value_to_python(v) for v in fut.result().values]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stage', type=int, required=True)
    ap.add_argument('--session', required=True)
    ap.add_argument('--planned-hours', type=float, default=None)
    ap.add_argument('--mb-per-s', type=float, default=None)
    ap.add_argument('--control', action='store_true', help='robot_control will run')
    ap.add_argument('--sim', action='store_true', help='simulated run (session.sim_root)')
    ap.add_argument('--listen-s', type=float, default=5.0)
    ap.add_argument('--config-dir')
    a, _ = ap.parse_known_args()
    scfg = config.load_config('session', a.config_dir)
    rig = config.load_config('rig', a.config_dir)
    tab = Table()

    # micrus licence / USB (no public way to check the licence: run `micrus machine-id`)
    mc = Path(scfg['micrus']['command']).expanduser()
    found = shutil.which(str(mc)) or (mc.exists() and str(mc))
    tab.add('micrus command', 'PASS' if found else 'FAIL', str(found or mc))
    lic = config.get_path(scfg, 'micrus.licence_file')
    if lic is None:
        tab.add('micrus licence file', 'UNSET', 'set micrus.licence_file in session.yaml')
    else:
        tab.add('micrus licence file', 'PASS' if Path(lic).expanduser().exists() else 'FAIL', lic)
    vid = config.get_path(scfg, 'micrus.usb_vendor_id')
    if vid is None:
        tab.add('micrus USB device', 'UNSET', 'set micrus.usb_vendor_id (lsusb) in session.yaml')
    else:
        out = subprocess.run(['lsusb'], capture_output=True, text=True).stdout
        tab.add('micrus USB device', 'PASS' if f'ID {vid}:' in out else 'FAIL', vid)

    # D415
    cam = scfg['wrist_camera']
    if cam.get('enabled'):
        devs = realsense_usb(cam.get('serial'))
        if not devs:
            tab.add('D415 on USB', 'FAIL', 'no RealSense on the bus')
        else:
            sp = devs[0]['speed_mbps']
            tab.add('D415 on USB 3.x', 'PASS' if sp >= 5000 else 'FAIL', f'{devs[0]["product"]} {sp} Mb/s')

    rclpy.init()
    rates = expected_rates(scfg, rig, a.control)
    p = Probe(list(rates))
    t_end = time.monotonic() + a.listen_s
    while time.monotonic() < t_end:
        rclpy.spin_once(p, timeout_sec=0.05)

    worst_offset = 0.0
    for t, (lo, hi) in rates.items():
        s = p.samples[t]
        if t in p.missing or len(s) < 2:
            tab.add(f'rate {t}', 'FAIL', 'not published' if t in p.missing else f'{len(s)} msgs')
            continue
        recv = np.array([x[0] for x in s])
        hz = (len(s) - 1) / ((recv[-1] - recv[0]) * 1e-9)
        tab.add(f'rate {t}', 'PASS' if lo <= hz <= hi else 'FAIL', f'{hz:.1f} Hz (expect {lo:.1f}-{hi:.1f})')
        st = [x for x in s if x[1] is not None]
        if st:
            off = np.median([(x[1] - x[0]) * 1e-3 for x in st])     # us -> report ms below
            worst_offset = max(worst_offset, abs(off) / 1000.0)
    tab.add('clock: header stamp vs now', 'PASS' if worst_offset < 100 else 'FAIL',
            f'worst median offset {worst_offset:.2f} ms (must be ms, not s)')

    # use_sim_time everywhere
    sim = []
    for name, ns in p.get_node_names_and_namespaces():
        fqn = (ns.rstrip('/') + '/' + name) if ns != '/' else '/' + name
        if name == 'presession_check':
            continue
        v = p.get_param(fqn, ['use_sim_time'], timeout=1.0)
        if v and v[0]:
            sim.append(fqn)
    tab.add('use_sim_time false everywhere', 'FAIL' if sim else 'PASS', ', '.join(sim))

    # camera exposure / white balance locked
    if cam.get('enabled'):
        node = f'/{cam["camera_namespace"]}/{cam["camera_name"]}'
        v = p.get_param(node, ['rgb_camera.enable_auto_exposure', 'rgb_camera.enable_auto_white_balance'])
        if v is None:
            tab.add('D415 exposure / WB locked', 'FAIL', f'cannot read params of {node} (names NOT VERIFIED)')
        else:
            tab.add('D415 exposure / WB locked', 'PASS' if v == [False, False] else 'FAIL',
                    f'auto_exposure={v[0]} auto_wb={v[1]}')

    # gates, limits, plan
    from usrig_common.session import data_root
    root_data = data_root(scfg, a.sim)
    tab.add('data root', 'PASS', f'{root_data} ({"SIM: never exported" if a.sim else "real collection"})')
    g = gates.check_stage(a.stage, scfg, root_data)
    tab.add(f'gate for stage {a.stage}', 'PASS' if g['passed'] else 'FAIL', json.dumps(g)[:120])
    if a.control or a.stage > 0:
        miss = config.find_unset(rig, config.MOTION_LIMITS)
        tab.add('rig.yaml safety limits set', 'FAIL' if miss else 'PASS', ', '.join(miss)[:200])
    else:
        tab.add('rig.yaml safety limits set', 'N/A', 'robot not commanded')
    if a.stage == 2:
        lock = read_json(Path(a.session) / 'session.json').get('plan_lock', {})
        pf = scfg['stage2'].get('plan_file')
        from hashlib import sha256
        ok = bool(pf) and Path(pf).exists() and lock.get('sha256') == sha256(Path(pf).read_bytes()).hexdigest()
        tab.add('plan locked', 'PASS' if ok else 'FAIL', str(pf))
    else:
        tab.add('plan locked', 'N/A', f'stage {a.stage}')

    # disk
    root = root_data
    while not root.exists():
        root = root.parent
    free_gb = shutil.disk_usage(root).free / 1e9
    mbps = a.mb_per_s
    if mbps is None:
        eps = sorted(root_data.glob('*/ep_*/episode.json'))
        for e in reversed(eps):
            ep = json.loads(e.read_text())
            if ep.get('duration_s'):
                size = sum(f.stat().st_size for f in e.parent.rglob('*') if f.is_file())
                h5 = ep.get('micrus_hdf5_path')
                if h5 and Path(h5).exists():
                    size += Path(h5).stat().st_size
                mbps = size / 1e6 / ep['duration_s']
                break
    if mbps is None or a.planned_hours is None:
        tab.add('free disk vs planned session', 'UNSET',
                f'{free_gb:.0f} GB free; give --planned-hours and --mb-per-s (or record a test episode)')
    else:
        need = mbps * 3600 * a.planned_hours / 1000 + scfg['session'].get('min_free_disk_gb', 50)
        tab.add('free disk vs planned session', 'PASS' if free_gb >= need else 'FAIL',
                f'{free_gb:.0f} GB free, need {need:.0f} GB ({mbps:.1f} MB/s x {a.planned_hours} h + reserve)')
    p.destroy_node()
    rclpy.shutdown()
    ok = tab.print()
    Path(a.session).mkdir(parents=True, exist_ok=True)
    update_session_json(a.session, presession={f'stage{a.stage}': {
        'pass': ok, 'control': bool(a.control), 'sim': bool(a.sim), 'checked_ns': now_ns(),
        'rows': [{'check': n, 'result': s, 'detail': d} for n, s, d in tab.rows]}})
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
