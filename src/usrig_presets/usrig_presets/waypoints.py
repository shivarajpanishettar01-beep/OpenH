"""Waypoint teach and verify.

  ros2 run usrig_presets waypoint_teach  <session_dir>
  ros2 run usrig_presets waypoint_verify <session_dir> [--only NAME ...]

Teach: drive the probe to the view (keyboard teleop), press Enter. Saves the probe-tip pose in
the phantom frame, the force along the probe axis, the micrus preset, a reference snapshot
(/micrus/snapshot), organ, view name and 3-5 instructions to waypoints/<phantom_id>.yaml.
The approach pose is approach_offset_mm (5 cm) back along the probe axis.

Verify (needs robot_control): retract, return to the approach pose, zero F/T, descend to contact,
take a snapshot and compare it with the reference by normalized cross-correlation. A waypoint is
kept (verified: true) only if NCC >= rig.yaml waypoint_verify_min_ncc.

Each waypoint records the rig configuration it was taught and verified with (controller TCP, probe
type, payload). episode_manager runs a waypoint only if it is verified AND verified with the current
configuration: after a probe remount or a TCP / payload change it must be verified again.
Poses are stored in the phantom frame, so a new phantom registration does not invalidate them.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from usrig_common import config, geometry
from usrig_common.session import read_json
from usrig_common.timeutil import now_ns
from usrig_interfaces.srv import DescendToContact, MoveToPose

from usrig_common.tool_node import run_tool


CONFIG_KEYS = (('tcp_setting', 'robot.tcp_setting'), ('probe_type', 'probe.type'),
               ('payload_kg', 'robot.payload_kg'), ('payload_cog_m', 'robot.payload_cog_m'))


def rig_fingerprint(rig):
    """The parts of rig.yaml a stored probe-tip pose depends on."""
    return {k: config.get_path(rig, path) for k, path in CONFIG_KEYS}


def _same(a, b, tol=1e-6):
    if isinstance(a, (list, tuple)) or isinstance(b, (list, tuple)):
        try:
            return len(a) == len(b) and all(_same(x, y, tol) for x, y in zip(a, b))
        except TypeError:
            return False
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(float(a) - float(b)) <= tol
    return a == b


def executable_problem(wp, rig):
    """None if the waypoint may be executed with this rig configuration, else the reason."""
    if not wp.get('verified'):
        return f'waypoint {wp.get("name")} is not verified (run waypoint_verify)'
    vc = wp.get('verified_config')
    if not isinstance(vc, dict):
        return f'waypoint {wp.get("name")} has no verified_config (verify it again)'
    cur = rig_fingerprint(rig)
    diff = [k for k, _ in CONFIG_KEYS if not _same(vc.get(k), cur[k])]
    if diff:
        return (f'waypoint {wp.get("name")} was verified with a different {", ".join(diff)}: '
                'verify it again with the current probe / TCP / payload')
    return None


def snapshot_png(msg):
    p = Path(msg)
    return p if p.suffix == '.png' else p.with_suffix('.png')


def ncc(a, b):
    a = a.astype(np.float64)
    b = b.astype(np.float64)
    if a.shape != b.shape:
        import cv2
        b = cv2.resize(b, (a.shape[1], a.shape[0]))
    a = (a - a.mean()) / (a.std() + 1e-9)
    b = (b - b.mean()) / (b.std() + 1e-9)
    return float((a * b).mean())


def _paths(node, session):
    sess = read_json(Path(session) / 'session.json')
    reg = sess.get('phantom_registration')
    if not reg:
        raise RuntimeError('register the phantom first (register_phantom)')
    pid = node.scfg['session']['phantom_id']
    if not pid:
        raise RuntimeError('session.phantom_id is null in session.yaml')
    f = Path(node.scfg['session']['waypoint_dir']).expanduser() / f'{pid}.yaml'
    return np.array(reg['T_base_phantom']), f, pid


def _load(f, pid):
    if f.exists():
        return yaml.safe_load(f.read_text())
    return {'phantom_id': pid, 'waypoints': []}


def _save(f, data):
    f.parent.mkdir(parents=True, exist_ok=True)
    tmp = f.with_suffix('.tmp')
    tmp.write_text(yaml.safe_dump(data, sort_keys=False))
    tmp.replace(f)


def _teach(node, a):
    T_bp, f, pid = _paths(node, a.session)
    back = float(node.rig['motion']['approach_offset_mm']) / 1000.0
    snap = node.create_client(Trigger, '/micrus/snapshot')
    latest = {}
    node.create_subscription(String, '/micrus/frame_info', lambda m: latest.update(fi=m.data), 5)
    data = _load(f, pid)
    while True:
        name = input('\nWaypoint name (empty to finish): ').strip()
        if not name:
            break
        input('Drive the probe to the view, then press Enter to capture... ')
        T_tip = node.tip_pose_base()
        s = node.state()
        T_ph = geometry.inv(T_bp) @ T_tip
        A = T_ph.copy()
        A[:3, 3] = T_ph[:3, 3] - T_ph[:3, 2] * back
        f_axis = float(T_tip[:3, 2] @ np.asarray(s.wrench[:3]))
        r = node.call(snap, Trigger.Request())
        fi = json.loads(latest['fi']) if 'fi' in latest else {}
        organ = input('  organ: ').strip()
        view = input('  view name: ').strip()
        instr = []
        print('  3-5 paraphrased instructions (empty line to stop):')
        while len(instr) < 5:
            t = input(f'   {len(instr) + 1}: ').strip()
            if not t:
                if len(instr) >= 3:
                    break
                print('   need at least 3')
                continue
            instr.append(t)
        p, q = geometry.pq_from_pose(T_ph)
        pa, qa = geometry.pq_from_pose(A)
        wp = {'name': name, 'organ': organ, 'view': view,
              'position_m': p.tolist(), 'quaternion_xyzw': q.tolist(),
              'approach_position_m': pa.tolist(), 'approach_quaternion_xyzw': qa.tolist(),
              'force_n': f_axis, 'preset': fi.get('settings'),
              'reference_snapshot': str(snapshot_png(r.message)) if r.success else None,
              'instructions': instr, 'verified': False, 'taught_ns': now_ns(),
              'taught_config': rig_fingerprint(node.rig)}
        data['waypoints'] = [w for w in data['waypoints'] if w['name'] != name] + [wp]
        _save(f, data)
        print(f'  saved {name} to {f}')


def _verify(node, a):
    import cv2
    T_bp, f, pid = _paths(node, a.session)
    min_ncc = config.get_path(node.rig, 'waypoint_verify_min_ncc')
    if min_ncc is None:
        raise config.UnsetLimitError(['waypoint_verify_min_ncc'], 'rig.yaml')
    snap = node.create_client(Trigger, '/micrus/snapshot')
    move = node.create_client(MoveToPose, '/robot/move_to_pose')
    descend = node.create_client(DescendToContact, '/robot/descend_to_contact')
    retract = node.create_client(Trigger, '/robot/retract')
    zero = node.create_client(Trigger, '/robot/zero_ft')
    data = _load(f, pid)
    for wp in data['waypoints']:
        if a.only and wp['name'] not in a.only:
            continue
        print(f'\n{wp["name"]}: retract -> approach -> contact')
        node.call(retract, Trigger.Request(), 60)
        req = MoveToPose.Request()
        req.target = PoseStamped()
        req.target.header.frame_id = 'phantom'
        geometry.fill_pose_msg(req.target.pose, geometry.pose_from_pq(wp['approach_position_m'],
                                                                      wp['approach_quaternion_xyzw']))
        r = node.call(move, req, 120)
        if not r.success:
            raise RuntimeError(f'move_to_pose: {r.message}')
        node.call(zero, Trigger.Request())
        d = DescendToContact.Request()
        d.force_setpoint_n = float(wp.get('force_n') or 0.0)
        r = node.call(descend, d, 120)
        if not r.success:
            raise RuntimeError(f'descend: {r.message}')
        time.sleep(1.0)
        r = node.call(snap, Trigger.Request())
        ref = cv2.imread(wp['reference_snapshot'], cv2.IMREAD_GRAYSCALE) if wp.get('reference_snapshot') else None
        cur = cv2.imread(str(snapshot_png(r.message)), cv2.IMREAD_GRAYSCALE)
        if ref is None or cur is None:
            print('  could not read snapshot images')
            wp['verified'] = False
            wp.pop('verified_config', None)
            continue
        score = ncc(ref, cur)
        T_ph = geometry.inv(T_bp) @ node.tip_pose_base()
        dp = np.linalg.norm(T_ph[:3, 3] - np.array(wp['position_m'])) * 1000
        wp['verified'] = score >= min_ncc
        if wp['verified']:
            wp['verified_config'] = rig_fingerprint(node.rig)
        else:
            wp.pop('verified_config', None)
        wp['verify'] = {'ncc': score, 'position_error_mm': float(dp), 'snapshot': str(snapshot_png(r.message)),
                        'stamp_ns': now_ns()}
        print(f'  NCC {score:.3f} (min {min_ncc}), tip position error {dp:.1f} mm -> '
              f'{"KEEP" if wp["verified"] else "REJECT"}')
        _save(f, data)
    node.call(retract, Trigger.Request(), 60)


def teach_main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    a, _ = ap.parse_known_args()
    run_tool(lambda n: _teach(n, a), 'waypoint_teach')


def verify_main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session')
    ap.add_argument('--only', nargs='*')
    a, _ = ap.parse_known_args()
    run_tool(lambda n: _verify(n, a), 'waypoint_verify')
