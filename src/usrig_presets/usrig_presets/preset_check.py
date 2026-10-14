"""preset_check: commanded moves through robot_control's move_to_pose, with the pose change reported.

  ros2 run usrig_presets preset_check offset --dx 10 [--dy 0 --dz 0 --rx 0 --ry 0 --rz 0] [--frame probe|base]
                                            [--speed 10] [--back] [--yes] [--print-only]
  ros2 run usrig_presets preset_check save NAME          # store the current TCP pose as a test preset
  ros2 run usrig_presets preset_check go NAME [--speed 10] [--yes]
  ros2 run usrig_presets preset_check list
  ros2 run usrig_presets preset_check waypoint NAME --session ~/data/sessions/<SESSION_ID> [--yes]

offset: relative move (mm, deg; probe frame by default: x along the array, y across, z into tissue).
save / go: free-space test presets in the robot base frame (<root>/presets/base_presets.yaml; --sim: under
session.sim_root) with the rig
configuration they were saved with; `go` refuses after a TCP / probe / payload change. These are for
testing and transit, not collection waypoints.
waypoint: the approach pose of an executable collection waypoint (verified with the current rig
configuration; phantom frame, via the safe height). Needs the phantom registration in that session and
robot_control started with the same session_json.
--print-only (offset): print the equivalent `ros2 service call` and do not move.

Every move goes through /robot/move_to_pose, so robot_control's pre-check (box, phantom box, tilt cone),
speed limit, timeouts and FAULT logic apply. Reports PASS if the move succeeded and the TCP ended within
1 mm / 1 deg of the target.
"""
import argparse
from pathlib import Path

import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from usrig_common import geometry
from usrig_common import motion_check as mc
from usrig_common.session import read_json
from usrig_interfaces.msg import RobotState
from usrig_interfaces.srv import MoveToPose

from .waypoints import executable_problem, rig_fingerprint, _same

MAX_OFFSET_MM = 50.0
MAX_OFFSET_DEG = 20.0
REACH_MM, REACH_DEG = 1.0, 1.0


def offset_target(T0, d_mm, r_deg, frame):
    """Target pose for a relative move. probe: offset in the probe frame at T0 (T0 @ dT); base: translation
    and rotation in base axes, rotation about the TCP."""
    dT = np.eye(4)
    dT[:3, :3] = Rotation.from_rotvec(np.radians(r_deg)).as_matrix()
    dT[:3, 3] = np.asarray(d_mm, float) / 1000.0
    if frame == 'probe':
        return T0 @ dT
    T = T0.copy()
    T[:3, 3] = T0[:3, 3] + dT[:3, 3]
    T[:3, :3] = dT[:3, :3] @ T0[:3, :3]
    return T


def service_call_text(T, speed):
    p, q = geometry.pq_from_pose(T)
    return ('ros2 service call /robot/move_to_pose usrig_interfaces/srv/MoveToPose "{target: {header: {frame_id: base}, '
            f'pose: {{position: {{x: {p[0]:.5f}, y: {p[1]:.5f}, z: {p[2]:.5f}}}, orientation: {{x: {q[0]:.6f}, '
            f'y: {q[1]:.6f}, z: {q[2]:.6f}, w: {q[3]:.6f}}}}}}}, speed_mm_s: {speed:g}}}"')


def presets_file(node, sim=False):
    from usrig_common.session import data_dir
    return data_dir(node.scfg, 'presets', sim) / 'base_presets.yaml'


def load_presets(f):
    return yaml.safe_load(f.read_text()) if f.exists() else {'presets': {}}


def preset_problem(entry, rig):
    cfg = entry.get('config') or {}
    diff = [k for k, v in rig_fingerprint(rig).items() if not _same(cfg.get(k), v)]
    return f'saved with a different {", ".join(diff)}: save it again' if diff else None


def _move(node, T_target, speed, frame='base', T_base_frame=None):
    """move_to_pose; returns (response, T_before, T_after, T_target_in_base)."""
    cli = node.create_client(MoveToPose, '/robot/move_to_pose')
    T0 = node.averaged_tcp_pose()
    req = MoveToPose.Request()
    req.target = PoseStamped()
    req.target.header.frame_id = frame
    geometry.fill_pose_msg(req.target.pose, T_target)
    req.speed_mm_s = float(speed)
    r = node.call(cli, req, timeout=300.0)
    T1 = node.averaged_tcp_pose()
    T_goal = T_base_frame @ T_target if frame == 'phantom' else T_target
    return r, T0, T1, T_goal


def report(label, r, T0, T1, T_goal):
    d = mc.pose_delta(T0, T1)
    e = mc.pose_delta(T_goal, T1)
    print(f'\n--- {label}\nbefore {mc.fmt_pose(T0)}\ntarget {mc.fmt_pose(T_goal)}\nafter  {mc.fmt_pose(T1)}')
    print(f'change:\n{mc.fmt_delta(d)}')
    rows = [('move_to_pose', bool(r.success), r.message),
            ('position reached', e['dist_mm'] <= REACH_MM, f'{e["dist_mm"]:.2f} mm from the target (limit {REACH_MM})'),
            ('orientation reached', e['angle_deg'] <= REACH_DEG, f'{e["angle_deg"]:.2f} deg from the target (limit {REACH_DEG})')]
    return mc.print_table(f'preset_check {label}', rows)


def ready(node):
    s = node.state()
    if s.mode == RobotState.MODE_FAULT:
        raise RuntimeError('robot in FAULT: acknowledge first')
    if not node.create_client(MoveToPose, '/robot/move_to_pose').wait_for_service(timeout_sec=3.0):
        raise RuntimeError('/robot/move_to_pose not available: start robot.launch.py control:=true')


def cmd_offset(node, a, ask):
    d, r = [a.dx, a.dy, a.dz], [a.rx, a.ry, a.rz]
    if not any(d) and not any(r):
        raise SystemExit('give at least one of --dx --dy --dz --rx --ry --rz')
    if np.linalg.norm(d) > MAX_OFFSET_MM or np.linalg.norm(r) > MAX_OFFSET_DEG:
        raise SystemExit(f'offset larger than this check allows ({MAX_OFFSET_MM} mm / {MAX_OFFSET_DEG} deg)')
    T0 = node.averaged_tcp_pose()
    T = offset_target(T0, d, r, a.frame)
    print(f'TCP now {mc.fmt_pose(T0)}\ntarget  {mc.fmt_pose(T)}')
    if a.print_only:
        print('\nEquivalent call (robot_control must be running):\n' + service_call_text(T, a.speed))
        return True
    ready(node)
    if not mc.confirm(f'preset_check will move by d{d} mm, r{r} deg ({a.frame} frame) at {a.speed:g} mm/s'
                      f'{" and back" if a.back else ""}.', ask, a.yes):
        print('not confirmed: nothing moved')
        return False
    ok = report('offset out', *_move(node, T, a.speed))
    if a.back:
        res = _move(node, T0, a.speed)
        ok = report('offset back', *res) and ok
    return ok


def cmd_save(node, a, ask):
    f = presets_file(node, a.sim)
    data = load_presets(f)
    T = node.averaged_tcp_pose(0.5)
    p, q = geometry.pq_from_pose(T)
    data['presets'][a.name] = {'frame': 'base', 'position_m': p.tolist(), 'quaternion_xyzw': q.tolist(),
                               'config': rig_fingerprint(node.rig)}
    f.parent.mkdir(parents=True, exist_ok=True)
    f.write_text(yaml.safe_dump(data, sort_keys=False))
    print(f'saved {a.name}: {mc.fmt_pose(T)} -> {f}')
    return True


def cmd_list(node, a, ask):
    f = presets_file(node, a.sim)
    data = load_presets(f)
    try:
        T = node.averaged_tcp_pose()
    except RuntimeError:
        T = None
    print(f'{f}:')
    for name, e in data['presets'].items():
        P = geometry.pose_from_pq(e['position_m'], e['quaternion_xyzw'])
        far = f'   {mc.pose_delta(T, P)["dist_mm"]:.1f} mm from here' if T is not None else ''
        bad = preset_problem(e, node.rig)
        print(f'  {name:16s} {mc.fmt_pose(P)}{far}{"   NOT USABLE: " + bad if bad else ""}')
    return True


def cmd_go(node, a, ask):
    data = load_presets(presets_file(node, a.sim))
    if a.name not in data['presets']:
        raise SystemExit(f'no preset {a.name} (preset_check list)')
    e = data['presets'][a.name]
    bad = preset_problem(e, node.rig)
    if bad:
        raise SystemExit(f'preset {a.name} {bad}')
    ready(node)
    T = geometry.pose_from_pq(e['position_m'], e['quaternion_xyzw'])
    if not mc.confirm(f'preset_check will move to preset {a.name} at {a.speed:g} mm/s (straight line, base frame).',
                      ask, a.yes):
        print('not confirmed: nothing moved')
        return False
    return report(f'go {a.name}', *_move(node, T, a.speed))


def cmd_waypoint(node, a, ask):
    sess = Path(a.session).expanduser()
    reg = read_json(sess / 'session.json').get('phantom_registration', {})
    if 'T_base_phantom' not in reg:
        raise SystemExit(f'no phantom registration in {sess}/session.json (register_phantom)')
    from usrig_common.session import data_dir, session_is_sim
    pid = node.scfg['session'].get('phantom_id')
    f = data_dir(node.scfg, 'waypoints', session_is_sim(sess)) / f'{pid}.yaml'
    wps = {w['name']: w for w in (yaml.safe_load(f.read_text()) if f.exists() else {'waypoints': []})['waypoints']}
    if a.name not in wps:
        raise SystemExit(f'waypoint {a.name} not in {f}')
    problem = executable_problem(wps[a.name], node.rig)
    if problem:
        raise SystemExit(problem)
    ready(node)
    A = geometry.pose_from_pq(wps[a.name]['approach_position_m'], wps[a.name]['approach_quaternion_xyzw'])
    if not mc.confirm(f'preset_check will move to the approach pose of waypoint {a.name} (phantom frame, via '
                      f'motion.transit_safe_height_mm) at {a.speed:g} mm/s. robot_control must have been started '
                      f'with session_json:={sess}/session.json.', ask, a.yes):
        print('not confirmed: nothing moved')
        return False
    return report(f'waypoint {a.name} (approach)', *_move(node, A, a.speed, 'phantom', np.array(reg['T_base_phantom'])))


def main(argv=None, ask=input, node=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    o = sub.add_parser('offset')
    for k in ('dx', 'dy', 'dz', 'rx', 'ry', 'rz'):
        o.add_argument(f'--{k}', type=float, default=0.0)
    o.add_argument('--frame', choices=['probe', 'base'], default='probe')
    o.add_argument('--back', action='store_true')
    o.add_argument('--print-only', action='store_true')
    s = sub.add_parser('save')
    s.add_argument('name')
    g = sub.add_parser('go')
    g.add_argument('name')
    sub.add_parser('list')
    w = sub.add_parser('waypoint')
    w.add_argument('name')
    w.add_argument('--session', required=True)
    for p in (o, g, w):
        p.add_argument('--speed', type=float, default=10.0, help='mm/s (robot_control caps it)')
        p.add_argument('--yes', action='store_true')
    for p in (o, s, g, sub.choices['list'], w):
        p.add_argument('--sim', action='store_true', help='simulation presets (session.sim_root)')
    a, _ = ap.parse_known_args(argv)
    fn = {'offset': cmd_offset, 'save': cmd_save, 'list': cmd_list, 'go': cmd_go, 'waypoint': cmd_waypoint}[a.cmd]
    if node is not None:
        return fn(node, a, ask)
    from usrig_common.tool_node import run_tool
    out = {}
    run_tool(lambda n: out.update(ok=fn(n, a, ask)), 'preset_check')
    raise SystemExit(0 if out.get('ok') else 1)


if __name__ == '__main__':
    main()
