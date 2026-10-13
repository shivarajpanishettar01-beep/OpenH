"""Stage 1 path editor: build paths from taught waypoints, set speed, preview in RViz, save.

  ros2 run usrig_episode path_editor line  NAME --through WP1 WP2 [WP3 ...] [--instruction "..."]
  ros2 run usrig_episode path_editor fan   NAME --at WP --axis tilt|rock --deg 10 --steps 5
  ros2 run usrig_episode path_editor sweep NAME --at WP --along x|y --mm 40
  ros2 run usrig_episode path_editor show  NAME        # nav_msgs/Path on /usrig/path_preview (frame phantom)
  ros2 run usrig_episode path_editor list

Paths are stored in paths/<phantom_id>.yaml. Waypoints in a path are names of taught waypoints
or inline poses generated here (fan / sweep). Speed is session.yaml stage1.speed_mm_s.
"""
import argparse
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from usrig_common import config, geometry


def _files(scfg):
    pid = scfg['session']['phantom_id']
    if not pid:
        raise SystemExit('session.phantom_id is null in session.yaml')
    return (Path(scfg['session']['waypoint_dir']).expanduser() / f'{pid}.yaml',
            Path(scfg['session']['path_dir']).expanduser() / f'{pid}.yaml')


def _wp(wps, name):
    for w in wps:
        if w['name'] == name:
            return geometry.pose_from_pq(w['position_m'], w['quaternion_xyzw'])
    raise SystemExit(f'waypoint {name} not found')


def _pose_entry(name, T):
    p, q = geometry.pq_from_pose(T)
    return {'name': name, 'position_m': p.tolist(), 'quaternion_xyzw': q.tolist()}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cmd', choices=['line', 'fan', 'sweep', 'show', 'list'])
    ap.add_argument('name', nargs='?')
    ap.add_argument('--through', nargs='+')
    ap.add_argument('--at')
    ap.add_argument('--axis', choices=['tilt', 'rock'], default='tilt')
    ap.add_argument('--deg', type=float)
    ap.add_argument('--steps', type=int, default=3)
    ap.add_argument('--along', choices=['x', 'y'], default='x')
    ap.add_argument('--mm', type=float)
    ap.add_argument('--instruction', default=None)
    ap.add_argument('--config-dir', default=None)
    a, _ = ap.parse_known_args(argv)
    scfg = config.load_config('session', a.config_dir)
    wfile, pfile = _files(scfg)
    wps = yaml.safe_load(wfile.read_text())['waypoints'] if wfile.exists() else []
    paths = yaml.safe_load(pfile.read_text()) if pfile.exists() else {'paths': {}}

    if a.cmd == 'list':
        for n, p in paths['paths'].items():
            print(n, [w if isinstance(w, str) else w['name'] for w in p['waypoints']])
        return
    if not a.name:
        raise SystemExit('path name required')
    if a.cmd == 'show':
        return _show(paths['paths'][a.name], wps)

    if a.cmd == 'line':
        if not a.through or len(a.through) < 2:
            raise SystemExit('--through needs at least two waypoints')
        for n in a.through:
            _wp(wps, n)
        entries = list(a.through)
    else:
        T0 = _wp(wps, a.at)
        entries = []
        if a.cmd == 'fan':
            if a.deg is None:
                raise SystemExit('--deg required')
            axis = 'x' if a.axis == 'tilt' else 'y'
            for k, ang in enumerate(np.linspace(-a.deg, a.deg, max(2, a.steps))):
                T = T0.copy()
                T[:3, :3] = T0[:3, :3] @ Rotation.from_euler(axis, ang, degrees=True).as_matrix()
                entries.append(_pose_entry(f'{a.at}_fan{k}', T))
        else:
            if a.mm is None:
                raise SystemExit('--mm required')
            col = 0 if a.along == 'x' else 1
            for k, d in enumerate((-a.mm / 2, a.mm / 2)):
                T = T0.copy()
                T[:3, 3] = T0[:3, 3] + T0[:3, col] * d / 1000.0
                entries.append(_pose_entry(f'{a.at}_sweep{k}', T))
    paths['paths'][a.name] = {'waypoints': entries, 'instruction': a.instruction}
    pfile.parent.mkdir(parents=True, exist_ok=True)
    pfile.write_text(yaml.safe_dump(paths, sort_keys=False))
    print(f'saved path {a.name} ({len(entries)} waypoints) to {pfile}')


def resolve(entries, wps):
    out = []
    for e in entries:
        out.append(_wp(wps, e) if isinstance(e, str) else geometry.pose_from_pq(e['position_m'], e['quaternion_xyzw']))
    return out


def _show(path, wps):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as PathMsg
    from rclpy.qos import DurabilityPolicy, QoSProfile
    rclpy.init()
    n = rclpy.create_node('path_preview')
    pub = n.create_publisher(PathMsg, '/usrig/path_preview',
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    m = PathMsg()
    m.header.frame_id = 'phantom'
    for T in resolve(path['waypoints'], wps):
        ps = PoseStamped()
        ps.header.frame_id = 'phantom'
        geometry.fill_pose_msg(ps.pose, T)
        m.poses.append(ps)
    pub.publish(m)
    print('publishing /usrig/path_preview (frame phantom); Ctrl+C to stop')
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.5)
            time.sleep(0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    n.destroy_node()
    rclpy.shutdown()
