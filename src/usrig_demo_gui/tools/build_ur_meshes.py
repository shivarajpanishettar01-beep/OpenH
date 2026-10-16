"""Build usrig_demo_gui/assets/ur7e_meshes.npz from Universal_Robots_ROS2_Description (BSD-3-Clause).

Run once, only when the meshes should be regenerated:
  git clone --depth 1 https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git /tmp/urdesc
  pip install trimesh pycollada
  python tools/build_ur_meshes.py /tmp/urdesc

The UR7e uses the UR5e meshes (config/ur7e/visual_parameters.yaml). Each link mesh is stored in its
URDF link frame (visual mesh_offset applied), decimated, with one colour per vertex from the
COLLADA material and precomputed vertex normals.
"""
import sys
from pathlib import Path

import numpy as np
import trimesh
import yaml

LINKS = {'base': 'base', 'shoulder': 'shoulder', 'upper_arm': 'upperarm', 'forearm': 'forearm',
         'wrist_1': 'wrist1', 'wrist_2': 'wrist2', 'wrist_3': 'wrist3'}
KEEP = 1.0          # fraction of faces kept on large sub-meshes (1.0 = no decimation)


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def main(root):
    root = Path(root)
    vp = yaml.safe_load((root / 'config/ur7e/visual_parameters.yaml').read_text().replace('!degrees', ''))
    out = {}
    for link, fname in LINKS.items():
        off = vp['mesh_files'][link]['mesh_offset']
        T_off = np.eye(4)
        T_off[:3, :3] = rpy(*np.radians([off['roll'], off['pitch'], off['yaw']]))
        T_off[:3, 3] = [off['x'], off['y'], off['z']]
        scene = trimesh.load(root / f'meshes/ur5e/visual/{fname}.dae', force='scene')
        V, F, C, N = [], [], [], []
        n0 = 0
        for node in scene.graph.nodes_geometry:
            T, geom = scene.graph[node]
            m = scene.geometry[geom].copy()
            col = np.asarray(m.visual.material.main_color, np.uint8)
            m.apply_transform(T_off @ T)
            if KEEP < 1.0 and len(m.faces) > 3000:
                m = m.simplify_quadric_decimation(percent=1 - KEEP)
            V.append(m.vertices.astype(np.float32))
            n = np.nan_to_num(np.asarray(m.vertex_normals, np.float32))
            N.append(n)
            F.append(m.faces.astype(np.uint32) + n0)
            C.append(np.tile(col, (len(m.vertices), 1)))
            n0 += len(m.vertices)
        out[f'{link}_v'] = np.concatenate(V)
        out[f'{link}_f'] = np.concatenate(F)
        out[f'{link}_c'] = np.concatenate(C)
        out[f'{link}_n'] = np.concatenate(N)
        print(link, len(out[f'{link}_f']), 'faces')
    dst = Path(__file__).resolve().parents[1] / 'usrig_demo_gui' / 'assets' / 'ur7e_meshes.npz'
    dst.parent.mkdir(exist_ok=True)
    np.savez_compressed(dst, **out)
    print('wrote', dst, dst.stat().st_size // 1024, 'KiB')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '/tmp/urdesc')
