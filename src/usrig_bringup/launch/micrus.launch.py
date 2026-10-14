"""Start micrus with every setting from session.yaml.

Shutdown: on Ctrl+C, ros2 launch sends SIGINT to micrus (micrus owns SIGINT and freezes and
powers down the scanner), then waits micrus.shutdown_timeout_s before escalating to SIGTERM.

  ros2 launch usrig_bringup micrus.launch.py session_id:=test_01 [sim:=true]
"""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration


def config_dir():
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def micrus_cmd(m, session_id, sim, scfg):
    # <root>/micrus/<session_id> (session.root, or session.sim_root with sim:=true)
    from usrig_common.session import data_dir
    out = data_dir(scfg, 'micrus', sim) / session_id
    cmd = [os.path.expanduser(m['command']), m['mode'], '--ros2',
           '--lines', str(m['lines']), '--depth', str(m['depth_mm']),
           '--pixel-mm', str(m['pixel_mm']), '--out', str(out),
           '--ros2-name', m['ros2_name'], '--ros2-frame', m['ros2_frame']]
    opt = {'gain': '--gain', 'power_db': '--power', 'freq_mhz': '--freq', 'focus_mm': '--focus',
           'dr': '--dr', 'view_area': '--view-area'}
    for key, flag in opt.items():
        if m.get(key) is not None:
            cmd += [flag, str(m[key])]
    if m.get('tgc') is not None:
        cmd += ['--tgc', ','.join(str(v) for v in m['tgc'])]
    cmd += [str(a) for a in m.get('extra_args') or []]
    if sim:
        cmd += [str(a) for a in m['simulate_args']]
    return cmd


def _setup(context):
    with open(config_dir() / 'session.yaml') as f:
        session = yaml.safe_load(f)
    m = session['micrus']
    sid = LaunchConfiguration('session_id').perform(context)
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    cmd = micrus_cmd(m, sid, sim, session)
    t = str(m.get('shutdown_timeout_s', 15))
    return [
        LogInfo(msg='micrus: ' + ' '.join(cmd)),
        ExecuteProcess(cmd=cmd, name='micrus', output='screen', emulate_tty=True,
                       sigterm_timeout=t, sigkill_timeout='5'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_id', description='session ID (micrus --out subfolder)'),
        DeclareLaunchArgument('sim', default_value='false', description='micrus --simulate'),
        OpaqueFunction(function=_setup),
    ])
