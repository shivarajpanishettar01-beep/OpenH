"""UR7e nodes.

robot_state: the C++ node (usrig_robot_cpp/robot_state_cpp, lossless RTDE via ur_client_library)
when it is built; otherwise the Python robot_state (polls RTDE; loses samples under load, see
docs/data_integrity.md) with a warning.
control:=true adds robot_control (refuses while limits are null); it reads RTDE for control and
publishes /robot/control_status, which robot_state_cpp merges into /robot/state.
sim:=true points everything at URSim (rig.yaml robot.ursim_ip).
"""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config_dir():
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def cpp_available():
    try:
        from ament_index_python.packages import get_package_prefix
        return (Path(get_package_prefix('usrig_robot_cpp')) / 'lib' / 'usrig_robot_cpp' / 'robot_state_cpp').exists()
    except Exception:
        return False


def _setup(context):
    cfg = config_dir()
    with open(cfg / 'rig.yaml') as f:
        rig = yaml.safe_load(f)
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    control = LaunchConfiguration('control').perform(context).lower() == 'true'
    sj = LaunchConfiguration('session_json').perform(context)
    ip = rig['robot']['ursim_ip'] if sim else rig['robot']['ip']
    T = []
    if sj and Path(sj).exists():
        import json
        reg = json.loads(Path(sj).read_text()).get('phantom_registration', {})
        if 'T_base_phantom' in reg:
            T = [float(x) for row in reg['T_base_phantom'] for x in row]
    actions = []
    use_cpp = cpp_available()
    if use_cpp:
        p = {'robot_ip': ip or '', 'frequency_hz': float(rig['robot']['state_rate_hz']),
             'rt_priority': int(rig['robot'].get('rt_priority') or 0), 'use_sim_time': False,
             'freedrive_input': int(-1 if (rig.get('freedrive') or {}).get('input') is None else rig['freedrive']['input'])}
        if T:          # launch rejects empty lists: pass the registration only when it exists
            p['T_base_phantom'] = T
        actions.append(Node(package='usrig_robot_cpp', executable='robot_state_cpp', output='screen',
                            parameters=[p]))
    else:
        actions.append(LogInfo(msg='WARNING: robot_state_cpp not built (ros-humble-ur-client-library missing); '
                                   'using the Python robot_state, which loses samples under load'))
        if not control:
            actions.append(Node(package='usrig_robot', executable='robot_state', output='screen',
                                parameters=[{'config_dir': str(cfg), 'robot_ip': ip or '',
                                             'frequency_hz': float(rig['robot']['state_rate_hz']),
                                             'session_json': sj, 'use_sim_time': False}]))
    if control:
        actions.append(Node(package='usrig_robot', executable='robot_control', output='screen',
                            additional_env={'USRIG_STATE_FROM_CPP': '1' if use_cpp else '0'},
                            parameters=[{'config_dir': str(cfg), 'robot_ip': ip or '',
                                         'frequency_hz': float(rig['robot']['state_rate_hz']),
                                         'session_json': sj, 'use_sim_time': False}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('control', default_value='false'),
        DeclareLaunchArgument('session_json', default_value=''),
        OpaqueFunction(function=_setup),
    ])
