"""Shared pieces of the staged test launch files (test1-test4). Not a launch file itself."""
import os
from pathlib import Path

from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent


def config_dir():
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def session_cfg():
    from usrig_common import config               # shared session.yaml + this PC's machine.yaml
    return config.load_config('session', str(config_dir()))


def include(name, args):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(HERE / name)),
                                    launch_arguments={k: str(v) for k, v in args.items()}.items())


def params(session_id='', session_json='', sim=False):
    return [{'config_dir': str(config_dir()), 'session_id': str(session_id), 'session_json': str(session_json),
             'sim': bool(sim), 'use_sim_time': False}]


def static_tf(session_json):
    return Node(package='usrig_episode', executable='rig_static_tf', output='screen',
                parameters=[{'config_dir': str(config_dir()), 'session_json': str(session_json), 'use_sim_time': False}])


def qc_monitor(session_id, sim):
    return Node(package='usrig_qc', executable='qc_monitor', output='screen', parameters=params(session_id, '', sim))


def recording(test_name, parts, sim):
    """ros2 bag record of the streams under test into <root>/checks/<test>_<stamp> (bag_check checks it)."""
    from usrig_common.check_recording import check_dir, record_cmd, topics_for
    scfg = session_cfg()
    out = check_dir(scfg, test_name, sim)
    cmd = record_cmd(scfg, topics_for(scfg, parts), out)
    return [LogInfo(msg=f'TEST RECORDING -> {out}   (Ctrl+C stops it; then: ros2 run usrig_qc bag_check {out} '
                        f'--expect {" ".join(parts)})'),
            ExecuteProcess(cmd=cmd, name='test_recording', output='screen', sigterm_timeout='10')]


def truthy(context, name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def value(context, name):
    from launch.substitutions import LaunchConfiguration
    return LaunchConfiguration(name).perform(context)
