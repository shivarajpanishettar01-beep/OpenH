"""TEST 2: micrus (ultrasound) + robot, and their calibrations (docs/TESTING_STEPS.md, test 2).

  ros2 launch usrig_bringup test2_micrus_robot.launch.py session_id:=<ID> [record:=true] [control:=false]
      [session_json:=$HOME/data/sessions/<ID>/session.json] [sim:=false]

Starts micrus (session.yaml settings; HDF5 / snapshots in <root>/micrus/<ID>), robot_state_cpp,
rig_static_tf and qc_monitor (live rates for status_console). record:=true records the ultrasound and
robot topics into <root>/checks/test2_micrus_robot_<stamp>/ (also the input of tools/temporal_calib.py).
"""
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _test_common as T  # noqa: E402


def _setup(context):
    sim, sj, sid = T.truthy(context, 'sim'), T.value(context, 'session_json'), T.value(context, 'session_id')
    acts = [T.include('micrus.launch.py', {'session_id': sid, 'sim': str(sim).lower()}),
            T.include('robot.launch.py', {'sim': str(sim).lower(), 'control': T.value(context, 'control'),
                                          'session_json': sj,
                                          'robot_ip': T.value(context, 'robot_ip')}),
            T.static_tf(sj)]
    if T.truthy(context, 'qc'):
        acts.append(T.qc_monitor(sid, sim))
    if T.truthy(context, 'record'):
        acts += T.recording('test2_micrus_robot', ['robot', 'micrus'], sim)
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_id', description='session ID (micrus output folder <root>/micrus/<ID>)'),
        DeclareLaunchArgument('session_json', default_value=''),
        DeclareLaunchArgument('sim', default_value='false', description='micrus --simulate + URSim'),
        DeclareLaunchArgument('control', default_value='false'),
        DeclareLaunchArgument('qc', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        DeclareLaunchArgument('robot_ip', default_value='', description='overrides rig.yaml robot.ip'),
        OpaqueFunction(function=_setup)])
