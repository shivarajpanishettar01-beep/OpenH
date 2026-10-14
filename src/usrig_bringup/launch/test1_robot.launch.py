"""TEST 1: robot connection, topics and poses (docs/TESTING_STEPS.md, test 1).

  ros2 launch usrig_bringup test1_robot.launch.py [record:=true] [control:=false] [sim:=false]

Starts robot_state_cpp (read-only RTDE, 500 Hz: /robot/state, /joint_states, /robot/tcp_pose,
/robot/wrench) and rig_static_tf (/tf_static). control:=true adds robot_control (needs every rig.yaml
limit; only for the motion checks). record:=true records the robot topics into
<root>/checks/test1_robot_<stamp>/ until Ctrl+C; check it with bag_check.
"""
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _test_common as T  # noqa: E402


def _setup(context):
    sim, sj = T.truthy(context, 'sim'), T.value(context, 'session_json')
    acts = [T.include('robot.launch.py', {'sim': str(sim).lower(), 'control': T.value(context, 'control'),
                                          'session_json': sj}),
            T.static_tf(sj)]
    if T.truthy(context, 'record'):
        acts += T.recording('test1_robot', ['robot'], sim)
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false', description='URSim (rig.yaml robot.ursim_ip)'),
        DeclareLaunchArgument('control', default_value='false', description='also start robot_control'),
        DeclareLaunchArgument('record', default_value='false', description='record the robot topics'),
        DeclareLaunchArgument('session_json', default_value='', description='session.json with a phantom registration'),
        OpaqueFunction(function=_setup)])
