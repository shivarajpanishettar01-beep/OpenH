"""TEST 3: wrist D415 + robot, and the camera calibrations (docs/TESTING_STEPS.md, test 3).

  ros2 launch usrig_bringup test3_camera_robot.launch.py [record:=true] [auto:=false] [control:=false]
      [session_json:=...] [session_id:=<ID>]

Starts the wrist camera (locked exposure; auto:=true only for camera_autolock sampling), robot_state_cpp,
rig_static_tf (/tf_static base -> flange -> wrist_camera once the hand-eye result is in rig.yaml) and
qc_monitor. record:=true records the camera and robot topics into <root>/checks/test3_camera_robot_<stamp>/.
The camera has no simulator: run this on the capture PC with the D415 on USB 3.
"""
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _test_common as T  # noqa: E402


def _setup(context):
    sj, sid = T.value(context, 'session_json'), T.value(context, 'session_id')
    acts = [T.include('wrist_camera.launch.py', {'auto': T.value(context, 'auto')}),
            T.include('robot.launch.py', {'sim': 'false', 'control': T.value(context, 'control'), 'session_json': sj}),
            T.static_tf(sj)]
    if T.truthy(context, 'qc'):
        acts.append(T.qc_monitor(sid, False))
    if T.truthy(context, 'record'):
        acts += T.recording('test3_camera_robot', ['robot', 'camera'], False)
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('auto', default_value='false', description='auto exposure (camera_autolock only)'),
        DeclareLaunchArgument('session_id', default_value=''),
        DeclareLaunchArgument('session_json', default_value=''),
        DeclareLaunchArgument('control', default_value='false'),
        DeclareLaunchArgument('qc', default_value='true'),
        DeclareLaunchArgument('record', default_value='false'),
        OpaqueFunction(function=_setup)])
