"""TEST 4: the whole rig, as on a collection day (docs/TESTING_STEPS.md, test 4).

  ros2 launch usrig_bringup test4_full_system.launch.py session_id:=<ID>
      session_json:=$HOME/data/sessions/<ID>/session.json
      [input:=keyboard|spacemouse|freedrive] [control:=false] [camera:=true] [tpv:=true] [episode_manager:=true] [sim:=false]

collect.launch.py with the test defaults: micrus, wrist camera, TPV camera (where enabled), robot_state, qc_monitor, rig_static_tf,
the operator input, and episode_manager (Stage 0 episodes into <root>/sessions/<ID>/ep_NNNN). Episodes
are the recording here (no extra test bag). A real session needs a passing presession_check first.
input:=freedrive starts episode_manager in freedrive collection (control must be false).
"""
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _test_common as T  # noqa: E402

ARGS = (('session_id', None), ('session_json', ''), ('sim', 'false'), ('control', 'false'), ('camera', 'true'),
        ('tpv', 'true'), ('input', 'keyboard'), ('episode_manager', 'true'), ('qc', 'true'), ('dashboard', 'false'), ('robot_ip', ''))


def _setup(context):
    args = {k: T.value(context, k) for k, _ in ARGS}
    if args['input'] == 'freedrive' and args['control'].lower() == 'true':
        raise RuntimeError('input:=freedrive needs control:=false (the arm is guided by hand)')
    return [T.include('collect.launch.py', {**args, 'teleop': 'true'})]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d) if d is not None else DeclareLaunchArgument(k) for k, d in ARGS]
        + [OpaqueFunction(function=_setup)])
