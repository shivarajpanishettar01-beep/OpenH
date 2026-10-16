"""Everything for a collection session.

  ros2 launch usrig_bringup collect.launch.py session_id:=<id> [sim:=true] [control:=false]
      [camera:=true] [teleop:=true] [episode_manager:=true] [qc:=true] [dashboard:=false]
      [input:=keyboard|spacemouse|freedrive] [stage:=0|1|2] [operator_id:=..] [phantom_id:=..] [console:=true]
      [demo_gui:=true]

console:=true starts the operator console (no images; usable during collection). The demo dashboard stays
dashboard:=true. demo_gui:=true starts the read-only demo screen (usrig_demo_gui) for demonstrations only:
while episode_manager runs a real (non-sim) session it drops all its subscriptions and shows "paused".

sim:=true -> micrus --simulate and the robot nodes on URSim. The wrist camera has no simulator:
use camera:=false in simulation.
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def config_dir():
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def generate_launch_description():
    share = FindPackageShare('usrig_bringup')
    L = LaunchConfiguration
    inc = lambda f, args: IncludeLaunchDescription(  # noqa: E731
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', f])),
        launch_arguments=args.items())
    # value_type=str: a purely numeric session id must not become an int parameter
    params = [{'config_dir': str(config_dir()), 'session_id': ParameterValue(L('session_id'), value_type=str),
               'session_json': ParameterValue(L('session_json'), value_type=str),
               'sim': ParameterValue(L('sim'), value_type=bool), 'use_sim_time': False}]
    return LaunchDescription([
        DeclareLaunchArgument('session_id'),
        DeclareLaunchArgument('session_json', default_value=''),
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('control', default_value='false'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='false'),
        DeclareLaunchArgument('input', default_value='keyboard',
                              description='keyboard | spacemouse (D1) | freedrive (arm guided by hand with the UR '
                                          'freedrive pedal; keyboard sends commands only, use control:=false)'),
        DeclareLaunchArgument('robot_ip', default_value='', description='overrides rig.yaml robot.ip'),
        DeclareLaunchArgument('episode_manager', default_value='false'),
        DeclareLaunchArgument('qc', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='false'),
        DeclareLaunchArgument('console', default_value='false', description='operator console (no images)'),
        DeclareLaunchArgument('demo_gui', default_value='false',
                              description='read-only demo screen; paused while a real collection session runs'),
        DeclareLaunchArgument('stage', default_value='-1', description='-1 = session.yaml session.stage'),
        DeclareLaunchArgument('operator_id', default_value='', description='default: first of session.operators'),
        DeclareLaunchArgument('phantom_id', default_value='', description='default: session.phantom_id'),

        inc('micrus.launch.py', {'session_id': L('session_id'), 'sim': L('sim')}),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'wrist_camera.launch.py'])),
            condition=IfCondition(L('camera'))),
        inc('robot.launch.py', {'sim': L('sim'), 'control': L('control'),
                                'session_json': L('session_json'), 'robot_ip': L('robot_ip')}),
        Node(package='usrig_teleop', executable='keyboard_teleop', output='screen',
             parameters=params + [{'publish_twist': ParameterValue(PythonExpression(["'", L('input'), "' == 'keyboard'"]), value_type=bool)}],
             condition=IfCondition(L('teleop'))),
        Node(package='spacenav', executable='spacenav_node', output='screen',
             condition=IfCondition(PythonExpression(["'", L('teleop'), "' == 'true' and '", L('input'), "' == 'spacemouse'"]))),
        Node(package='usrig_teleop', executable='spacemouse_teleop', output='screen', parameters=params,
             condition=IfCondition(PythonExpression(["'", L('teleop'), "' == 'true' and '", L('input'), "' == 'spacemouse'"]))),
        Node(package='usrig_qc', executable='qc_monitor', output='screen',
             parameters=params, condition=IfCondition(L('qc'))),
        Node(package='usrig_episode', executable='episode_manager', output='screen',
             parameters=params + [{'robot_control': ParameterValue(L('control'), value_type=bool),
                                   'camera': ParameterValue(L('camera'), value_type=bool),
                                   'stage': ParameterValue(L('stage'), value_type=int),
                                   'operator_id': ParameterValue(L('operator_id'), value_type=str),
                                   'phantom_id': ParameterValue(L('phantom_id'), value_type=str),
                                   'collection_mode': ParameterValue(PythonExpression(
                                       ["'freedrive' if '", L('input'), "' == 'freedrive' else 'robot'"]), value_type=str)}],
             condition=IfCondition(L('episode_manager'))),
        Node(package='usrig_episode', executable='rig_static_tf', output='screen', parameters=params),
        Node(package='usrig_dashboard', executable='dashboard', output='screen',
             parameters=params, condition=IfCondition(L('dashboard'))),
        Node(package='usrig_dashboard', executable='operator_console', output='screen',
             parameters=params, condition=IfCondition(L('console'))),
        Node(package='usrig_demo_gui', executable='demo_gui', output='screen', arguments=['--source', 'ros'],
             condition=IfCondition(L('demo_gui'))),
    ])
