"""UR7e nodes. control:=false runs robot_state only (read-only RTDE).
control:=true runs robot_state + robot_control in one process (refuses while limits are null).
sim:=true points both at URSim (rig.yaml robot.ursim_ip).
"""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def config_dir():
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def _setup(context):
    cfg = config_dir()
    with open(cfg / 'rig.yaml') as f:
        rig = yaml.safe_load(f)
    sim = LaunchConfiguration('sim').perform(context).lower() == 'true'
    control = LaunchConfiguration('control').perform(context).lower() == 'true'
    ip = rig['robot']['ursim_ip'] if sim else rig['robot']['ip']
    params = [{'config_dir': str(cfg), 'robot_ip': ip or '',
               'frequency_hz': float(rig['robot']['state_rate_hz']),
               'session_json': LaunchConfiguration('session_json').perform(context),
               'use_sim_time': False}]
    exe = 'robot_control' if control else 'robot_state'
    # No name= here: robot_control hosts two nodes (robot_state + robot_control) in one process
    # and a name remap would give both the same name.
    return [Node(package='usrig_robot', executable=exe, output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('control', default_value='false'),
        DeclareLaunchArgument('session_json', default_value=''),
        OpaqueFunction(function=_setup),
    ])
