"""Wrist RealSense D415 via realsense2_camera's rs_launch.py, settings from session.yaml.

Refuses to start while serial, exposure or white balance are null.
"""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def config_dir():
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def _setup(context):
    with open(config_dir() / 'session.yaml') as f:
        cam = yaml.safe_load(f)['wrist_camera']
    missing = [k for k in ('serial', 'exposure_us', 'white_balance_k') if cam.get(k) is None]
    if missing:
        return [LogInfo(msg=f'wrist_camera: null in session.yaml: {missing}; not starting'),
                Shutdown(reason='wrist camera settings missing')]
    try:
        from ament_index_python.packages import get_package_share_directory
        rs_share = get_package_share_directory('realsense2_camera')
    except Exception:
        return [LogInfo(msg='realsense2_camera is not installed (apt install ros-humble-realsense2-camera)'),
                Shutdown(reason='realsense2_camera missing')]
    args = {k: str(v).format(**cam) for k, v in cam['rs_launch_args'].items()}
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
        launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
