"""A RealSense camera via realsense2_camera's rs_launch.py, settings from session.yaml.

camera:=wrist_camera (default, the wrist D415) or camera:=tpv_camera (the fixed third-person view camera,
tpv_camera.launch.py): the session.yaml block to start.

Refuses to start while serial, exposure or white balance are null, unless auto:=true
(auto exposure / auto white balance, only for sampling values with camera_autolock).
Settings that rs_launch.py does not accept as launch arguments (session.yaml rs_params) are
written to a parameter file and passed as its config_file.
"""
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def config_dir():
    env = os.environ.get('USRIG_CONFIG_DIR')
    if env:
        return Path(env).expanduser()
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('usrig_bringup')) / 'config'


def _setup(context):
    import tempfile
    from usrig_common import config               # shared session.yaml + this PC's machine.yaml
    name = LaunchConfiguration('camera').perform(context)
    if name not in config.CAMERAS:
        return [LogInfo(msg=f'camera:={name}: not one of {sorted(config.CAMERAS)}'), Shutdown(reason='unknown camera')]
    cam = config.load_config('session', str(config_dir()))[name]
    if not cam.get('enabled'):
        return [LogInfo(msg=f'{name}: disabled on this PC ({name}.enabled false, see machine.yaml); not starting')]
    auto = LaunchConfiguration('auto').perform(context).lower() == 'true'
    missing = [k for k in ['serial'] + ([] if auto else ['exposure_us', 'white_balance_k']) if cam.get(k) is None]
    if missing:
        return [LogInfo(msg=f'{name}: null in session.yaml: {missing}; not starting'),
                Shutdown(reason=f'{name} settings missing')]
    try:
        from ament_index_python.packages import get_package_share_directory
        rs_share = get_package_share_directory('realsense2_camera')
    except Exception:
        return [LogInfo(msg='realsense2_camera is not installed (apt install ros-humble-realsense2-camera)'),
                Shutdown(reason='realsense2_camera missing')]
    args = {k: str(v).format(**cam) for k, v in cam['rs_launch_args'].items()}
    params = {}
    for k, v in (cam.get('rs_params') or {}).items():
        if isinstance(v, str):
            v = yaml.safe_load(v.format(**cam))
        params[k] = v
    if auto:
        args['rgb_camera.enable_auto_exposure'] = 'true'
        params = {k: v for k, v in params.items() if k not in ('rgb_camera.exposure', 'rgb_camera.white_balance')}
        params['rgb_camera.enable_auto_white_balance'] = True
    f = tempfile.NamedTemporaryFile('w', prefix=f'usrig_{name}_', suffix='.yaml', delete=False)
    yaml.safe_dump(params, f)
    f.close()
    args['config_file'] = f.name
    return [LogInfo(msg=f'{name}: {"AUTO exposure/WB (sampling only)" if auto else "locked exposure/WB"}; '
                        f'params {params}'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(rs_share, 'launch', 'rs_launch.py')),
                launch_arguments=args.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='wrist_camera',
                              description='session.yaml camera block: wrist_camera | tpv_camera'),
        DeclareLaunchArgument('auto', default_value='false',
                              description='auto exposure / WB, only for camera_autolock sampling'),
        OpaqueFunction(function=_setup)])
