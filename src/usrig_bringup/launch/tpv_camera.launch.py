"""Third-person view (TPV) RealSense: wrist_camera.launch.py with camera:=tpv_camera (session.yaml tpv_camera).

  ros2 launch usrig_bringup tpv_camera.launch.py [auto:=true]
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('auto', default_value='false',
                              description='auto exposure / WB, only for camera_autolock sampling'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([ThisLaunchFileDir(), 'wrist_camera.launch.py'])),
            launch_arguments={'camera': 'tpv_camera', 'auto': LaunchConfiguration('auto')}.items())])
