from setuptools import setup

package_name = 'usrig_episode'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shivaraj Panishettar',
    maintainer_email='shivarajpanishettar01@gmail.com',
    description='Episode lifecycle (episode_manager, recorder) and calibration tools for the ultrasound rig.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'episode_manager = usrig_episode.episode_manager:main',
        'record_guard = usrig_episode.record_guard:main',
        'rig_static_tf = usrig_episode.static_tf:main',
        'register_phantom = usrig_episode.registration:main',
        'handeye_calib = usrig_episode.handeye:main',
    ]},
)
