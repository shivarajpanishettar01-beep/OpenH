from glob import glob

from setuptools import setup

package_name = 'usrig_presets'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shivaraj Panishettar',
    maintainer_email='shivarajpanishettar01@gmail.com',
    description='Preset waypoints, Stage 1 paths, Stage 2 plans and the preset motion check.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'waypoint_teach = usrig_presets.waypoints:teach_main',
        'waypoint_verify = usrig_presets.waypoints:verify_main',
        'path_editor = usrig_presets.path_editor:main',
        'plan_generator = usrig_presets.plan_generator:main',
        'preset_check = usrig_presets.preset_check:main',
    ]},
)
