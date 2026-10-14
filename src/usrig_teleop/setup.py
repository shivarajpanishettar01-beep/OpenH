from setuptools import setup

package_name = 'usrig_teleop'

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
    description='Teleoperation mode: keyboard and SpaceMouse input, teleop_check.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'keyboard_teleop = usrig_teleop.keyboard_teleop:main',
        'spacemouse_teleop = usrig_teleop.spacemouse_teleop:main',
        'teleop_check = usrig_teleop.teleop_check:main',
    ]},
)
