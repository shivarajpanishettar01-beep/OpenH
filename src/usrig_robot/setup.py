from setuptools import setup

package_name = 'usrig_robot'

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
    description='UR7e state and control for the ultrasound rig.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state = usrig_robot.robot_state:main',
            'robot_control = usrig_robot.robot_control:main',
            'rig_measure = usrig_robot.rig_measure:main',
            'pivot_calib = usrig_robot.pivot_calib:main',
        ],
    },
)
