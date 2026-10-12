from glob import glob

from setuptools import setup

package_name = 'usrig_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shivaraj Panishettar',
    maintainer_email='shivarajpanishettar01@gmail.com',
    description='Configuration and launch files for the ultrasound rig.',
    license='Proprietary',
)
