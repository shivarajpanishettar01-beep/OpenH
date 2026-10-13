from glob import glob

from setuptools import setup

package_name = 'usrig_dashboard'

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
    description='Operator dashboard for the ultrasound rig.',
    license='MIT',
    entry_points={'console_scripts': [
        'dashboard = usrig_dashboard.dashboard:main',
        'annotator = usrig_dashboard.annotator:main',
    ]},
)
