from glob import glob

from setuptools import setup

package_name = 'usrig_demo_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.sources', package_name + '.widgets'],
    package_data={package_name: ['assets/*', 'assets/fonts/*']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Shivaraj Panishettar',
    maintainer_email='shivarajpanishettar01@gmail.com',
    description='Read-only demo / monitoring GUI for the UR7e robotic ultrasound rig (mock data first).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'demo_gui = usrig_demo_gui.app:main',
    ]},
)
