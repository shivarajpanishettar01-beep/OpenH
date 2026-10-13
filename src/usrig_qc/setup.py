from setuptools import setup

package_name = 'usrig_qc'

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
    description='QC for the ultrasound rig.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'qc_monitor = usrig_qc.qc_monitor:main',
        'episode_qc = usrig_qc.episode_qc:main',
        'presession_check = usrig_qc.presession_check:main',
        'stream_check = usrig_qc.stream_check:main',
        'camera_autolock = usrig_qc.camera_autolock:main',
        'auto_annotate = usrig_qc.auto_annotate:main',
        'collection_progress = usrig_qc.collection_progress:main',
        'status_console = usrig_qc.status_console:main',
    ]},
)
