from setuptools import setup

package_name = 'usrig_freedrive'

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
    description='Freedrive (kinesthetic) collection: sample labelling rule and the guided freedrive check.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'freedrive_check = usrig_freedrive.freedrive_check:main',
    ]},
)
