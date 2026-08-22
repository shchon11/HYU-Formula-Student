# Copyright 2026 simseunghwan
#
# Licensed under the MIT License.

from setuptools import setup


package_name = 'drive_udp_bridge'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config',
         [
             'config/drive_udp_bridge.yaml',
             'config/plotjuggler_drive.xml',
             'config/steering_kinematics.csv',
         ]),
        ('share/' + package_name + '/launch', ['launch/drive_udp_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='simseunghwan',
    maintainer_email='simseunghwan@example.com',
    description=(
        'Exchange drive commands and per-wheel encoder speeds with a Speedgoat ECU over UDP.'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'drive_udp_bridge = drive_udp_bridge.drive_udp_bridge_node:main',
            'cmd_pattern = drive_udp_bridge.cmd_pattern:main',
        ],
    },
)
