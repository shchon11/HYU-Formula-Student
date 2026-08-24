import os
from glob import glob

from setuptools import setup

package_name = 'hyu_lite_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shchon11',
    maintainer_email='shchon724@gmail.com',
    description='Lightweight vehicle/sensor/ECU/perception simulator for the FSK stack (no Gazebo).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lite_sim = hyu_lite_sim.lite_sim_node:main',
            'clutter_tool = hyu_lite_sim.clutter_tool:main',
        ],
    },
)
