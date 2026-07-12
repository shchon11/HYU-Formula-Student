from setuptools import setup

package_name = 'eufs_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shchon11',
    maintainer_email='shchon724@gmail.com',
    description='Comfortable keyboard teleop for the EUFS simulator.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop = eufs_teleop.teleop_node:main',
        ],
    },
)
