import os
from glob import glob
from setuptools import setup

package_name = 'ausra_comms'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json5')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.com',
    description='Jetson relay node for AUSRA swarm — namespaces and throttles SLAM topics for DDS',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'relay_node = ausra_comms.relay_node:main',
            'map_decompressor_node = ausra_comms.map_decompressor_node:main',
        ],
    },
)
