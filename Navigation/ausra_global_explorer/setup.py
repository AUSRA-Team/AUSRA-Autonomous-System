from setuptools import setup
import os
from glob import glob

package_name = 'ausra_global_explorer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'frontier_coordinator = ausra_global_explorer.frontier_coordinator:main',
            'robot_obstacle_publisher = ausra_global_explorer.robot_obstacle_publisher:main',
        ],
    },
)
