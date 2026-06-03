# ============================================================
# FILE: setup.py
# RUNS ON: All machines (Robot 1, Robot 2, Robot 3, Laptop)
# PURPOSE: ROS2 Python package setup for ausra_comms.
#          Registers relay_node and fake_robot_pub as console scripts.
#          Installs launch files and config files.
#
# PLACEHOLDERS: None
# ============================================================

import os
from glob import glob
from setuptools import setup

package_name = 'ausra_comms'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # --- Package index registration ---
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # --- Package manifest ---
        ('share/' + package_name, ['package.xml']),
        # --- Launch files ---
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # --- Config files (bridge YAMLs) ---
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # --- Shell scripts ---
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.com',
    description='Communication layer and launch files for 3-robot swarm Search and Rescue system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'relay_node = ausra_comms.relay_node:main',
            'fake_robot_pub = ausra_comms.fake_robot_pub:main',
        ],
    },
)
