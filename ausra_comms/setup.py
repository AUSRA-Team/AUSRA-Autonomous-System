# ============================================================
# FILE: setup.py
# PACKAGE: ausra_comms
# RUNS ON: Jetson (ausra_1, ausra_2)
# PURPOSE: Registers relay_node as a console script.
#          Installs launch files for Jetson hardware + comms.
#
# NOTE: This is the JETSON-ONLY package. The base station
#       package is ausra_comms_base (runs on laptop).
# ============================================================

import os
from glob import glob
from setuptools import setup

package_name = 'ausra_comms'

setup(
    name=package_name,
    version='0.2.0',
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
        # --- Zenoh bridge config (JSON5) ---
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
        ],
    },
)
