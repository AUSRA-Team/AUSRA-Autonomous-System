from setuptools import setup
import os
from glob import glob

package_name = 'ausra_numpad_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdelrhman',
    maintainer_email='abdelrhman048@gmail.com',
    description='Keyboard teleoperation for AUSRA robot using numpad',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'numpad_teleop = ausra_numpad_teleop.numpad_teleop_node:main',
        ],
    },
)
