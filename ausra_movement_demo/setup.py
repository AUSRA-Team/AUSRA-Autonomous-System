from setuptools import setup
import os
from glob import glob

package_name = 'ausra_movement_demo'

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
    description='Holonomic movement demonstration node for AUSRA robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holonomic_demo = ausra_movement_demo.holonomic_movement_demo:main',
        ],
    },
)
