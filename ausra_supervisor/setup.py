from setuptools import setup

package_name = 'ausra_supervisor'

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
    maintainer='AUSRA Team',
    maintainer_email='ausra@team.com',
    description='Per-robot supervisor sidecar: health watchdogs, task FSM, e-stop, fleet heartbeat',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'supervisor_node = ausra_supervisor.supervisor_node:main',
        ],
    },
)
