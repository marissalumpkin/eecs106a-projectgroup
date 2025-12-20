from setuptools import setup
import os
from glob import glob

package_name = 'morphing_airfoil'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Airfoil Team',
    maintainer_email='team@example.com',
    description='ROS2 Control package for Morphing Airfoil Project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = morphing_airfoil.controller_node:main',
            'pid_controller = morphing_airfoil.pid_controller_node:main',
            'sensor_sim = morphing_airfoil.sensor_sim_node:main',
            'actuator_sim = morphing_airfoil.actuator_sim_node:main',
        ],
    },
)