from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'px4_takeoff'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hann',
    maintainer_email='hann@ieee.org',
    description='Simple PX4 offboard takeoff to 5.0m using px4_msgs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'takeoff_node = px4_takeoff.takeoff_node:main',
            'attitude_control_node = px4_takeoff.attitude_control_node:main',
            'mission_node = px4_takeoff.mission_node:main',
        ],
    },
)
