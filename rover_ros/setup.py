"""
Setup configuration for ros_rover ROS2 package.
"""

import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rover_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    
    data_files=[
        # Package marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package.xml
        ('share/' + package_name, ['package.xml']),
        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    
    install_requires=[
        'setuptools',
    ],
    
    zip_safe=True,
    
    maintainer='Eyas Taifour',
    maintainer_email='etaifour@me.com',
    description='ROS2 driver for PyRover (Waveshare ROVER)',
    license='MIT',
    
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            'rover_controller = rover_ros.rover_controller:main',
            'pose_estimator = rover_ros.pose_estimator:main',
        ],
    },
    
    python_requires='>=3.8',
)
