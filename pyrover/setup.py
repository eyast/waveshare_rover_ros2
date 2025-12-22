"""
Setup configuration for PyRover library.

This is a pure Python library with no ROS2 dependencies.
It can be installed via:
    - pip install .
    - colcon build (as part of a ROS2 workspace)
"""

from setuptools import setup, find_packages

package_name = 'pyrover'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    
    # ROS2 ament support
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    
    zip_safe=True,
    
    maintainer='Eyas Taifour',
    maintainer_email='etaifour@me.com',
    description='Pure Python library for Waveshare WAVE ROVER robots',
    license='MIT',
    
    tests_require=['pytest'],
    
    # CLI tools
    entry_points={
        'console_scripts': [
            # Calibration tools
            'pyrover-calibrate-motors = pyrover.calibration.motor:run_motor_calibration_cli',
        ],
    },
    
    python_requires='>=3.8',
)
