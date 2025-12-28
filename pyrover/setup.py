"""
Setup configuration for PyRover library (v2.0 - Line-Based Protocol).

This is a pure Python library with no ROS2 dependencies.
It can be installed via:
    - pip install .
    - pip install -e .  (for development)
    - colcon build (as part of a ROS2 workspace)
"""

from setuptools import setup, find_packages

package_name = 'pyrover'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', 'tests']),
    
    # ROS2 ament support (optional)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    
    install_requires=[
        'setuptools',
        'pyserial>=3.5',
    ],
    
    extras_require={
        'dev': [
            'pytest>=7.0',
            'pytest-cov',
            'pytest-timeout',
        ],
        'ros2': [
            # ROS2 dependencies are handled by package.xml
        ],
    },
    
    zip_safe=True,
    
    maintainer='Eyas Taifour',
    maintainer_email='etaifour@me.com',
    description='Python library for Waveshare WAVE ROVER robots (line-based protocol)',
    long_description=open('README.md').read() if __import__('os').path.exists('README.md') else '',
    long_description_content_type='text/markdown',
    license='MIT',
    
    tests_require=['pytest'],
    
    # CLI tools
    entry_points={
        'console_scripts': [
            # Calibration tools
            'pyrover-calibrate-motors = pyrover.calibration.motor:run_motor_calibration_cli',
            'pyrover-calibrate-motioncal = pyrover.calibration.remote_motioncal:run_calibrator_cli',
            
            # Test tool
            'pyrover-test = pyrover.tests.test_commands:run_command_test_cli',
        ],
    },
    
    python_requires='>=3.8',
    
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Scientific/Engineering :: Robotics',
    ],
)