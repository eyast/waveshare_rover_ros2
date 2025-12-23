"""
MotionCal Python - Sensor Calibration Tool

Python port of Paul Stoffregen's MotionCal sensor calibration application.
"""

from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="motioncal",
    version="1.0.0",
    description="Motion Sensor Calibration Tool - Python port of MotionCal",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="MotionCal Python Port",
    author_email="",
    url="https://github.com/pjrc/MotionCal",
    license="BSD-3-Clause",

    packages=find_packages(),

    install_requires=[
        "PyQt6>=6.4.0",
        "PyOpenGL>=3.1.6",
        "PyOpenGL-accelerate>=3.1.6",
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "pyserial>=3.5",
    ],

    entry_points={
        "console_scripts": [
            "motioncal=motioncal.__main__:main",
        ],
        "gui_scripts": [
            "motioncal-gui=motioncal.__main__:main",
        ],
    },

    package_data={
        "motioncal.gui": ["resources/*.png"],
    },

    python_requires=">=3.8",

    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: BSD License",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: OS Independent",
    ],
)
