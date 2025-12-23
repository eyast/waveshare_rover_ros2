# MotionCal Python - Installation Guide

## Quick Start

### Using Conda (Recommended)

```bash
# Create conda environment
conda create -n motioncal python=3.10

# Activate environment
conda activate motioncal

# Install dependencies
cd Python/
pip install -e .
```

### Using pip

```bash
cd Python/
pip install -e .
```

## Dependencies

The following packages will be installed:
- **PyQt6** (>=6.4.0) - GUI framework
- **PyOpenGL** (>=3.1.6) - OpenGL bindings for 3D visualization
- **PyOpenGL-accelerate** (>=3.1.6) - Performance acceleration
- **numpy** (>=1.21.0) - Numerical operations
- **scipy** (>=1.7.0) - Scientific computing (eigenvalues)
- **pyserial** (>=3.5) - Serial port communication

## Platform-Specific Notes

### macOS
PyQt6 OpenGL widgets are included in the main PyQt6 package on macOS.

### Linux
On some Linux distributions, you may need to install additional packages:
```bash
# Ubuntu/Debian
sudo apt-get install python3-pyqt6.qtopengl python3-opengl

# Or with pip
pip install PyQt6-OpenGL
```

### Windows
PyQt6 should install with OpenGL support by default. If you encounter issues:
```bash
pip install PyQt6-OpenGL
```

## Verification

Test that everything is installed correctly:

```bash
# Test import
python -c "import motioncal; print('✓ Package installed')"

# Test GUI import
python -c "from motioncal.gui.main_window import MainWindow; print('✓ GUI ready')"

# Run application
motioncal
```

## Troubleshooting

### ImportError: cannot import name 'QOpenGLWidget'

This means PyQt6 OpenGL widgets are not installed. Fix with:
```bash
pip install PyQt6-OpenGL
```

### OpenGL errors on Linux

Install Mesa OpenGL libraries:
```bash
sudo apt-get install libgl1-mesa-glx libglu1-mesa
```

### Serial port access denied (Linux)

Add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Serial port access denied (macOS)

No special permissions needed on macOS, but ensure you have the correct driver for your USB-serial adapter.

## Running Tests

```bash
cd Python/
python -m unittest discover -s tests -p "test_*.py" -v
```

All 72 tests should pass.

## Uninstallation

```bash
pip uninstall motioncal
```

Or delete the conda environment:
```bash
conda deactivate
conda env remove -n motioncal
```
