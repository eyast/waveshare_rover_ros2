# Waveshare Rover ROS2

A comprehensive ROS2 robotics stack for the Waveshare WAVE ROVER platform, featuring custom LiDAR integration, enhanced ESP32 firmware with IMU calibration support, and autonomous navigation capabilities.

## Table of Contents

- [Overview](#overview)
- [Components](#components)
  - [LiDAR Interface](#1-lidar-interface)
  - [ESP32 Driver](#2-esp32-driver)
  - [MotionCal (Mac-Compatible)](#3-motioncal-mac-compatible)
  - [PyRover Library](#4-pyrover-library)
  - [Custom ROS2 Messages](#5-custom-ros2-messages)
  - [ROS2 Navigation Package](#6-ros2-navigation-package)
- [Hardware Requirements](#hardware-requirements)
- [Quick Start](#quick-start)

---

## Overview

This repository provides everything needed to build an autonomous mobile robot using the Waveshare WAVE ROVER chassis. It includes:

- Custom YDLidar X4 Pro driver with ROS2 compatibility
- Enhanced ESP32 firmware enabling proper IMU calibration via MotionCal
- Mac-compatible MotionCal Python port (fixes CRLF line ending issues)
- Pure Python robot control library (no ROS2 dependencies)
- Custom ROS2 message definitions
- Complete ROS2 navigation stack implementation

---

## Components

### 1. LiDAR Interface

**Location:** `lidar_interface/`

A hardware-agnostic LiDAR abstraction layer custom-built for the **YDLidar X4 Pro** with full ROS2 compatibility.

#### Why It's Important

The off-the-shelf YDLidar ROS2 packages often have compatibility issues and lack flexibility. This custom implementation provides:

- Clean Python API with standardized data structures
- Automatic port detection
- Configurable scan frequency and sample rates
- Easy integration with ROS2 or standalone Python applications

#### Architecture

```
lidar_interface/
├── base.py              # Abstract LidarBase class and data structures
├── drivers/
│   ├── ydlidar.py       # YDLidar X4 Pro driver implementation
│   └── dummy.py         # Test driver with synthetic data
└── __init__.py
```

#### Installation

```bash
# Install YDLidar SDK first (see YDLidar documentation)
cd lidar_interface
pip install -e .
```

#### Usage

```python
from lidar_interface import YDLidarDriver, LidarConfig

config = LidarConfig(
    port="/dev/ttyUSB0",
    baudrate=128000,
    scan_frequency=10.0,
    range_min=0.1,
    range_max=12.0
)

with YDLidarDriver(config) as lidar:
    scan = lidar.get_scan()
    print(f"Got {len(scan.ranges)} points")
```

---

### 2. ESP32 Driver

**Location:** `rover_esp_driver/ESPDriverPlatformIO/`

An enhanced version of the Waveshare ESP32 driver firmware that adds **MotionCal calibration support** for the onboard IMU.

#### Why It's Important

The stock Waveshare firmware does not support magnetometer calibration, leading to poor heading estimation. This enhanced driver:

- Outputs IMU data in MotionCal-compatible ASCII format for calibration
- Implements Madgwick AHRS filter with calibrated magnetometer
- Provides JSON streaming for real-time sensor fusion
- Stores calibration data persistently

#### Hardware Configuration

| I2C Address | Device    | Function              |
|-------------|-----------|----------------------|
| 0x6B        | QMI8658C  | 6-DOF IMU (Accel/Gyro) |
| 0x0C        | AK09918C  | 3-axis Magnetometer   |
| 0x3C        | SSD1306   | OLED Display          |
| 0x42        | INA219    | Power Monitor         |

#### Key Commands

| Command | Type | Description |
|---------|------|-------------|
| `{"T":1,"L":0.3,"R":0.3}` | Speed Control | Set wheel speeds (-0.5 to 0.5) |
| `{"T":325,"cmd":1}` | IMU Stream | Enable IMU streaming |
| `{"T":400,"fmt":1}` | Stream Format | 0=JSON, 1=MotionCal ASCII |
| `{"T":126}` | Request IMU | Get single IMU reading |

#### IMU Data Output (JSON Mode)

```json
{
  "T": 326,
  "ax": 0.01, "ay": -0.02, "az": 0.98,
  "gx": 0.001, "gy": -0.002, "gz": 0.0,
  "mx": 25.3, "my": -12.1, "mz": 45.6,
  "pitch": 1.2, "roll": -0.8, "yaw": 127.5,
  "dt": 0.02
}
```

#### Installation

```bash
# Using PlatformIO
cd rover_esp_driver/ESPDriverPlatformIO
pio run --target upload
```

#### Calibration Workflow

1. Flash the enhanced firmware
2. Set output to MotionCal format: `{"T":400,"fmt":1}`
3. Enable streaming: `{"T":325,"cmd":1}`
4. Run MotionCal and rotate the robot in all orientations
5. Send calibration data back to ESP32
6. Switch back to JSON mode: `{"T":400,"fmt":0}`

---

### 3. MotionCal (Mac-Compatible)

**Location:** `MotionCal-master/Python/`

A complete Python port of the [MotionCal](https://github.com/PaulStoffregen/MotionCal) sensor calibration tool, **fixed to work on macOS**.

#### Why It's Important

The original MotionCal has issues on macOS due to **CRLF line ending handling** in the serial protocol. This Python port:

- Correctly parses both `\r\n` and `\n` line endings
- Provides native PyQt6 GUI that works on macOS
- Includes OpenGL 3D visualization of magnetometer coverage
- Implements all calibration algorithms (4/7/10 element)

#### Features

- **Calibration Algorithms:** Hard iron offset, scale factors, full soft iron matrix
- **Quality Metrics:** Gap detection, variance, wobble, fit error
- **Visualization:** Real-time 3D sphere showing magnetometer coverage
- **Mahony AHRS:** 9-DOF sensor fusion for orientation display

#### Installation

```bash
cd MotionCal-master/Python
pip install -e .
```

#### Usage

```bash
# Launch the GUI
motioncal

# Run tests
pytest tests/
```

#### Serial Protocol

**Input (from device):**
```
Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r\n
```

**Output (to device):**
68-byte binary calibration packet with CRC16 checksum.

---

### 4. PyRover Library

**Location:** `pyrover/`

A pure Python library for controlling the Waveshare WAVE ROVER **without requiring ROS2**.

#### Why It's Important

Not every application needs a full ROS2 stack. PyRover provides:

- Simple Pythonic API for robot control
- No ROS2 dependencies
- Direct serial communication with the ESP32
- Battery monitoring and estimation

#### Installation

```bash
cd pyrover
pip install -e .
```

#### Usage

```python
from pyrover import PyRover

with PyRover(port="/dev/serial0") as rover:
    # Move forward
    rover.set_speed(left=0.3, right=0.3)

    # Get IMU data
    imu = rover.get_imu()
    print(f"Heading: {imu.yaw}°")

    # Get battery status
    chassis = rover.get_chassis_info()
    print(f"Battery: {chassis.voltage}V")

    # Display on OLED
    rover.set_oled_line(0, "Hello World")
```

#### Available Commands

```python
rover.set_speed(left, right)      # Speed control (-0.5 to 0.5)
rover.set_pwm(left, right)        # Direct PWM (-255 to 255)
rover.get_imu()                   # Returns IMUData
rover.get_chassis_info()          # Returns ChassisInfo
rover.set_oled_line(line, text)   # Display text (4 lines)
rover.set_motor_pid(p, i, d)      # Configure PID
```

#### CLI Tools

```bash
# Motor calibration utility
pyrover-calibrate-motors --port /dev/serial0
```

---

### 5. Custom ROS2 Messages

**Location:** `rover_msgs/`

Custom ROS2 message definitions for robot sensor data.

#### Message Types

**IMU.msg** - 9-DOF sensor data
```
std_msgs/Header header
float32 pitch, roll, yaw          # Euler angles (degrees)
float32 axx, axy, axz             # Acceleration (milli-g)
int32 gx, gy, gz                  # Gyroscope (raw)
```

**Battery.msg** - Power monitoring
```
std_msgs/Header header
float32 value                     # Voltage or percentage
```

**Wheel.msg** - Wheel odometry
```
std_msgs/Header header
float32 l, r                      # Left/right speeds (m/s)
```

**State.msg** - Robot pose
```
std_msgs/Header header
float32 x, y                      # Position (meters)
float32 theta                     # Heading (radians)
```

#### Building

```bash
cd <ros2_workspace>/src
ln -s /path/to/rover_msgs .
cd ..
colcon build --packages-select rover_msgs
source install/setup.bash
```

---

### 6. ROS2 Navigation Package

**Location:** `rover_ros/`

Complete ROS2 driver package implementing an **autonomous navigation stack from scratch**.

#### Why It's Important

This package demonstrates building a full autonomous stack without relying on Nav2 or other pre-built solutions. It's designed for learning and customization.

#### Architecture

```
rover_ros/
├── rover_ros/
│   ├── rover_controller.py    # Main driver node
│   ├── pose_estimator.py      # Localization algorithms
│   ├── tools.py               # Utilities
│   └── filters/               # Sensor fusion filters
├── launch/
│   └── bringup.launch.py      # Main launch file
└── config/
    └── rover_params.yaml      # Configuration
```

#### Topics

**Subscriptions:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Publications:**
- `/sensor/IMU` (rover_msgs/IMU) - IMU data
- `/sensor/Battery` (rover_msgs/Battery) - Battery level
- `/sensor/Wheel` (rover_msgs/Wheel) - Wheel speeds
- `/sensor/Temperature` (rover_msgs/Temperature) - Board temp

#### Localization Modes

Configure via `rover_params.yaml`:

- `dead_reckoning` - Wheel odometry only
- `imu` - IMU with Mahony fusion
- `bayes` - Bayesian filtering
- `kalman` - Kalman filter

#### Installation

```bash
cd <ros2_workspace>/src
ln -s /path/to/rover_ros .
ln -s /path/to/rover_msgs .
cd ..
colcon build
source install/setup.bash
```

#### Usage

```bash
# Launch the driver
ros2 launch rover_ros bringup.launch.py

# With custom config
ros2 launch rover_ros bringup.launch.py \
    port:=/dev/ttyUSB0 \
    config:=/path/to/params.yaml

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist \
    "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

#### Configuration (rover_params.yaml)

```yaml
# Serial connection
port: "/dev/serial0"
baudrate: 115200
max_speed: 0.5

# Motor calibration
linear_gain: 1.0
angular_gain: 0.333

# Publishing
publish_imu: true
publish_battery: true
publish_tf: true
publish_rate: 20.0

# Frames
frame_id: "base_link"
odom_frame_id: "odom"

# Battery
battery_cells: 3
battery_capacity_mah: 2600

# Localization
localization_mode: imu
```

---

## Hardware Requirements

- **Platform:** Waveshare WAVE ROVER or UGV01/UGV02
- **Controller:** ESP32 with QMI8658C IMU and AK09918C magnetometer
- **LiDAR:** YDLidar X4 Pro (optional, for mapping/navigation)
- **Computer:** Raspberry Pi 4, Jetson Nano, or similar (ROS2 compatible)

## Quick Start

### 1. Flash the ESP32

```bash
cd rover_esp_driver/ESPDriverPlatformIO
pio run --target upload
```

### 2. Calibrate the IMU

```bash
cd MotionCal-master/Python
pip install -e .
motioncal
# Rotate robot, send calibration when complete
```

### 3. Test with PyRover

```bash
cd pyrover
pip install -e .
python -c "
from pyrover import PyRover
with PyRover() as r:
    r.set_speed(0.2, 0.2)
    import time; time.sleep(1)
    r.set_speed(0, 0)
"
```

### 4. Launch ROS2 Stack

```bash
# Build packages
cd ~/ros2_ws/src
ln -s /path/to/rover_ros .
ln -s /path/to/rover_msgs .
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch
ros2 launch rover_ros bringup.launch.py
```

---

## License

MIT License - See individual package directories for details.

## Contributing

Contributions welcome! Please open an issue or pull request on GitHub.
