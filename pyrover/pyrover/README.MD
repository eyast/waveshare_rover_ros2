# PyRover - Waveshare WAVE ROVER Python Library

A Python library for controlling Waveshare WAVE ROVER robots using the new line-based serial protocol.

## Protocol Overview

The new ESP32 driver uses a simple line-based protocol instead of JSON:

### Output (Device → Host)

| Prefix | Description | Format |
|--------|-------------|--------|
| `I:` | IMU telemetry | `yaw,pitch,roll,temp,ax,ay,az,gx,gy,gz,mx,my,mz` |
| `P:` | Power data | `voltage,current,power,shunt` |
| `Raw:` | MotionCal raw | `ax,ay,az,gx,gy,gz,mx,my,mz` |
| `Ori:` | MotionCal orientation | `yaw,pitch,roll` |
| `S:` | System message | `module,message` |
| `A:` | Acknowledgment | `command[,details]` |
| `E:` | Error | `source,message` |

### Input (Host → Device)

| Command | Description |
|---------|-------------|
| `M:left,right` | Motor speeds (-255 to 255) |
| `STOP` | Stop motors |
| `ESTOP` | Emergency stop |
| `ENABLE` | Re-enable after ESTOP |
| `HB` | Heartbeat |
| `STREAM:ON/OFF` | Enable/disable telemetry |
| `FMT:RAW/IMU` | Switch output format |
| `CAL:G/A/ALL` | Calibrate sensors |
| `BETA:value` | Set filter beta |
| `FAST:duration` | Fast convergence |
| `INIT` | Re-init filter |
| `WS:ssid,pass,ip[,port]` | Connect WiFi/WebSocket |
| `WS:OFF` | Disconnect WebSocket |
| `STATUS` | Request status |
| `REBOOT` | Restart device |

## Installation

```bash
pip install pyserial
```

## Usage

### Standalone (No ROS)

```python
from pyrover import PyRover, RoverCallbacks, IMUData

def on_imu(data: IMUData):
    print(f"Yaw: {data.yaw:.1f}°")

callbacks = RoverCallbacks(on_imu=on_imu)

with PyRover('/dev/serial0', callbacks=callbacks) as rover:
    rover.stream_on()
    rover.set_format_imu()
    
    rover.move(100, 100)  # Forward
    time.sleep(2)
    rover.stop()
```

### With ROS2

1. Build the rover_msgs package with the message definitions
2. Use rover_controller.py as your ROS2 node:

```bash
ros2 run pyrover rover_controller --ros-args -p port:=/dev/serial0
```

## Data Types

- `IMUData`: Full IMU telemetry (orientation + sensors)
- `PowerData`: Battery voltage, current, power
- `RawSensorData`: MotionCal format raw sensor values
- `Orientation`: Just yaw/pitch/roll
- `BatteryEstimator`: Convert voltage to SoC percentage

## Files

- `rover.py` - Main PyRover class
- `data_types.py` - Data structures
- `commands.py` - Protocol constants
- `tools.py` - ROS2 utilities
- `rover_controller.py` - ROS2 driver node
- `pose_estimator.py` - ROS2 localization node
- `example.py` - Standalone example