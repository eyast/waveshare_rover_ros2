# IMU Visualizer

Real-time 3D orientation visualization and sensor data plotting for IMU devices.

## Features

- **Left Panel**: 3D arrow showing IMU orientation with XYZ reference frame
- **Right Panel**: 10 real-time scrolling plots:
  - Roll, Pitch, Yaw (orientation)
  - Accelerometer X, Y, Z
  - Gyroscope X, Y, Z
  - Temperature
- **Serial Communication**: Reads from `/dev/serial0` and sends `{"T":126}` every 20 seconds

## Installation

```bash
# Create virtual environment (optional but recommended)
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# On some systems you may also need:
sudo apt-get install python3-opengl
```

## Usage

### Normal Mode (with hardware)

```bash
python3 imu_viz.py
```

### Demo Mode (no hardware needed)

```bash
python3 imu_viz.py --demo
```

### Custom Serial Port

```bash
python3 imu_viz.py --port=/dev/ttyUSB0 --baud=115200
```

## Expected IMU JSON Format

The visualizer accepts JSON data with flexible key names. Examples:

```json
{"roll":1.2,"pitch":3.4,"yaw":5.6,"ax":0.1,"ay":0.2,"az":9.8,"gx":1.0,"gy":2.0,"gz":3.0,"temp":25.5}
```

Or Waveshare-style:
```json
{"T":126,"Roll":1.2,"Pitch":3.4,"Yaw":5.6,"accX":0.1,"accY":0.2,"accZ":9.8}
```

Supported key mappings:
- **Roll**: `roll`, `r`, `Roll`
- **Pitch**: `pitch`, `p`, `Pitch`
- **Yaw**: `yaw`, `y`, `Yaw`
- **Accel**: `ax/ay/az`, `accX/accY/accZ`, `accel_x/accel_y/accel_z`
- **Gyro**: `gx/gy/gz`, `gyroX/gyroY/gyroZ`, `gyro_x/gyro_y/gyro_z`
- **Temp**: `temp`, `temperature`

## Controls

- **ESC**: Exit
- **SPACE**: Manually send `{"T":126}` command

## Configuration

Edit the `Config` class in `imu_viz.py` to customize:

```python
@dataclass
class Config:
    serial_port: str = "/dev/serial0"
    baud_rate: int = 115200
    command_interval: float = 20.0  # seconds
    window_width: int = 1400
    window_height: int = 800
    fps: int = 60
    plot_history: int = 200  # data points per plot
```

## Troubleshooting

### Permission denied on serial port
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### OpenGL errors on Raspberry Pi
```bash
sudo apt-get install libgl1-mesa-dri
```

### No display / headless mode
This requires a display. For headless systems, use X11 forwarding:
```bash
ssh -X user@raspberrypi
python3 imu_viz.py --demo
```