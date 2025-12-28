# Rover Driver

A clean, FreeRTOS-based driver for Waveshare rover with IMU, magnetometer, and power monitoring.

## Features

- **FreeRTOS tasks** for proper separation of concerns
- **Simple serial protocol** with prefixes for easy Python routing
- **MotionCal compatibility** for magnetometer calibration
- **Watchdog monitoring** for system reliability
- **Optional WebSocket** for remote telemetry

## Hardware

- ESP32 microcontroller
- QMI8658C 6-axis IMU (accelerometer + gyroscope)
- AK09918C 3-axis magnetometer
- INA219 power monitor
- Dual H-bridge motor driver

## Protocol

All messages use a simple line-based format:

```
PREFIX:data,data,data
```

### Output Messages (Device → Host)

| Prefix | Description | Format |
|--------|-------------|--------|
| `Raw:` | MotionCal raw data | `Raw:ax,ay,az,gx,gy,gz,mx,my,mz` |
| `Ori:` | MotionCal orientation | `Ori: yaw,pitch,roll` |
| `I:` | IMU telemetry | `I:yaw,pitch,roll,temp,ax,ay,az,gx,gy,gz,mx,my,mz` |
| `P:` | Power data | `P:voltage_V,current_mA,power_mW,shunt_mV` |
| `S:` | System message | `S:message` or `S:message,value` |
| `A:` | Acknowledgment | `A:command` or `A:command,detail` |
| `E:` | Error | `E:message` or `E:message,detail` |

### Input Commands (Host → Device)

| Command | Description | Example |
|---------|-------------|---------|
| `M:left,right` | Set motor speeds (-255 to 255) | `M:100,-100` |
| `STOP` | Stop motors | `STOP` |
| `ESTOP` | Emergency stop (disable motors) | `ESTOP` |
| `ENABLE` | Re-enable motors after ESTOP | `ENABLE` |
| `HB` | Heartbeat (reset timeout) | `HB` |
| `STREAM:ON/OFF` | Enable/disable telemetry | `STREAM:ON` |
| `FMT:RAW/IMU` | Set output format | `FMT:RAW` |
| `CAL:G/A/ALL` | Calibrate sensors | `CAL:ALL` |
| `BETA:value` | Set filter beta | `BETA:0.1` |
| `FAST:duration` | Fast convergence mode | `FAST:2000` |
| `INIT` | Re-initialize filter | `INIT` |
| `WS:ssid,pass,ip[,port]` | Connect WebSocket | `WS:MyWiFi,password,192.168.1.100` |
| `WS:OFF` | Disconnect WebSocket | `WS:OFF` |
| `STATUS` | Get system status | `STATUS` |
| `REBOOT` | Reboot device | `REBOOT` |

## Python Receiver Example

```python
import serial

def parse_message(line):
    """Parse a message from the rover."""
    line = line.strip()
    if ':' not in line:
        return None, line
    
    prefix, data = line.split(':', 1)
    return prefix, data

def handle_imu(data):
    """Handle IMU telemetry."""
    values = data.split(',')
    yaw, pitch, roll, temp = float(values[0]), float(values[1]), float(values[2]), float(values[3])
    print(f"Orientation: Y={yaw:.1f} P={pitch:.1f} R={roll:.1f} T={temp:.1f}°C")

def handle_power(data):
    """Handle power telemetry."""
    values = data.split(',')
    voltage, current, power = float(values[0]), float(values[1]), float(values[2])
    print(f"Power: {voltage:.2f}V {current:.0f}mA {power:.0f}mW")

# Message handlers
handlers = {
    'I': handle_imu,
    'P': handle_power,
    'S': lambda d: print(f"[SYS] {d}"),
    'A': lambda d: print(f"[ACK] {d}"),
    'E': lambda d: print(f"[ERR] {d}"),
    'Raw': lambda d: None,  # Ignore MotionCal data
    'Ori': lambda d: None,
}

# Main loop
ser = serial.Serial('/dev/ttyUSB0', 115200)
while True:
    line = ser.readline().decode('utf-8')
    prefix, data = parse_message(line)
    
    if prefix in handlers:
        handlers[prefix](data)
    else:
        print(f"Unknown: {line}")
```

## Calibration

### Magnetometer Calibration with MotionCal

1. Set format to RAW: `FMT:RAW`
2. Open MotionCal application
3. Rotate the rover in all orientations to collect data
4. Copy calibration values to `hardcoded_calibration.h`
5. Rebuild and flash

### IMU Calibration

1. Place rover on a **level** surface
2. Keep **stationary** during calibration
3. Send `CAL:ALL` command
4. Wait for `A:CAL,all` acknowledgment

## Building

```bash
# Install PlatformIO
pip install platformio

# Build
pio run

# Upload
pio run -t upload

# Monitor
pio device monitor
```

## FreeRTOS Tasks

| Task | Priority | Core | Description |
|------|----------|------|-------------|
| IMU | 5 (High) | 1 | Sensor reading and filter update at 100Hz |
| Cmd | 4 | 0 | Command parsing and WebSocket events |
| Telem | 3 | 0 | Telemetry output at configured rate |
| Power | 2 | 0 | Power monitoring and motor timeout |
| WDT | 1 (Low) | 0 | Watchdog feeding |

## Safety Features

- **Motor timeout**: Motors stop if no command received for 3 seconds
- **Watchdog**: System resets if tasks don't respond for 10 seconds
- **Emergency stop**: `ESTOP` command disables motors until `ENABLE`

## License

MIT License
