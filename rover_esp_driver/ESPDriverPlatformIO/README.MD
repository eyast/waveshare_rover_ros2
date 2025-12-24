# Waveshare Rover ESP32 Driver

ESP32-based driver for the Waveshare Rover platform with IMU sensor fusion, motor control, and OLED display.

## I2C Devices

| Address | Device | Description |
|---------|--------|-------------|
| 0x0C | AK09918C | Magnetometer |
| 0x3C | SSD1306 | OLED Display |
| 0x6B | QMI8658C | IMU (Accel + Gyro) |
| 0x42 | INA219 | Voltage/Current Sensor |

## JSON Command Protocol

Commands are sent over serial as JSON objects with a `"T"` field indicating the command type. All commands must end with a newline character.

### Motor Control

#### Speed Control (T: 1)
Set target wheel speeds using PID control.

```json
{"T": 1, "L": <float>, "R": <float>}
```
- `L`: Left wheel speed
- `R`: Right wheel speed

#### PWM Direct Control (T: 11)
Bypass PID and set PWM values directly.

```json
{"T": 11, "L": <int>, "R": <int>}
```
- `L`: Left motor PWM (-255 to 255)
- `R`: Right motor PWM (-255 to 255)

#### Set Motor PID (T: 2)
Configure PID controller parameters.

```json
{"T": 2, "P": <float>, "I": <float>, "D": <float>, "L": <float>}
```
- `P`: Proportional gain
- `I`: Integral gain
- `D`: Derivative gain
- `L`: Windup limit

### OLED Display

#### Set OLED Line (T: 3)
Write text to a specific line on the OLED.

```json
{"T": 3, "lineNum": <int>, "Text": "<string>"}
```
- `lineNum`: Line number (0-3)
- `Text`: Text to display

#### Reset OLED to Default (T: -3)
Reset OLED to default display mode.

```json
{"T": -3}
```

### IMU Streaming

#### Toggle IMU Stream (T: 325)
Enable or disable IMU data streaming.

```json
{"T": 325, "cmd": <int>}
```
- `cmd`: 1 = enable, 0 = disable

#### Set Stream Format (T: 400)
Switch between JSON and MotionCal output formats.

```json
{"T": 400, "cmd": <int>}
```
- `cmd`: 1 = JSON format, 0 = MotionCal format

### IMU Stream Output (T: 326)
When streaming is enabled and JSON format is selected, the robot sends:

```json
{
  "T": 326,
  "ax": <float>, "ay": <float>, "az": <float>,
  "gx": <float>, "gy": <float>, "gz": <float>,
  "mx": <float>, "my": <float>, "mz": <float>,
  "pitch": <float>, "roll": <float>, "yaw": <float>,
  "dt": <float>
}
```
- `ax/ay/az`: Accelerometer (g)
- `gx/gy/gz`: Gyroscope (rad/s)
- `mx/my/mz`: Magnetometer (uT, calibrated)
- `pitch/roll/yaw`: Orientation (degrees)
- `dt`: Delta time since last update (seconds)

### System Commands

#### Reboot (T: 600)
Restart the ESP32.

```json
{"T": 600}
```

#### Set Platform Type (T: 900)
Configure the platform and module type.

```json
{"T": 900, "main": <int>, "module": <int>}
```
- `main`: 1 = WAVE ROVER, 2 = UGV02 (UGV Rover), 3 = UGV01 (UGV Beast)
- `module`: 0 = Base, 1 = RoArm-M2, 2 = Gimbal

## Configuration Variables

### IMU Configuration (include/config.h)

| Variable | Default | Description |
|----------|---------|-------------|
| `I2C_SDA` | 32 | I2C data pin |
| `I2C_SCL` | 33 | I2C clock pin |
| `I2C_FREQ` | 400000 | I2C frequency (Hz) |
| `QMI8658C_ADDR` | 0x6B | IMU I2C address |
| `AK09918C_ADDR` | 0x0C | Magnetometer I2C address |
| `ACCEL_FS_SEL` | 2 | Accelerometer full scale (0=2g, 1=4g, 2=8g, 3=16g) |
| `GYRO_FS_SEL` | 2 | Gyroscope full scale (0=16dps, 1=32dps, 2=64dps, etc.) |
| `ODR_SEL` | 6 | Output data rate selection |
| `MADGWICK_BETA` | 0.1f | Madgwick filter gain (higher = faster convergence, more noise) |
| `OUTPUT_INTERVAL_MS` | 20 | IMU stream interval (ms) - 50Hz default |
| `stream_as_json` | false | Output format: true = JSON, false = MotionCal |
| `stream_orientiation` | true | Include orientation in MotionCal output |

### Platform Configuration (include/ugv_config.h)

| Variable | Default | Description |
|----------|---------|-------------|
| `InfoPrint` | 1 | Debug output: 0 = none, 1 = debug info, 2 = flow feedback |
| `mainType` | 1 | Platform: 1 = WAVE ROVER, 2 = UGV02, 3 = UGV01 |
| `moduleType` | 0 | Module: 0 = Base, 1 = RoArm-M2, 2 = Gimbal |
| `WHEEL_D` | 0.0800 | Wheel diameter (meters) |
| `ONE_CIRCLE_PLUSES` | 2100 | Encoder pulses per wheel revolution |
| `TRACK_WIDTH` | 0.125 | Distance between wheels (meters) |
| `SET_MOTOR_DIR` | false | Reverse motor direction |
| `HEART_BEAT_DELAY` | 3000 | Safety timeout (ms) - motors stop if no command received |

### Motor PID Configuration (include/ugv_config.h)

| Variable | Default | Description |
|----------|---------|-------------|
| `__kp` | 20.0 | Proportional gain |
| `__ki` | 2000.0 | Integral gain |
| `__kd` | 0 | Derivative gain |
| `windup_limits` | 255 | Anti-windup limit |

### Motor Pin Configuration (include/ugv_config.h)

| Pin | GPIO | Description |
|-----|------|-------------|
| `PWMA` | 25 | Motor A PWM |
| `AIN1` | 21 | Motor A input 1 |
| `AIN2` | 17 | Motor A input 2 |
| `PWMB` | 26 | Motor B PWM |
| `BIN1` | 22 | Motor B input 1 |
| `BIN2` | 23 | Motor B input 2 |
| `AENCA` | 35 | Encoder A channel A |
| `AENCB` | 34 | Encoder A channel B |
| `BENCA` | 27 | Encoder B channel A |
| `BENCB` | 16 | Encoder B channel B |

### UART Configuration (include/uart_ctrl.h)

| Variable | Default | Description |
|----------|---------|-------------|
| `imu_stream_enabled` | true | IMU streaming enabled at startup |

### Serial Configuration (include/config.h)

| Variable | Default | Description |
|----------|---------|-------------|
| `SERIAL_BAUD` | 115200 | Serial baud rate |

## Magnetometer Calibration

Hardcoded calibration values are stored in `include/hardcoded_calibration.h`. To recalibrate:

1. Set `stream_as_json` to `false` in config.h (or send `{"T": 400, "cmd": 0}`)
2. Enable streaming with `{"T": 325, "cmd": 1}`
3. Use MotionCal application to collect calibration data
4. Update the calibration values in `hardcoded_calibration.h`
5. Rebuild and flash

## Safety Features

- **Heartbeat Timeout**: Motors automatically stop if no command is received within `HEART_BEAT_DELAY` milliseconds (default 3 seconds)
- **PID Control**: Smooth velocity control with configurable gains
- **PWM Limits**: Maximum PWM bounded by `MAX_PWM` (255) and minimum by `MIN_PWM` (63)


## Todo:

- **Code validation**: Ensure there are not overflows, unused variables, etc
- **Debugging**: Collect all errors and store them in variable, that can be retrieved using a new T command
- **OLED display**: Implement an OLED library that can display larger text.
- **NVS integration**: Integrate NVS to store/retrieve calibration data.
- **Calibration JSON**: Create a JSON command that saves/loads calibration.
- **INA219**: Integrate INA219, retrieve Voltage, Current, last temperature from IMU.
- **Calibrate with PI**.
- **Calibrate with battery and motors on at minimal cycles**.
- **Create temperature curve for calibration**: Change IMU calibration based on the temperature of the device. 
- **Correct bias of accelerometer and gyroscope** - reduce the mean at runtime
- **Calibrate with Orientation**: Try to calibrate with Orientation.
- **Validation of JSON** to prevent malformed messages
- **Code Analysis**
- **Test motors**
- 