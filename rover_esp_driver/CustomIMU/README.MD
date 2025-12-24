# CustomIMU Driver - QMI8658C + AK09918C

A custom IMU driver for the Waveshare rover, providing 9-DOF sensor fusion with MotionCal compatibility for magnetometer calibration.

## Hardware

- **IMU**: QMI8658C (6-DOF: Accelerometer + Gyroscope)
- **Magnetometer**: AK09918C (3-DOF)
- **I2C Addresses**: QMI8658C @ 0x6B, AK09918C @ 0x0C
- **I2C Pins**: SDA=32, SCL=33

## What This Driver Does

1. Reads accelerometer, gyroscope, and magnetometer data
2. Outputs data in PJRC MotionCal format for magnetometer calibration
3. Uses Madgwick filter for sensor fusion and orientation estimation
4. Supports receiving calibration data from MotionCal and saving to NVS (flash)
5. Provides yaw/pitch/roll orientation output


---

## Serial Output Formats

### 1. Raw Data (for MotionCal)
```
Raw:ax,ay,az,gx,gy,gz,mx,my,mz
```
- Accel: scaled by 8192 LSB/g
- Gyro: scaled by 16 LSB/dps
- Mag: scaled by 10 LSB/uT
- **Magnetometer data is RAW (uncalibrated)** so MotionCal can compute calibration

### 2. Unified Data (for debugging)
```
Uni:ax,ay,az,gx,gy,gz,mx,my,mz
```
- Accel: m/s²
- Gyro: rad/s
- Mag: uT (calibrated if calibration exists)
- **Currently commented out in main.cpp**

### 3. Orientation (from Madgwick filter)
```
Ori: yaw,pitch,roll
```
- All values in degrees
- Uses calibrated magnetometer for heading

---

## Serial Commands

| Command | Description |
|---------|-------------|
| `RECAL` | Re-calibrate gyroscope (keep device still!) |
| `RESET` | Reset Madgwick filter orientation |
| `CALINFO` | Print current magnetometer calibration |
| `LOADCAL` | Reload calibration from NVS |

---

## Magnetometer Calibration with MotionCal

### Setup
1. Download MotionCal from PJRC: https://www.pjrc.com/store/prop_shield.html
2. Connect ESP32 via USB
3. Open MotionCal, select serial port at 115200 baud

### Calibration Procedure
1. Start MotionCal - it will receive "Raw:" data automatically
2. Rotate the device slowly in ALL orientations:
   - Roll it like a rolling pin
   - Pitch it up and down
   - Yaw (rotate) it around
   - Combine movements (figure-8 patterns work well)
3. Goal: Fill in the sphere visualization with data points
4. **Target**: 80%+ coverage, <3% fit error
5. Click "Send Cal" in MotionCal to transmit calibration to device

### What Gets Calibrated
- **Hard Iron**: Constant magnetic offset (from nearby magnets, PCB traces)
- **Soft Iron**: Magnetic field distortion (from nearby ferromagnetic materials)

### Calibration Storage

**Two storage mechanisms exist:**

1. **NVS (Non-Volatile Storage)**
   - Saved automatically when MotionCal sends calibration
   - Persists across reboots
   - Stored in ESP32 flash under namespace "magcal"
   - Loaded automatically at startup

2. **Hardcoded Calibration** (`include/hardcoded_calibration.h`)
   - Backup calibration compiled into firmware
   - Applied at startup BEFORE NVS is checked
   - If NVS has valid calibration, it overrides hardcoded values
   - Set `USE_HARDCODED_CALIBRATION` to `false` to disable

### Current Hardcoded Values
```cpp
Hard Iron: X=25.99, Y=-56.78, Z=-27.08 uT
Field Strength: 56.42 uT
Fit Error: 1.4%, Gaps: 5.1%
```

---

## Important Functions to Comment/Uncomment

### In main.cpp

**Unified output** (lines 234-238):
```cpp
// motioncal_send_unified(...)
```
Currently commented out. Uncomment to see calibrated sensor values in physical units.

### In hardcoded_calibration.h

**Disable hardcoded calibration** (line 38):
```cpp
#define USE_HARDCODED_CALIBRATION true  // Change to false to test without
```

### In config.h

**Madgwick beta** (line 59):
```cpp
#define MADGWICK_BETA 0.05f
```
- Higher (0.1-0.2): Faster response, more noise, faster drift correction
- Lower (0.01-0.03): Smoother, slower response, slower drift correction

---

## Known Issues & Tips

### Remaining Drift (~2°/min)
This is normal for MEMS gyroscopes. Causes:
- Temperature-related gyro bias drift
- Residual calibration errors

### To Minimize Drift
1. **Keep device still during startup** - on-demand calibration runs for ~2.5s
2. **Let device warm up** - run 5-10 minutes before expecting stable readings
3. **Re-run gyro calibration after warmup** - send `RECAL` command
4. **Improve magnetometer calibration** - aim for 80%+ coverage

### Yaw vs Pitch/Roll Drift
- **Yaw drift**: Usually magnetometer calibration issue
- **Pitch/Roll drift**: Usually gyroscope/accelerometer bias issue

---

## File Structure

```
CustomIMU/
├── include/
│   ├── config.h                  # Pin definitions, sensor config, filter params
│   ├── qmi8658c.h               # QMI8658C register definitions and class
│   ├── ak09918c.h               # AK09918C register definitions and class
│   ├── madgwick.h               # Madgwick AHRS filter
│   ├── motioncal_output.h       # MotionCal data formatting
│   ├── mag_calibration.h        # Calibration receiver and NVS storage
│   ├── hardcoded_calibration.h  # Backup calibration values
│   └── i2c_helpers.h            # I2C read/write utilities
├── src/
│   ├── main.cpp                 # Main application loop
│   ├── qmi8658c.cpp             # IMU driver implementation
│   ├── ak09918c.cpp             # Magnetometer driver implementation
│   ├── madgwick.cpp             # Madgwick filter implementation
│   ├── motioncal_output.cpp     # Serial output formatting
│   ├── mag_calibration.cpp      # Calibration parsing and NVS storage
│   └── i2c_helpers.cpp          # I2C implementation
└── README.MD                    # This file
```

---

## Reference: General_Driver Location

The original working driver that this was compared against:
```
ugv_base_general-main/General_Driver/
├── QMI8658.cpp    # Reference IMU implementation
├── QMI8658.h      # Reference IMU header
├── QMI8658reg.h   # Complete register definitions
├── AK09918.cpp    # Reference magnetometer
└── AK09918.h      # Reference magnetometer header
```

Key functions to reference:
- `qmi8658_on_demand_cali()` - The critical calibration routine
- `config_acc()` / `config_gyro()` - Sensor configuration
- `autoOffsets()` - Runtime bias calibration

---

## Build Notes

Platform: ESP32 (PlatformIO)
Framework: Arduino
Baud Rate: 115200
