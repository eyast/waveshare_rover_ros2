# IMU Yaw Drift Fix — Complete Solution

## Root Cause

Your yaw drift was caused by **two issues**:

### 1. Magnetometer Z-Axis Inverted (PRIMARY CAUSE)
Your calibrated magnetometer showed a dip angle of **+73°**, but Melbourne, Australia should have **-65°** (negative = field pointing DOWN in the southern hemisphere).

This 138° error means the Madgwick filter was receiving an inverted magnetic reference. Instead of correcting yaw drift, the filter was actively making it worse — fighting against the true magnetic heading.

### 2. Gyro/Accel Bias Not Fully Removed
The logged gyro data showed -7°/s, +3°/s, -1°/s instead of ~0. The accelerometer also had small biases (~0.016g) that weren't being calibrated out.

---

## Files Changed

| File | Changes |
|------|---------|
| `ak09918c.h` | Added `set_axis_signs()` method |
| `ak09918c.cpp` | Applies axis sign corrections after soft iron |
| `qmi8658c.h` | Added `calibrate_accel()` declaration |
| `qmi8658c.cpp` | Added accelerometer bias calibration, improved gyro calibration |
| `hardcoded_calibration.h` | Added `MAG_AXIS_Z_SIGN = -1`, calls `set_axis_signs()` |
| `config.h` | Added new JSON command definitions (330-350 range) |
| `uart_ctrl.h` | Added JSON handlers for all new IMU commands |
| `imu_stream.h` | Contains `updateIMUFilter()` and `sendIMUStreamData()` with JSON support |
| `main.cpp` | Calls both calibrations at startup, verifies dip angle |
| `imu_viz_v5.py` | **NEW:** Python visualizer with control panel UI |

---

## JSON Commands Reference

All new commands use the same JSON format as your existing control library:

### Calibration Commands
| Command | JSON | Description |
|---------|------|-------------|
| Calibrate Gyro | `{"T":330}` | Calibrate gyroscope bias (keep still) |
| Calibrate Accel | `{"T":331}` | Calibrate accelerometer bias (keep still & level) |
| Calibrate All | `{"T":332}` | Calibrate both + reset filter |

### Filter Commands
| Command | JSON | Description |
|---------|------|-------------|
| Reset Filter | `{"T":335}` | Reset Madgwick quaternion |
| Set Beta | `{"T":336,"beta":0.1}` | Set filter gain (0.01-1.0) |

### Magnetometer Axis
| Command | JSON | Description |
|---------|------|-------------|
| Set Axis Signs | `{"T":340,"x":1,"y":1,"z":-1}` | Set axis sign corrections |

### Status/Debug
| Command | JSON | Description |
|---------|------|-------------|
| Debug Print | `{"T":345}` | Print human-readable debug info |
| Get Status | `{"T":346}` | Returns JSON with all sensor data + dip angle |
| Get Orientation | `{"T":350}` | Returns `{"T":350,"yaw":x,"pitch":y,"roll":z}` |

### Streaming Control (existing)
| Command | JSON | Description |
|---------|------|-------------|
| Toggle Stream | `{"T":325,"cmd":1}` | Enable (1) / disable (0) streaming |
| Stream Format | `{"T":400,"cmd":1}` | JSON (1) / MotionCal (0) format |

---

## Python Visualizer (imu_viz_v5.py)

The updated visualizer includes a **control panel** with buttons for all IMU commands:

### Features
- 3D orientation view (toggle with SPACE)
- Magnetometer sphere visualization
- Real-time sensor plots
- Dip angle display (should be ~-65° for Melbourne)
- **Control Panel** with:
  - Calibration buttons (Gyro, Accel, All)
  - Filter reset button
  - Beta slider (0.01 - 1.0)
  - Stream toggle (on/off)
  - Format toggle (JSON/MotionCal)
  - Mag axis sign toggles (X, Y, Z)
  - Status request button

### Usage
```bash
python imu_viz_v5.py --port=/dev/cu.usbserial-110 --baud=115200
```

### Controls
- **SPACE** — Toggle 3D view (Orientation / Mag Sphere)
- **S** — Save CSV data
- **ESC** — Exit

---

## Quick Start

### 1. Flash Firmware
Replace these files in your project:
- `ak09918c.h`, `ak09918c.cpp`
- `qmi8658c.h`, `qmi8658c.cpp`
- `hardcoded_calibration.h`
- `config.h`
- `uart_ctrl.h`
- `imu_stream.h`
- `main.cpp`

### 2. Run Visualizer
```bash
python imu_viz_v5.py --port=/dev/cu.usbserial-110
```

### 3. Verify Dip Angle
The status panel should show **Dip: -65°** (±5°) for Melbourne. If positive, the Z-axis fix isn't applied.

### 4. Calibrate
Click **"Calibrate ALL"** in the control panel (keep device still & level).

### 5. Test Stability
Leave stationary for 60 seconds. Yaw should be stable within ±1-2°.

---

## Troubleshooting

### Dip Angle Still Positive
Check that `MAG_AXIS_Z_SIGN = -1` in `hardcoded_calibration.h` and that `apply_hardcoded_calibration(mag)` is called in `setup()`.

### Yaw Still Drifting
Try adjusting beta via the slider or `{"T":336,"beta":0.2}`. Higher values = faster correction but more noise.

### Stream Not Working
Send `{"T":325,"cmd":1}` to enable streaming, or click the toggle in the visualizer.

---

## Expected Results

After fixes:
- **Yaw drift**: <2°/min (from -34°/min)
- **Pitch/Roll drift**: <0.1°/min
- **Magnetic dip angle**: -65° ±5° (from +73°)
