# MotionCal Python Port - Implementation Plan

## Overview

Rewrite the MotionCal C++ sensor calibration application in Python, preserving all functionality without optimization or alteration. The application calibrates IMU sensors (magnetometer, accelerometer, gyroscope) using NXP/Freescale algorithms.

## Requirements Summary

From [CLAUDE.md](CLAUDE.md):
- ✅ Save in `Python/` directory
- ✅ Use PyQt6 for UI/graphics
- ✅ Include full NXP Kalman filter sensor fusion
- ✅ Support ASCII protocol only (Raw:, Cal1:, Cal2:)
- ✅ Use setuptools with setup.py
- ✅ Self-contained and installable
- ✅ Preserve ALL functionality - no additions, deletions, or optimizations
- ✅ Maintain exact data format compatibility
- ✅ Write plan and task tracking
- ✅ Write release notes with troubleshooting

## Critical C++ Reference Files

1. **imuread.h** - All data structures, constants, function signatures
2. **magcal.c** (620 lines) - Core calibration algorithms (4/7/10 element)
3. **gui.cpp** (1232 lines) - wxWidgets UI, application flow
4. **serialdata.parsing.c** (373 lines) - ASCII protocol parser
5. **rawdata.c** (353 lines) - Data orchestration, buffer management
6. **quality.c** (244 lines) - Quality metrics
7. **fusion.c** (2000+ lines) - NXP Kalman filter
8. **mahony.c** (289 lines) - Mahony AHRS
9. **visualize.c** (252 lines) - OpenGL 3D rendering

## Directory Structure

```
Python/
├── setup.py                          # setuptools installation
├── README.md                         # User documentation
├── IMPLEMENTATION_PLAN.md            # Task tracking (this file)
├── RELEASE_NOTES.md                  # Release notes + troubleshooting
├── requirements.txt                  # Dependencies
├── motioncal/                        # Main package
│   ├── __init__.py
│   ├── __main__.py                   # Entry point
│   ├── gui/                          # PyQt GUI
│   │   ├── __init__.py
│   │   ├── main_window.py            # Main window (ports gui.cpp)
│   │   ├── gl_canvas.py              # OpenGL widget
│   │   ├── connection_panel.py       # Port/baud/line ending controls
│   │   ├── calibration_panel.py      # Results display
│   │   ├── actions_panel.py          # Pause/Clear/Send buttons
│   │   ├── data_panel.py             # Raw data grids
│   │   └── resources.py              # Images (checkmarks)
│   ├── calibration/                  # Calibration algorithms
│   │   ├── __init__.py
│   │   ├── magcal.py                 # Ports magcal.c
│   │   ├── quality.py                # Ports quality.c
│   │   ├── matrix.py                 # Ports matrix.c
│   │   └── data_structures.py        # MagCalibration, Point, etc.
│   ├── fusion/                       # Sensor fusion
│   │   ├── __init__.py
│   │   ├── mahony.py                 # Ports mahony.c
│   │   ├── nxp_fusion.py             # Ports fusion.c
│   │   └── base.py                   # Common interfaces
│   ├── serial/                       # Serial communication
│   │   ├── __init__.py
│   │   ├── port_manager.py           # Port enumeration
│   │   ├── protocol.py               # ASCII parser (ports serialdata.parsing.c)
│   │   └── calibration_sender.py     # Binary 68-byte packet
│   ├── data/                         # Data handling
│   │   ├── __init__.py
│   │   ├── raw_data.py               # Ports rawdata.c
│   │   ├── apply_calibration.py      # Apply cal to mag
│   │   └── sensor_data.py            # Data classes
│   ├── visualization/                # 3D rendering
│   │   ├── __init__.py
│   │   ├── sphere_renderer.py        # Ports visualize.c
│   │   └── transforms.py             # Quaternion math
│   └── utils/                        # Utilities
│       ├── __init__.py
│       ├── constants.py              # All constants from imuread.h
│       ├── logging.py                # Logging
│       └── crc.py                    # CRC16
└── tests/                            # Unit tests
    ├── __init__.py
    ├── test_magcal.py
    ├── test_quality.py
    ├── test_protocol.py
    ├── test_matrix.py
    └── test_fusion.py
```

## Dependencies (requirements.txt)

```
PyQt6>=6.4.0                    # GUI framework
PyOpenGL>=3.1.6                 # OpenGL bindings
PyOpenGL-accelerate>=3.1.6      # OpenGL performance
numpy>=1.21.0                   # Numerical operations
scipy>=1.7.0                    # Eigenvalue decomposition
pyserial>=3.5                   # Serial communication
```

Minimum Python: 3.8 (for dataclasses, type hints, f-strings)

## Implementation Phases

### Phase 1: Foundation (Days 1-7)

**Goal**: Core data structures and utilities

#### Day 1-2: Setup
- [ ] Create `Python/` directory structure
- [ ] Write `setup.py` with entry points
- [ ] Write `requirements.txt`
- [ ] Create `motioncal/utils/constants.py` (port from imuread.h):
  - `MAGBUFFSIZE = 650`
  - `OVERSAMPLE_RATIO = 4`
  - `SENSORFS = 100`
  - `G_PER_COUNT = 0.0001220703125` (1/8192)
  - `UT_PER_COUNT = 0.1`
  - `DEG_PER_SEC_PER_COUNT = 0.0625` (1/16)
  - Quality thresholds: GAPS<15%, VARIANCE<4.5%, WOBBLE<4%, FIT<5%
- [ ] Create `motioncal/data/sensor_data.py`:
  - `Point3D` dataclass (x, y, z)
  - `ImuData` dataclass (accelerometer, gyroscope, magnetometer)
  - `Quaternion` dataclass (q0, q1, q2, q3)
  - `YawPitchRoll` dataclass
  - `OffsetsCalibrationData`, `SoftIronCalibrationData`
- [ ] Create `motioncal/calibration/data_structures.py`:
  - `MagCalibration` class with V[3], invW[3][3], B, FitError, BpFast[3][650], valid[650], etc.

#### Day 3-4: Matrix Operations
- [ ] Port `motioncal/calibration/matrix.py` using NumPy:
  - `matrix_identity_3x3()` → `np.eye(3)`
  - `matrix_set_scalar_3x3(A, s)` → `A.fill(s)`
  - `matrix_determinant_3x3(A)` → `np.linalg.det(A)`
  - `matrix_inverse_symmetric_3x3(A, B)` → `A = np.linalg.inv(B)`
  - `matrix_inverse_general(A, ...)` → `np.linalg.inv(A)`
  - `eigencompute(A, n)` → `np.linalg.eig(A[:n, :n])`
- [ ] Write unit tests for each function with known test vectors
- [ ] **Critical**: Use `numpy.float32` to match C `float` precision

#### Day 5-7: Serial Communication
- [ ] Port `motioncal/serial/protocol.py` (ASCII parser from serialdata.parsing.c):
  - State machine: STATE_WORD, STATE_RAW, STATE_CAL1, STATE_CAL2
  - `parse_raw_line()` - Extract 9 int16 from "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r"
  - `parse_cal1_line()` - Extract 10 floats from "Cal1:..."
  - `parse_cal2_line()` - Extract 9 floats from "Cal2:..."
  - Handle line endings: LF, CR, CRLF
  - Callbacks: `on_raw_data()`, `on_cal1_data()`, `on_cal2_data()`
- [ ] Port `motioncal/serial/port_manager.py`:
  - `enumerate_ports()` using `serial.tools.list_ports`
  - `open_port(name, baud, line_ending)`
  - `read_serial_data()`
  - `write_serial_data(bytes)`
- [ ] Port `motioncal/serial/calibration_sender.py` (from rawdata.c send_calibration):
  - Build 68-byte packet: signature(117,84) + accel(3×float, zeros) + gyro(3×float, zeros) + mag_offset(3×float) + field(float) + soft_iron(6×float) + CRC16
  - `crc16(data)` in `motioncal/utils/crc.py` - polynomial 0xA001
- [ ] Test with recorded serial data

### Phase 2: Calibration Algorithms (Days 8-14)

**Goal**: Core calibration working

#### Day 8-10: Magnetic Calibration
- [ ] Port `motioncal/calibration/magcal.py` from magcal.c:
  - `update_calibration_4inv()` - 4 element matrix inversion (≥40 points)
    - Assumes identity soft iron
    - Solves for hard iron offset only
    - Minimum fit error clamped to 12%
  - `update_calibration_7eig()` - 7 element eigendecomposition (≥100 points)
    - Diagonal soft iron matrix
    - Minimum fit error clamped to 7.5%
  - `update_calibration_10eig()` - 10 element eigendecomposition (≥150 points)
    - Full symmetric soft iron matrix
    - No minimum fit error clamp
  - `magcal_run()` - Main orchestration:
    - Rate limit: run every 20 iterations
    - Age existing fit error: `FitErrorAge *= 1.02`
    - Choose algorithm based on count
    - Validate geomagnetic field: 22-67 µT
    - Accept if: no prev cal OR fit improves OR better algorithm with fit≤4%
- [ ] **Critical**: Preserve EXACT math - DEFAULTB=50.0, FMATRIXSCALING, all thresholds
- [ ] Test each algorithm with captured C++ intermediate values

#### Day 11-12: Quality Metrics
- [ ] Port `motioncal/calibration/quality.py` from quality.c:
  - `quality_reset()` - Initialize 100 sphere regions
  - `quality_surface_gap_error()` - Coverage metric (0-100%)
    - Arctic cap (1 region): lat > 78.52°
    - N temperate (15): 42.84° < lat ≤ 78.52°
    - N tropic (34): 0° < lat ≤ 42.84°
    - S tropic (34): -42.84° ≤ lat < 0°
    - S temperate (15): -78.52° ≤ lat < -42.84°
    - Antarctic cap (1): lat < -78.52°
  - `quality_magnitude_variance_error()` - Field uniformity
  - `quality_wobble_error()` - Sphericity metric
  - `quality_spherical_fit_error()` - Returns magcal.FitError
- [ ] Test with known point distributions

#### Day 13-14: Data Processing
- [ ] Port `motioncal/data/raw_data.py` from rawdata.c:
  - `raw_data_reset()` - Initialize to V=[0,0,80], invW=identity, B=50, FitError=100
  - `raw_data(int16[9])` - Process incoming IMU:
    - Add to mag buffer via `add_magcal_data()`
    - Run `magcal_run()` periodically
    - Accumulate accel/gyro/mag for oversampling
    - Call `fusion_update()` when OVERSAMPLE_RATIO reached
  - `choose_discard_magcal()` - Smart buffer management:
    - If gaps < 25%: discard point farthest from expected field
    - Else: find 2 closest points, randomly discard one
  - `cal1_data()`, `cal2_data()` - Verify calibration echo
- [ ] Port `motioncal/data/apply_calibration.py` from visualize.c:
  - Convert raw counts to µT, subtract hard iron, apply soft iron matrix
- [ ] Test end-to-end data flow

### Phase 3: Sensor Fusion (Days 15-21)

**Goal**: Orientation working

#### Day 15-17: Mahony AHRS
- [ ] Port `motioncal/fusion/mahony.py` from mahony.c:
  - Constants: `twoKp = 0.04`, `twoKi = 0.0`, `INV_SAMPLE_RATE = 0.01`
  - State: quaternion (q0,q1,q2,q3), integral feedback (0,0,0)
  - `mahony_init()` - Reset to identity quaternion (1,0,0,0)
  - `mahony_update(gx,gy,gz,ax,ay,az,mx,my,mz)` - 9-DOF update
  - `inv_sqrt(x)` - Fast inverse square root (bit hack or `1.0/sqrt(x)`)
- [ ] Test with synthetic IMU data

#### Day 18-21: NXP Kalman Filter
- [ ] Port `motioncal/fusion/nxp_fusion.py` from fusion.c (COMPLEX - 2000 lines):
  - 12-element state: orientation error (3) + gyro bias (3) + accel error (3) + mag disturbance (3)
  - Kalman components: P (12×12 covariance), Q (process noise), C (measurement matrix), K (Kalman gain)
  - `fusion_init()` - Initialize Kalman filter
  - `fusion_update()` - Prediction + correction steps
  - 70+ helper functions for quaternion/rotation math
- [ ] **Strategy**: Port incrementally, validate each step
- [ ] Consider using `scipy.linalg` for Kalman operations
- [ ] Test with recorded sensor sequences

### Phase 4: Visualization (Days 22-26)

**Goal**: 3D rendering working

#### Day 22-24: OpenGL Renderer
- [ ] Port `motioncal/visualization/sphere_renderer.py` from visualize.c:
  - `visualize_init()` - Setup OpenGL, lighting, sphere display lists
  - `display_callback()` - Render frame:
    - Draw 650 calibrated mag points as spheres
    - Color: green if CanSave, red otherwise
    - Rotate by current quaternion
    - Draw sensor as yellow/blue triangle
  - `resize_callback(w, h)` - Handle resize
- [ ] Use PyOpenGL display lists for performance
- [ ] Test sphere rendering with sample data

#### Day 25-26: Transforms
- [ ] Port `motioncal/visualization/transforms.py`:
  - `quaternion_to_rotation_matrix(q)` - Convert to 3×3 matrix
  - `rotate_point(point, R)` - Apply rotation
  - `quaternion_normalize(q)` - Normalize
- [ ] Validate quaternion math

### Phase 5: GUI (Days 27-33)

**Goal**: Complete application

#### Day 27-28: Main Window & Connection Panel
- [ ] Port `motioncal/gui/main_window.py` from gui.cpp:
  - `MainWindow(QMainWindow)` with horizontal layout
  - Left panel (processing) + Right panel (calibration)
  - Menu: File (Send Cal, Quit), Port, Baud, Help (About)
  - QTimer at 14ms interval for serial reading
  - State: `_paused`, last values for deduplication
- [ ] Port `motioncal/gui/connection_panel.py`:
  - Port dropdown (QComboBox) from `enumerate_ports()`
  - Baud dropdown: 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400
  - Line ending dropdown: (none), LF, CR, CRLF
  - Auto-connect on selection change

#### Day 29-30: Actions & Data Panels
- [ ] Port `motioncal/gui/actions_panel.py`:
  - Status icon (QLabel + QPixmap):
    - Gray empty: no cal
    - White empty: CanSave
    - Green filled: confirmed
  - Pause button: toggles "Pause" ↔ "Capture"
  - Clear button: calls `raw_data_reset()`
  - Send Cal button: enabled when CanSave, calls `send_calibration()`
- [ ] Port `motioncal/gui/data_panel.py`:
  - Raw data QTableWidget (3×3): Accel, Gyro, Mag
  - Orientation QTableWidget (1×3): Yaw, Pitch, Roll
  - Right-aligned float formatting

#### Day 31-32: Calibration Panel & GL Canvas
- [ ] Port `motioncal/gui/calibration_panel.py`:
  - Magnetic Offset: V[0], V[1], V[2] (format: "%.2f")
  - Magnetic Mapping: 3×3 grid invW[i][j] (format: "%+.3f")
  - Magnetic Field: B (format: "%.2f")
  - Accelerometer: 0.000, 0.000, 0.000 (always zero in ASCII mode)
  - Gyroscope: 0.000, 0.000, 0.000 (always zero in ASCII mode)
- [ ] Port `motioncal/gui/gl_canvas.py`:
  - `GLCanvas(QOpenGLWidget)`
  - `initializeGL()` → `visualize_init()`
  - `paintGL()` → `display_callback()`
  - `resizeGL(w,h)` → `resize_callback(w,h)`
  - Size: 480×480 minimum

#### Day 33: Integration
- [ ] Port `motioncal/gui/resources.py`:
  - Load checkmark PNGs (gray empty, white empty, green filled)
- [ ] Connect all callbacks
- [ ] Test full UI flow

### Phase 6: Testing & Documentation (Days 34-40)

**Goal**: Production-ready release

#### Day 34-36: Testing
- [ ] Unit tests for all modules:
  - `tests/test_magcal.py` - Test 4/7/10 element calibration
  - `tests/test_quality.py` - Test sphere region calculation
  - `tests/test_protocol.py` - Test ASCII parsing
  - `tests/test_matrix.py` - Test matrix operations
  - `tests/test_fusion.py` - Test sensor fusion
- [ ] Integration tests with recorded data
- [ ] Hardware loop test with real IMU
- [ ] Regression tests: compare Python vs C++ output

#### Day 37-38: Documentation
- [ ] Write `README.md`:
  - Installation: `pip install .`
  - Usage: `motioncal`
  - Supported hardware
  - Calibration process
- [ ] Write `RELEASE_NOTES.md`:
  - Port information
  - Differences from C++ (removed binary protocol)
  - Troubleshooting:
    - Serial port not listed → dialout group (Linux), drivers (Win/Mac)
    - Calibration not converging → slow rotation, full sphere, avoid interference
    - OpenGL errors → Mesa install, driver update
    - CRC mismatch → line ending, baud rate
  - Validation traces with test case results

#### Day 39-40: Packaging
- [ ] Finalize `setup.py`:
  - Package metadata
  - Entry points: `motioncal` (console), `motioncal-gui` (GUI on Windows)
  - Package data: resources/*.png
- [ ] Write `motioncal/__main__.py`:
  ```python
  def main():
      from PyQt6.QtWidgets import QApplication
      from motioncal.gui.main_window import MainWindow
      app = QApplication(sys.argv)
      window = MainWindow()
      window.show()
      sys.exit(app.exec())
  ```
- [ ] Test installation on Linux/Windows/macOS

## Critical Preservation Points

### Exact Math Preservation
- Use `numpy.float32` to match C `float`
- Preserve ALL constants: DEFAULTB=50.0, thresholds, scaling factors
- NO optimizations to calibration algorithms
- Validate intermediate values match C++ within 1e-6 tolerance

### Data Format Compatibility
- ASCII input: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r"
- Binary output: 68 bytes (signature + offsets + CRC16)
- Scale factors: accel(1/8192), gyro(1/16), mag(0.1)

### Algorithm Validation
- Test 4-element cal with 40 points
- Test 7-element cal with 100 points
- Test 10-element cal with 650 points
- Compare V, invW, B, FitError to C++ within 0.01%

## Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| NXP fusion complexity (2000 lines) | Port incrementally, use scipy.linalg, validate each step |
| NumPy float precision differs from C | Use `np.float32`, test with tolerance 1e-5 to 1e-6 |
| PyQt OpenGL integration | Test early on all platforms, have matplotlib 3D fallback |
| Python serial timing slower | Use native pyserial with adequate buffer, profile if needed |
| Cross-platform differences | Test on Linux/Windows/macOS early, use GitHub Actions CI |

## Success Criteria

- ✅ All 4/7/10 element calibration algorithms produce identical results to C++
- ✅ Quality metrics match C++ exactly
- ✅ 3D visualization displays correctly rotated sphere
- ✅ Serial communication works at 115200 baud
- ✅ Binary calibration packet verified by ESP32 device
- ✅ GUI layout matches original wxWidgets application
- ✅ Installable via `pip install .`
- ✅ No new functionality, no deleted functionality, no optimizations

## Estimated Timeline

**Total: 40 days (8 weeks)**

- Phase 1 (Foundation): 7 days
- Phase 2 (Calibration): 7 days
- Phase 3 (Fusion): 7 days
- Phase 4 (Visualization): 5 days
- Phase 5 (GUI): 7 days
- Phase 6 (Testing & Docs): 7 days

## Next Steps

1. Create `Python/` directory structure
2. Write `setup.py` and `requirements.txt`
3. Begin Phase 1: Port constants and data structures
4. Maintain this plan file - check off completed tasks
5. Update RELEASE_NOTES.md with discoveries during implementation
