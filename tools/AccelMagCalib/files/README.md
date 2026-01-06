# Ellipsoid Calibration for Accelerometers and Magnetometers

Complete toolset for calibrating MEMS sensors using ellipsoid fitting with constrained least squares optimization.

## 📋 Table of Contents

1. [Quick Start](#quick-start)
2. [What This Does](#what-this-does)
3. [Files Included](#files-included)
4. [Complete Workflow](#complete-workflow)
5. [Theory and Mathematics](#theory-and-mathematics)
6. [Troubleshooting](#troubleshooting)
7. [FAQ](#faq)

---

## 🚀 Quick Start

### Python-Only (Desktop Testing)

```bash
# 1. Install dependencies
pip install numpy matplotlib

# 2. Run with example data (generates synthetic data)
python ellipsoid_calibration.py

# 3. Or with your own data file
python ellipsoid_calibration.py your_sensor_data.txt
```

### Full ESP32 Workflow

```bash
# 1. Upload data collection sketch to ESP32
#    (Use Arduino IDE or PlatformIO with esp32_calibration_data_collection.ino)

# 2. Collect data using helper script
python serial_data_collector.py
#    OR manually save Serial Monitor output

# 3. Run calibration
python ellipsoid_calibration.py sensor_data.txt

# 4. Copy calibration parameters into esp32_use_calibration.ino

# 5. Upload and enjoy calibrated sensor readings!
```

---

## 🎯 What This Does

### The Problem

Real-world accelerometers and magnetometers have errors:

| Error Type | Accelerometer | Magnetometer |
|------------|---------------|--------------|
| **Offset/Bias** | Zero-g offset (reads non-zero at rest) | Hard-iron effects (permanent magnets nearby) |
| **Scaling** | Different sensitivity per axis | Soft-iron effects (ferromagnetic materials) |
| **Non-orthogonality** | Axes not perfectly 90° apart | Field distortion |
| **Result** | Ellipsoid offset from origin | Ellipsoid offset from origin |

### The Solution

This implements the **constrained least-squares ellipsoid fitting** algorithm that:

1. **Fits an ellipsoid** to your 3D sensor measurements
2. **Extracts bias vector (B)**: The center of the ellipsoid → subtract this from raw readings
3. **Extracts calibration matrix (A⁻¹)**: Transforms ellipsoid back to sphere → multiply bias-corrected readings by this

**Mathematical formula:**
```
calibrated = A⁻¹ × (raw - B)
```

### What You Get

**Before calibration:**
- Accelerometer magnitude varies: 0.85g - 1.15g ❌
- Tilt angle errors: ±10-20° ❌
- Magnetometer heading errors: ±30° or worse ❌

**After calibration:**
- Accelerometer magnitude: 1.00g ± 0.01g ✅
- Tilt angle errors: <2° ✅
- Magnetometer heading errors: <5° ✅

---

## 📁 Files Included

### Core Calibration Tool
- **`ellipsoid_calibration.py`** - Main Python calibration program
  - Implements ellipsoid fitting algorithm
  - Visualizes raw vs calibrated data
  - Exports calibration parameters
  - Validates calibration quality

### Documentation
- **`DATA_COLLECTION_GUIDE.md`** - Comprehensive guide on:
  - Why calibration is necessary
  - How to collect data properly
  - Common mistakes to avoid
  - Validation procedures

### ESP32/Arduino Code
- **`esp32_calibration_data_collection.ino`** - Data collection sketch
  - Outputs sensor readings over Serial
  - Works with MPU6050, MPU9250, QMI8658C, AK09918C, etc.
  - Configurable sample rate and duration
  
- **`esp32_use_calibration.ino`** - Example of using calibration
  - Shows how to apply calibration in real-time
  - Includes validation functions
  - Demonstrates roll/pitch/heading calculations
  - Shows how to store calibration in flash

### Helper Tools
- **`serial_data_collector.py`** - Automated serial data capture
  - Lists available serial ports
  - Filters out non-data lines automatically
  - Interactive or command-line mode
  - Works on macOS, Linux, Windows

---

## 🔄 Complete Workflow

### Step 1: Hardware Setup

**You need:**
- ESP32 or Arduino board
- Accelerometer or magnetometer (MPU6050, MPU9250, QMI8658C, etc.)
- USB cable
- Computer with Python 3.x

**Wiring (I2C):**
```
ESP32 Pin 21 (SDA) → Sensor SDA
ESP32 Pin 22 (SCL) → Sensor SCL
ESP32 3.3V → Sensor VCC
ESP32 GND → Sensor GND
```

### Step 2: Collect Calibration Data

**Option A: Using Arduino IDE/Serial Monitor**

1. Open `esp32_calibration_data_collection.ino`
2. Configure for your sensor (change I2C address, init functions)
3. Upload to ESP32
4. Open Serial Monitor (115200 baud)
5. Follow on-screen instructions
6. Copy/paste all data between dashed lines to a text file

**Option B: Using Python Helper Script**

```bash
# List available serial ports
python serial_data_collector.py --list

# Run in interactive mode
python serial_data_collector.py

# Or specify port directly
python serial_data_collector.py --port /dev/cu.usbserial-1420 --output accel_data.txt
```

**Critical Requirements:**

For **Accelerometer**:
- ✅ Keep sensor STATIONARY in each orientation
- ✅ Rotate through ALL 3D orientations (all faces, edges, corners)
- ✅ Hold each position 2-3 seconds
- ✅ 100-300 samples total
- ❌ Don't move/shake during collection

For **Magnetometer**:
- ✅ Move sensor SLOWLY in figure-8 patterns
- ✅ Cover all 3D orientations smoothly
- ✅ Stay >1 meter from metal, electronics, magnets
- ✅ 300-1000 samples total
- ❌ Don't move too fast

### Step 3: Run Calibration

```bash
python ellipsoid_calibration.py sensor_data.txt
```

**What it does:**
1. Loads your sensor data
2. Fits ellipsoid using constrained least squares
3. Computes bias vector and calibration matrix
4. Shows visualization (raw vs calibrated data)
5. Saves calibration to `calibration.npz`
6. Displays calibration quality metrics

**Expected output:**
```
Calibrating with 450 samples...
Data range: X[-0.98, 1.02], Y[-1.01, 0.99], Z[-0.95, 1.05]
Largest eigenvalue: 0.023456
Bias (offset): [0.1234, -0.0987, 0.0543]
Measured field magnitude: 1.0123
Reference field strength: 1.0000
Scaling factor: 0.9878

Calibration Matrix:
[[0.987654  0.012345  0.006789]
 [0.012345  1.023456  0.009876]
 [0.006789  0.009876  0.995432]]

============================================================
CALIBRATION QUALITY METRICS
============================================================
Raw data magnitude:        1.0234 ± 0.0567
Calibrated data magnitude: 1.0000 ± 0.0089
Improvement in std dev:    84.3%
============================================================
```

### Step 4: Apply Calibration

**Copy the bias and calibration matrix into your ESP32 code:**

```cpp
// From Python output
float accel_bias[3] = {0.1234, -0.0987, 0.0543};

float accel_cal_matrix[3][3] = {
  {0.987654, 0.012345, 0.006789},
  {0.012345, 1.023456, 0.009876},
  {0.006789, 0.009876, 0.995432}
};
```

**Upload `esp32_use_calibration.ino` and test!**

### Step 5: Validate

**Quick validation (accelerometer):**
1. Place sensor flat on table
2. Check reading ≈ (0, 0, 1.0) with magnitude ≈ 1.0
3. Flip upside down
4. Check reading ≈ (0, 0, -1.0) with magnitude ≈ 1.0

**Detailed validation:**
- Run the validation function in `esp32_use_calibration.ino`
- Check standard deviation < 2% of mean magnitude
- Verify angles match physical orientation

---

## 📐 Theory and Mathematics

### Ellipsoid Equation

The general ellipsoid equation in 3D:
```
ax² + by² + cz² + 2fyz + 2gxz + 2hxy + 2px + 2qy + 2rz + d = 0
```

In matrix form:
```
X^T Q X + 2U^T X + J = 0
```

Where:
- `X = [x, y, z]^T` is the measurement vector
- `Q` is a 3×3 symmetric matrix (quadratic terms)
- `U` is a 3×1 vector (linear terms)
- `J` is a scalar (constant term)

### Constraint

To ensure the solution is an **ellipsoid** (not hyperboloid or paraboloid), we enforce:
```
4abc + fgh - af² - bg² - ch² > 0
```

This is implemented via the constraint matrix `C`.

### Parameter Extraction

From the fitted ellipsoid parameters:

**1. Bias (center of ellipsoid):**
```
B = -Q⁻¹ U
```

**2. Field magnitude:**
```
h = √(B^T Q B - J)
```

**3. Calibration matrix:**
```
A⁻¹ = (h_ref / h) × √Q
```

Where `√Q` is the matrix square root obtained via eigendecomposition.

### Algorithm Steps

1. Build design matrix `D` from measurements
2. Compute scatter matrix `S = D D^T`
3. Partition `S` into blocks `[S₁₁, S₁₂; S₁₂^T, S₂₂]`
4. Apply constraint via `E = C (S₁₁ - S₁₂ S₂₂⁻¹ S₁₂^T)`
5. Solve eigenvalue problem to find parameters
6. Extract Q, U, J from eigenvector
7. Compute bias and calibration matrix

---

## 🔧 Troubleshooting

### "Poor calibration quality" / High standard deviation

**Causes:**
- Insufficient 3D coverage → Collect data in MORE orientations
- Magnetic interference (magnetometer) → Move away from metal
- Movement during collection (accelerometer) → Stay completely still
- Too few samples → Collect 2-3× more data

**Fix:** Recollect data following guidelines in `DATA_COLLECTION_GUIDE.md`

### "Calibration makes things worse"

**Causes:**
- Data collected in wrong conditions (different temperature, location)
- Mixed data from different sensors
- Corrupt data file

**Fix:** Start fresh, collect new calibration data carefully

### "Can't connect to serial port"

**macOS:**
```bash
# List ports
ls /dev/cu.*

# Check permissions
sudo chmod 666 /dev/cu.usbserial-*
```

**Linux:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in

# Or use sudo
sudo python serial_data_collector.py --port /dev/ttyUSB0
```

**Windows:**
```
Check Device Manager → Ports (COM & LPT)
Use port like COM3, COM4, etc.
```

### "Sensor readings are in wrong units"

**No problem!** The algorithm works with raw ADC values, LSB counts, g's, m/s², Gauss, µT, etc.

Just ensure:
- Consistent units for all samples
- Specify appropriate reference field strength:
  - Accelerometer: `1.0` (representing 1g)
  - Magnetometer: Local field strength (~0.25-0.65 Gauss depending on location)

### "Matrix values look strange"

**Normal ranges:**
- Diagonal elements: 0.8 - 1.2 (± 20% is typical)
- Off-diagonal elements: -0.1 to 0.1 (small is good)
- Bias values: Can be 10-30% of full scale

If values are orders of magnitude off:
- Check data units
- Verify sensor is working properly
- Ensure data was collected correctly

---

## ❓ FAQ

### Q: How often should I recalibrate?

**A:** 
- Initial calibration: Once per board/sensor
- Recalibration needed if:
  - Sensor mounting changes
  - Temperature environment changes significantly
  - Magnetic environment changes (magnetometer)
  - After physical impact/shock
  - Yearly maintenance calibration recommended

### Q: Can I use the same calibration for multiple identical sensors?

**A:**
Not recommended. Each sensor has unique manufacturing variations. Calibrate each individually for best results.

### Q: Does temperature affect calibration?

**A:**
Yes! Sensors drift with temperature. For best results:
- Warm up sensor 5-10 minutes before collecting data
- Use sensor in similar temperature range as calibration
- Advanced: Implement temperature compensation

### Q: My magnetometer calibration is very poor. Why?

**A:**
Magnetometers are VERY sensitive to nearby metal and electronics:
- Collect data >1m from computers, phones, tablets
- Avoid metal furniture, reinforced concrete
- Keep away from motors, speakers, transformers
- Even steel-toed boots can affect readings!

### Q: Can I calibrate gyroscope with this method?

**A:**
No. Gyroscopes measure rotation rate (not a field), so they don't form ellipsoids. Use different calibration methods:
- Bias: Average readings when stationary
- Scale: Compare to known rotation rate

### Q: What's a good sample count?

**A:**
- **Minimum:** 100 samples (accelerometer), 200 (magnetometer)
- **Recommended:** 300-500 (accelerometer), 500-1000 (magnetometer)
- **More is better** if you have good 3D coverage

### Q: How do I know if my coverage is good?

**A:**
The Python script shows a 3D plot. Good coverage looks like:
- ✅ Points distributed ALL AROUND (sphere/ellipsoid surface)
- ❌ NOT: Clustered in one region
- ❌ NOT: Only on one arc or plane

### Q: Can I use this with other sensors?

**A:**
Yes! Any sensor measuring a 3D field with bias and scaling errors:
- Accelerometers (all types)
- Magnetometers (all types)
- Even: Electric field sensors, pressure sensor arrays, etc.

Just collect data as (x, y, z) samples.

### Q: Do I need exact field strength for magnetometer?

**A:**
Not critical. The algorithm can use measured field strength. But for best results:
- Look up local magnetic field strength: [NOAA Geomagnetic Calculator](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml)
- Typical values: 0.25-0.65 Gauss (Earth's field)

### Q: Why constrained least squares instead of simple least squares?

**A:**
Simple least squares can fit hyperboloids or paraboloids (not ellipsoids). The constraint ensures:
```
4abc + fgh - af² - bg² - ch² > 0
```
This guarantees an ellipsoid solution, which is physically correct for sensor errors.

---

## 📚 Additional Resources

### Papers & Theory
- "Ellipsoid Fit" - Yury Petrov (Mathworks)
- "Calibration of Three-Axis Magnetometer Using Stretching Particle Swarm Optimization Based Approach"
- "A Simple and Efficient Algorithm for Large-Scale Ellipsoid Fitting"

### Related Projects
- [PyIMU](https://github.com/pyIMU/pyIMU) - IMU calibration tools
- [Magneto](http://www.sailboatinstruments.blogspot.com) - Original C implementation
- [imutools](https://github.com/CCNYRoboticsLab/imu_tools) - ROS IMU calibration

### Sensor Datasheets
Always refer to your specific sensor datasheet for:
- Register maps
- Sensitivity values
- Measurement ranges
- I2C addresses

---

## 🤝 Contributing

Improvements welcome! Areas for enhancement:
- Support for more sensor types
- Temperature compensation
- Real-time calibration
- GUI interface
- Mobile app integration

---

## 📄 License

This implementation is based on public domain algorithms and is provided as-is for educational and practical use.

---

## 🆘 Support

If you encounter issues:

1. Read `DATA_COLLECTION_GUIDE.md` carefully
2. Check your sensor datasheet
3. Verify I2C connections
4. Try with synthetic data first
5. Open an issue with:
   - Your data file
   - Sensor type
   - Error messages
   - Plots/visualization

---

**Happy calibrating! 🎯**
