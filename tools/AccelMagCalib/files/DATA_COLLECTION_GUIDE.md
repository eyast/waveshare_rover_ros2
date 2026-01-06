# COMPREHENSIVE GUIDE: Collecting Calibration Data

## Table of Contents
1. [Why Calibration is Necessary](#why-calibration-is-necessary)
2. [What Happens Without Calibration](#what-happens-without-calibration)
3. [Data Collection Procedure](#data-collection-procedure)
4. [Common Mistakes and How to Avoid Them](#common-mistakes-and-how-to-avoid-them)
5. [Data Format](#data-format)
6. [Validation](#validation)

---

## 1. Why Calibration is Necessary

### For Accelerometers:
**Ideal behavior**: When you rotate a stationary accelerometer in any orientation, 
the magnitude of the reading should always be exactly 1g (9.81 m/s²).

**Reality**: Manufacturing imperfections cause:
- **Zero-g offset**: Sensor doesn't read (0,0,0) in free fall
- **Sensitivity mismatch**: Each axis has slightly different gain
- **Non-orthogonality**: Axes aren't perfectly 90° apart
- **Result**: Readings form an offset ellipsoid instead of a sphere

### For Magnetometers:
**Ideal behavior**: When you rotate a magnetometer, readings should trace a 
sphere centered at (0,0,0) with radius = local magnetic field strength.

**Reality**:
- **Hard-iron effects**: Permanent magnets on your board (speakers, motors) 
  create constant offset
- **Soft-iron effects**: Ferromagnetic materials (iron, steel) distort the 
  magnetic field differently in each direction
- **Result**: Readings form an offset, distorted ellipsoid

### What the Calibration Fixes:
1. **Bias (B)**: Shifts the center back to origin
2. **Calibration Matrix (A⁻¹)**: Transforms ellipsoid back to sphere
   - Corrects sensitivity differences
   - Corrects non-orthogonality
   - Normalizes to reference field strength

---

## 2. What Happens Without Calibration

### Accelerometer Issues:
1. **Tilt/Attitude Errors**:
   - ±5-10° errors in roll/pitch angles are common
   - Accumulates in dead reckoning
   
2. **Velocity Integration Errors**:
   - Even tiny bias causes velocity drift
   - Example: 0.01g bias → 10 cm/s error per second → 36 m/s in 1 minute!

3. **Activity Recognition Failures**:
   - Walking vs running thresholds become unreliable
   - Gesture recognition accuracy drops significantly

### Magnetometer Issues:
1. **Compass Heading Errors**:
   - ±10-30° heading errors typical
   - Can be completely backwards in extreme cases
   
2. **Sensor Fusion Breakdown**:
   - IMU algorithms rely on magnetometer for yaw
   - Bad mag → IMU drift → unusable orientation
   
3. **Position Errors**:
   - If using mag-based positioning: position errors grow unbounded

### Real Example:
```
Uncalibrated accelerometer reading while stationary:
X: 0.12g, Y: -0.05g, Z: 1.03g → Magnitude = 1.037g ❌

After calibration:
X: 0.00g, Y: 0.00g, Z: 1.00g → Magnitude = 1.000g ✅
```

---

## 3. Data Collection Procedure

### Equipment Needed:
- Your sensor board (accelerometer/magnetometer)
- Power supply (battery or USB)
- Computer to log data
- Flat, non-magnetic surface (for accelerometer)
- Magnetically clean area (for magnetometer - away from metal, electronics)

### CRITICAL REQUIREMENTS:

#### For Accelerometers:
✅ **DO:**
- Collect data in a stationary state (no movement)
- Rotate to cover ALL orientations in 3D space
- Minimum 100 samples (300-500 recommended)
- Sample at moderate rate (10-100 Hz is fine)
- Keep sensor still in each orientation for 2-3 seconds

❌ **DON'T:**
- Move sensor while collecting data
- Shake or vibrate the sensor
- Only collect data in a few orientations
- Rush through orientations

#### For Magnetometers:
✅ **DO:**
- Move sensor SLOWLY and SMOOTHLY
- Cover ALL orientations in 3D space
- Stay far from metal objects, electronics, magnets
- Minimum 200 samples (500-1000 recommended)
- Sample continuously while rotating

❌ **DON'T:**
- Collect data near computers, motors, speakers
- Move too fast (creates motion artifacts)
- Only rotate around one axis
- Collect data in different locations (field strength varies)

### Step-by-Step Procedure:

#### For Accelerometer:

**Method 1: Box Method (Easiest)**
1. Place sensor flat on table (+Z up) → wait 3 sec → record 30 samples
2. Flip upside down (-Z up) → wait 3 sec → record 30 samples
3. Stand on +X edge → wait 3 sec → record 30 samples
4. Stand on -X edge → wait 3 sec → record 30 samples
5. Stand on +Y edge → wait 3 sec → record 30 samples
6. Stand on -Y edge → wait 3 sec → record 30 samples

**Method 2: Tumbling Method (More comprehensive)**
1. Start logging data
2. Slowly rotate sensor through all possible orientations
3. Spend 2-3 seconds in each orientation
4. Cover at least:
   - All 6 faces (like a dice)
   - All 12 edges
   - Several corner positions
5. Total: 300-500 samples over 2-3 minutes

#### For Magnetometer:

**Figure-8 Pattern (Recommended)**
1. Start logging data continuously
2. Hold sensor and wave it in large figure-8 patterns:
   - Figure-8 in horizontal plane (yaw rotation)
   - Figure-8 in vertical plane (pitch rotation)
   - Figure-8 in side plane (roll rotation)
3. Repeat each pattern 3-4 times
4. Move SLOWLY and SMOOTHLY
5. Total: 500-1000 samples over 1-2 minutes

**Sphere Pattern (Alternative)**
1. Start logging data continuously
2. Imagine sensor is a paintbrush painting inside of a sphere
3. Move sensor to cover entire inside surface
4. Rotate around all three axes
5. Total: 500-1000 samples over 1-2 minutes

### Coverage Verification:
Good calibration needs FULL 3D coverage. Check:

```python
# After collecting data, visualize it
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(data[:,0], data[:,1], data[:,2])
plt.show()

# Should see:
# ✅ Points distributed all around (like surface of a sphere/ellipsoid)
# ❌ NOT: Points clustered in one region
# ❌ NOT: Points only on one plane or arc
```

---

## 4. Common Mistakes and How to Avoid Them

### Mistake 1: Insufficient Coverage
**Problem**: Only rotating around one axis (e.g., only yaw)
**Result**: Calibration will fail or give poor results
**Fix**: Must rotate around ALL THREE axes

### Mistake 2: Moving During Collection (Accelerometer)
**Problem**: Accelerometer sees motion, not just gravity
**Result**: Data includes acceleration, ruins calibration
**Fix**: Stay completely still in each orientation

### Mistake 3: Nearby Magnetic Interference (Magnetometer)
**Problem**: Metal objects, electronics affect readings
**Result**: Calibrates to wrong ellipsoid
**Fix**: Collect data >1 meter from:
- Computers, phones, tablets
- Metal furniture, reinforced concrete
- Speakers, motors, transformers
- Power lines, electrical panels

### Mistake 4: Too Few Samples
**Problem**: <50 samples don't cover enough space
**Result**: Poor fit, unreliable calibration
**Fix**: Aim for:
- Accelerometer: 100-300 samples
- Magnetometer: 300-1000 samples

### Mistake 5: Inconsistent Motion Speed (Magnetometer)
**Problem**: Moving too fast or jerking motion
**Result**: Motion artifacts, poor data quality
**Fix**: Move SLOWLY and SMOOTHLY

### Mistake 6: Temperature Drift
**Problem**: Sensor warms up during collection
**Result**: Bias changes, calibration less effective
**Fix**: Let sensor warm up 5-10 minutes before collecting

---

## 5. Data Format

### File Format:
Simple text file, one measurement per line, tab or space separated:

```
x_value    y_value    z_value
-23.45     45.12      12.78
-21.34     43.89      15.23
-19.23     42.67      17.89
...
```

### Units:
- **Accelerometer**: Any units (g, m/s², LSB from sensor)
  - Algorithm works with raw ADC values
  - Will normalize to 1.0 (representing 1g)
  
- **Magnetometer**: Any units (Gauss, µT, LSB from sensor)
  - Algorithm works with raw ADC values
  - Specify reference field strength for your location
  - Earth's field: ~0.25-0.65 Gauss (depends on location)

### Example File (accelerometer_data.txt):
```
-0.12  0.05  1.03
-0.10  0.03  1.02
 0.15 -0.02 -0.98
 0.14 -0.03 -0.99
 1.02  0.08  0.01
 1.01  0.07  0.02
-1.00  0.05 -0.03
-0.99  0.06 -0.02
 0.03  1.05  0.01
 0.02  1.04  0.00
 0.01 -0.98  0.02
 0.02 -0.99  0.03
```

### Saving from Python:
```python
import numpy as np
data = np.array([[x1,y1,z1], [x2,y2,z2], ...])
np.savetxt('sensor_data.txt', data, fmt='%.6f', delimiter='\t')
```

### Saving from C/C++:
```cpp
FILE *fp = fopen("sensor_data.txt", "w");
for (int i = 0; i < n_samples; i++) {
    fprintf(fp, "%.6f\t%.6f\t%.6f\n", x[i], y[i], z[i]);
}
fclose(fp);
```

---

## 6. Validation

### After Calibration:

#### Check 1: Visual Inspection
Plot raw vs calibrated data - should see:
- Raw data: ellipsoid, off-center
- Calibrated data: sphere, centered at origin

#### Check 2: Magnitude Statistics
Calculate magnitude of all calibrated samples:
```python
magnitudes = np.linalg.norm(calibrated_data, axis=1)
mean_mag = np.mean(magnitudes)
std_mag = np.std(magnitudes)
print(f"Mean: {mean_mag:.4f}, Std: {std_mag:.4f}")
```

**Good calibration:**
- Accelerometer: mean ≈ 1.0, std < 0.02 (2% variation)
- Magnetometer: mean ≈ reference field, std < 5% of mean

**Poor calibration (needs recollection):**
- std > 0.05 (5% variation) → insufficient coverage
- mean far from expected → wrong reference or bad data

#### Check 3: Real-World Test

**Accelerometer:**
1. Place sensor flat on table
2. Check calibrated reading ≈ (0, 0, 1.0)
3. Flip upside down
4. Check calibrated reading ≈ (0, 0, -1.0)
5. Stand on each edge, verify magnitude = 1.0

**Magnetometer:**
1. Hold sensor horizontal
2. Rotate 360° slowly
3. Note maximum and minimum X, Y readings
4. Should be roughly ±same magnitude
5. Check heading changes smoothly 0-360°

---

## Quick Reference: Data Collection Checklist

### Before Collection:
- [ ] Sensor warmed up (5-10 min)
- [ ] Data logging software ready
- [ ] Accelerometer: flat surface available
- [ ] Magnetometer: magnetically clean area confirmed
- [ ] Power supply connected
- [ ] Know sampling rate (10-100 Hz typical)

### During Collection:
- [ ] Covering all orientations in 3D
- [ ] Accelerometer: holding still in each position
- [ ] Magnetometer: moving slowly and smoothly
- [ ] Collecting enough samples (100-1000 depending on sensor)
- [ ] No external disturbances

### After Collection:
- [ ] Data saved to text file
- [ ] Visual inspection (3D scatter plot)
- [ ] Number of samples adequate
- [ ] Coverage looks complete (full sphere/ellipsoid)
- [ ] Ready to run calibration algorithm

---

## Troubleshooting

### "Calibration makes things worse"
→ Insufficient 3D coverage, collect more data in different orientations

### "Very high standard deviation after calibration"
→ Magnetic interference (mag) or movement during collection (accel)

### "Calibration matrix has huge values"
→ Raw data units might be ADC counts (thousands), use normalized values

### "Different calibrations each time"
→ Not collecting data in same conditions (temperature, location, magnetic environment)

### "Bias values seem too large"
→ Normal! Bias can be 10-30% of full scale, that's why calibration is needed

---

## Summary

**Good calibration requires:**
1. ✅ Complete 3D orientation coverage
2. ✅ Sufficient samples (100-1000)
3. ✅ Stationary measurements (accelerometer)
4. ✅ Slow smooth motion (magnetometer)
5. ✅ Clean magnetic environment (magnetometer)
6. ✅ Proper validation after calibration

**Result:**
- Accelerometer magnitude accuracy: <2% error
- Magnetometer heading accuracy: <5° error
- Reliable sensor fusion and orientation tracking
