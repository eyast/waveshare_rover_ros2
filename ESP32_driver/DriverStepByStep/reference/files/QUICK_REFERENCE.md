# 📖 Quick Reference Guide

## Serial Commands

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Motor Control | `M<left>,<right>` | `M100,100` | Set motor speeds (-255 to +255) |
| Stop Motors | `S` | `S` | Emergency stop both motors |
| RAW Data | `R` | `R` | Send sensor data in MotionCal format |
| Orientation | `O` | `O` | Send roll/pitch angles |
| Temperature | `T` | `T` | Send IMU temperature |
| Battery | `B` | `B` | Send battery voltage/current/power |
| All Data | `A` | `A` | Send comprehensive data dump |

## Common Operations

### Check if System is Working

**Upload code and open Serial Monitor. You should see:**
```
======================
  MINIMAL IMU DRIVER  
======================

...
[IMU] QMI8658C OK
[IMU] AK09918C OK
[Battery] INA219 OK
[Motors] Initialized
[System] Ready!
```

### Test Motors Safely

**Before connecting motors:**
1. Upload code
2. Open Serial Monitor
3. Send: `M50,50`
4. Measure voltage on motor driver outputs
5. Should see PWM signal (~20% duty cycle)

**After motors connected:**
1. Start with low speed: `M25,25`
2. Increase gradually: `M50,50`, `M100,100`
3. Emergency stop ready: `S`

### Calibrate with MotionCal

1. **Start streaming:**
   - Open Serial Monitor
   - Type: `R` and press Enter
   - You should see continuous output:
     ```
     Raw:123,-456,8192,15,22,-8,450,320,-380
     Raw:125,-454,8190,16,23,-9,448,322,-382
     ...
     ```

2. **Open MotionCal:**
   - Download from: https://www.pjrc.com/store/prop_shield.html
   - Select your ESP32's serial port
   - Should see dots appearing as you rotate device

3. **Collect data:**
   - Slowly rotate device in all axes
   - Get good coverage of sphere
   - Wait for gap to close to < 5%

4. **Get calibration values:**
   - Copy hard iron offsets
   - Copy soft iron matrix
   - Add to your code (see below)

### Apply Calibration

**Add to imu.cpp, in `init_ak09918c()`:**

```cpp
bool IMU::init_ak09918c() {
    // ... existing init code ...
    
    // Apply calibration from MotionCal
    // Replace these values with yours:
    mag_offset[0] = 25.15f;   // Hard iron X
    mag_offset[1] = -56.42f;  // Hard iron Y
    mag_offset[2] = -27.62f;  // Hard iron Z
    
    // Soft iron matrix (if needed)
    float soft_iron[3][3] = {
        {1.039f, 0.013f, -0.061f},
        {0.013f, 1.061f,  0.003f},
        {-0.061f, 0.003f, 0.911f}
    };
    
    return true;
}
```

**Then in `read_ak09918c()`, apply corrections:**

```cpp
bool IMU::read_ak09918c() {
    // ... read raw data ...
    
    // Apply hard iron correction
    for (int i = 0; i < 3; i++) {
        data.mag[i] = (data.mag_raw[i] * mag_scale) - mag_offset[i];
    }
    
    // Apply soft iron correction (if needed)
    float mx = data.mag[0];
    float my = data.mag[1];
    float mz = data.mag[2];
    
    data.mag[0] = soft_iron[0][0]*mx + soft_iron[0][1]*my + soft_iron[0][2]*mz;
    data.mag[1] = soft_iron[1][0]*mx + soft_iron[1][1]*my + soft_iron[1][2]*mz;
    data.mag[2] = soft_iron[2][0]*mx + soft_iron[2][1]*my + soft_iron[2][2]*mz;
    
    return true;
}
```

## Troubleshooting

### Problem: No Serial Output

**Check:**
- ✓ Correct COM port selected?
- ✓ Correct baud rate (115200)?
- ✓ USB cable connected?
- ✓ USB cable supports data? (some are power-only)
- ✓ Driver installed for ESP32?

**Try:**
```bash
# List serial ports (Linux/Mac):
ls /dev/tty*

# List serial ports (Windows):
# Device Manager → Ports (COM & LPT)
```

### Problem: IMU Not Found

**Error:**
```
[IMU] QMI8658C wrong ID: 0xFF
```

**Check:**
1. **I2C connections:**
   - SDA to pin 32
   - SCL to pin 33
   - VCC to 3.3V (NOT 5V!)
   - GND to GND

2. **I2C address:**
   - Run I2C scanner (add to setup):
   ```cpp
   for (uint8_t addr = 1; addr < 127; addr++) {
       Wire.beginTransmission(addr);
       if (Wire.endTransmission() == 0) {
           Serial.print("Found: 0x");
           Serial.println(addr, HEX);
       }
   }
   ```
   - If you see 0x6A instead of 0x6B, change `QMI8658C_ADDR`

3. **Power:**
   - Measure 3.3V on VCC pin
   - Check current (should be < 50mA)

### Problem: Battery Monitor Not Found

**Error:**
```
[Battery] INA219 not responding!
```

**Check:**
1. Is INA219 connected?
2. I2C address correct (0x42 or 0x40)?
3. Try I2C scanner to find it

**If not needed:**
- Comment out in main.cpp:
```cpp
void setup() {
    // ... other init ...
    
    // battery.begin();  // Commented out
}
```

### Problem: Motors Run But Won't Stop

**Possible causes:**
1. **Wiring issue:**
   - Direction pins crossed?
   - Check connections match pin definitions

2. **Motor driver problem:**
   - Test with multimeter
   - Measure voltage on driver outputs

3. **Code issue:**
```cpp
// Add debug to stop():
void Motors::stop() {
    Serial.println("[DEBUG] Stopping motors");
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
    Serial.println("[DEBUG] All pins set LOW");
}
```

### Problem: Data Looks Noisy/Wrong

**Accelerometer:**
- Should read ~1.0g magnitude when still
- If way off: check calibration
- If noisy: add low-pass filter (later feature)

**Gyroscope:**
- Should read ~0°/s when still
- If drifting: normal for MEMS gyros
- If very large values: check connections

**Magnetometer:**
- Very sensitive to interference
- Keep away from motors, batteries, magnets
- Test by rotating in circles - should see smooth sine waves

### Problem: Compilation Errors

**Error: `undefined reference to 'IMU::begin()'`**
- Missing .cpp file in build
- Check platformio.ini

**Error: `'imu' was not declared`**
- Missing `extern IMU imu;`
- Add to top of file

**Error: `multiple definition of 'imu'`**
- Remove `IMU imu;` from .h file
- Should only be in .cpp file

## Performance Tips

### Optimize Serial Output

**Problem:** Printing slows down loop

**Solution:** Print less often
```cpp
static unsigned long last_print = 0;
if (millis() - last_print >= 100) {  // 10Hz max
    last_print = millis();
    Serial.println("...");
}
```

### Reduce I2C Traffic

**Problem:** Reading sensors too fast

**Solution:** Match sensor ODR
```cpp
// If sensor updates at 100Hz, don't read at 1000Hz
#define IMU_UPDATE_MS 10  // 100Hz, matches sensor
```

### Battery Life

**Power consumption:**
- ESP32 active: ~100mA
- Motors: 100-500mA each (depends on load)
- Sensors: <10mA

**To save power:**
1. Lower ESP32 clock speed
2. Use deep sleep when idle
3. Disable sensors not in use
4. Reduce LED brightness

## File Structure Reference

```
minimal_driver/
├── config.h              # All constants
├── i2c_helpers.h/cpp     # I2C utilities
├── imu.h/cpp             # IMU module
├── battery.h/cpp         # Battery module
├── motors.h/cpp          # Motor module
├── main.cpp              # Entry point
├── platformio.ini        # Build config
└── TUTORIAL.md           # This guide
```

**When to edit each file:**

| File | Edit When |
|------|-----------|
| `config.h` | Change pins, timing, constants |
| `i2c_helpers.*` | Add I2C functions (rare) |
| `imu.*` | Add IMU features, change sensors |
| `battery.*` | Add battery features |
| `motors.*` | Change motor control logic |
| `main.cpp` | Add commands, change program flow |
| `platformio.ini` | Change board, add libraries |

## Safety Checklist

Before running motors:
- [ ] Code uploaded successfully
- [ ] Serial monitor working
- [ ] Motors disconnected for first test
- [ ] Emergency stop command ready (`S`)
- [ ] Speed started low (< 50)
- [ ] Battery voltage checked
- [ ] No loose wires
- [ ] Clear space around robot

## Next Steps

1. **Master the basics:**
   - Understand every line of code
   - Modify and experiment
   - Break things and fix them

2. **Add features:**
   - LED status indicators
   - Data logging
   - Automatic modes

3. **Learn RTOS:**
   - FreeRTOS basics
   - Task creation
   - Synchronization

4. **Build something cool:**
   - Line follower
   - Balance robot
   - Autonomous rover

**Good luck!** 🚀
