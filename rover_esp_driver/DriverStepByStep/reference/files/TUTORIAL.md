# 🎓 Minimal Motor Driver - Complete Learning Tutorial

## 📋 Table of Contents
1. [Understanding the Big Picture](#understanding-the-big-picture)
2. [How C++ Code is Organized](#how-c-code-is-organized)
3. [Understanding Memory and Objects](#understanding-memory-and-objects)
4. [Step-by-Step Code Walkthrough](#step-by-step-code-walkthrough)
5. [How to Build and Upload](#how-to-build-and-upload)
6. [Testing Your System](#testing-your-system)
7. [Debugging Techniques](#debugging-techniques)
8. [Extending the Code](#extending-the-code)
9. [Preparing for RTOS](#preparing-for-rtos)

---

## Understanding the Big Picture

### What We Built

You now have a **modular embedded system** with these components:

```
┌─────────────────────────────────────────┐
│           ESP32 Main Loop               │
│  ┌────────────────────────────────────┐ │
│  │  Read Sensors Every 20ms           │ │
│  │  • IMU (accel, gyro, mag)          │ │
│  │  • Battery (every 1000ms)          │ │
│  └────────────────────────────────────┘ │
│  ┌────────────────────────────────────┐ │
│  │  Process Serial Commands           │ │
│  │  • Motor control                   │ │
│  │  • Data requests                   │ │
│  └────────────────────────────────────┘ │
└─────────────────────────────────────────┘
         │              │              │
    ┌────┴────┐    ┌───┴───┐    ┌─────┴─────┐
    │   IMU   │    │Battery│    │  Motors   │
    │ Module  │    │Module │    │  Module   │
    └─────────┘    └───────┘    └───────────┘
         │              │              │
    ┌────┴────┐    ┌───┴───┐    ┌─────┴─────┐
    │I2C Sensor│   │I2C Sensor│ │PWM H-Bridge│
    │QMI8658C │    │ INA219 │   │  L298N    │
    └─────────┘    └────────┘   └───────────┘
```

### Why This Matters

**What you have:**
- ✅ Clean separation of concerns (each module does ONE thing)
- ✅ Easy to test (each module works independently)
- ✅ Easy to debug (know exactly where problems are)
- ✅ Easy to extend (add features without breaking existing code)
- ✅ RTOS-ready (each module can become a task)

**What you DON'T have (intentionally removed):**
- ❌ OLED display code
- ❌ WiFi/WebSocket complexity
- ❌ Madgwick filter (complex math)
- ❌ Multiple configuration files
- ❌ Unclear dependencies

---

## How C++ Code is Organized

### The Header File (.h) vs Implementation File (.cpp) Pattern

#### Example: The IMU Module

**imu.h** (The Interface - What is available):
```cpp
class IMU {
public:
    bool begin();        // Declaration: "This function exists"
    bool update();       // Declaration: "This function exists"
private:
    IMU_Data data;       // Declaration: "This variable exists"
};
```

**imu.cpp** (The Implementation - How it works):
```cpp
bool IMU::begin() {
    // ACTUAL CODE THAT DOES THE WORK
    i2c_init(...);
    configure_sensors();
    return true;
}
```

### Why Separate Them?

**The Restaurant Analogy:**
- **.h file = Menu** (tells you what's available, not how to cook it)
- **.cpp file = Kitchen** (where the actual cooking happens)

**What happens if you put everything in .h:**
- Every file that includes it gets ALL the implementation code
- Compile time increases (rebuilding the kitchen for every customer)
- Hard to find the actual code
- "Multiple definition" errors (multiple kitchens making the same dish)

**What happens with proper separation:**
- Fast compilation (kitchen built once, menu copied many times)
- Clear interface (menu) vs implementation (kitchen)
- Easy to change implementation without affecting users
- No duplication errors

### Include Guards - Preventing Double Inclusion

```cpp
#ifndef IMU_H    // "If not defined IMU_H"
#define IMU_H    // "Define IMU_H"

// ... your code ...

#endif // IMU_H
```

**Why needed:**
- Prevents including the same file twice
- Avoids "redefinition" errors

**What happens without them:**
```cpp
// file1.h includes imu.h
// file2.h includes imu.h  
// main.cpp includes BOTH file1.h and file2.h
// → Without guards: IMU class defined TWICE → ERROR
// → With guards: IMU class defined ONCE → OK
```

---

## Understanding Memory and Objects

### Where Things Live in Memory

#### 1. **Stack Memory** (Automatic/Local Variables)

```cpp
void loop() {
    int counter = 0;           // Lives on stack
    float temperature = 25.5;  // Lives on stack
    // When loop() ends, these are DESTROYED
}
```

**Properties:**
- Fast allocation
- Automatic cleanup (destroyed when function ends)
- Small (ESP32: ~8KB stack)
- Used for local variables

**What happens:**
```
loop() starts  → Stack grows (allocates counter, temperature)
loop() ends    → Stack shrinks (deallocates everything)
loop() starts  → New stack space (old data GONE)
```

#### 2. **Heap Memory** (Dynamic Allocation)

```cpp
void setup() {
    int* big_array = new int[1000];  // Allocated on heap
    // Stays alive until you call: delete[] big_array;
}
```

**Properties:**
- Slower allocation
- Manual cleanup (YOU must delete)
- Larger (ESP32: ~300KB heap)
- Used for large/long-lived data

**Danger:**
```cpp
void bad_function() {
    int* data = new int[100];
    // FORGOT TO DELETE!
    // → MEMORY LEAK (lost forever, can't use that RAM)
}

// Call this 1000 times → run out of memory → crash
```

#### 3. **Global/Static Memory**

```cpp
// In imu.cpp:
IMU imu;  // Global object - lives ENTIRE program lifetime

// In main.cpp:
extern IMU imu;  // Says "imu exists somewhere else"
```

**Properties:**
- Allocated at program start
- Lives forever (until program ends)
- Known address (doesn't move)
- Used for hardware interfaces, shared state

### Our Architecture Uses Globals - Why?

```cpp
// In imu.cpp:
IMU imu;  // The ACTUAL object

// In main.cpp:
extern IMU imu;  // Reference to that object

// In battery.cpp:
extern IMU imu;  // Same reference - everyone sees ONE object
```

**Why this is good for embedded:**
- Hardware only exists once (one I2C bus, one battery)
- No passing pointers everywhere
- Clear, simple access
- RTOS-friendly (shared resources have fixed addresses)

**Alternative (worse for embedded):**
```cpp
void setup() {
    IMU imu;  // Stack object
    process_data(&imu);  // Must pass pointer
    control_motors(&imu);  // Must pass pointer
    update_display(&imu);  // Must pass pointer
    // Gets messy fast!
}
```

---

## Step-by-Step Code Walkthrough

### Part 1: System Startup (setup)

Let's trace what happens when you power on:

```
1. ESP32 boots
   ↓
2. Runs global constructors
   - IMU imu;      → IMU::IMU() called
   - Battery battery; → Battery::Battery() called
   - Motors motors;  → Motors::Motors() called
   ↓
3. setup() runs
   ↓
4. Serial.begin(115200)
   - Opens USB serial port at 115200 baud
   - WHY 115200: Fast enough for data, slow enough to be reliable
   ↓
5. i2c_init(32, 33, 400000)
   - Pin 32 = SDA (data line)
   - Pin 33 = SCL (clock line)  
   - 400kHz = communication speed
   - Enables I2C hardware peripheral
   ↓
6. imu.begin()
   - Checks WHO_AM_I registers (confirms chips present)
   - Configures accelerometer range (±16g)
   - Configures gyroscope range (±2048°/s)
   - Enables sensors
   - Returns true if successful
   ↓
7. battery.begin()
   - Writes configuration to INA219
   - Sets up voltage/current measurement
   - Returns true if successful
   ↓
8. motors.begin()
   - Configures GPIO pins as outputs
   - Sets up PWM channels
   - Ensures motors start OFF
   ↓
9. setup() finishes
   ↓
10. loop() starts (runs forever)
```

### Part 2: Main Loop Execution

Every time loop() runs (hundreds of times per second):

```cpp
void loop() {
    unsigned long now = millis();  // Get current time
    
    // CHECK: Should we update IMU?
    if (now - last_imu_update >= 20) {  // 20ms = 50Hz
        last_imu_update = now;
        imu.update();  // Read fresh sensor data
    }
    
    // CHECK: Should we update battery?
    if (now - last_battery_update >= 1000) {  // 1000ms = 1Hz
        last_battery_update = now;
        battery.update();  // Read fresh battery data
    }
    
    // CHECK: Any serial commands?
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            process_command(command_buffer);
            command_buffer = "";
        } else {
            command_buffer += c;
        }
    }
}
```

**What's happening:**

1. **Non-blocking timing:**
   - Don't use `delay()` - would freeze everything
   - Instead: remember when we last did something
   - Check if enough time has passed

2. **Different update rates:**
   - IMU: 50Hz (fast - motion changes quickly)
   - Battery: 1Hz (slow - battery changes slowly)
   - WHY: Don't waste CPU on slow-changing data

3. **Serial processing:**
   - Check if data available (non-blocking)
   - Accumulate characters until newline
   - Process complete command

### Part 3: Reading the IMU

Let's trace `imu.update()`:

```cpp
bool IMU::update() {
    // STEP 1: Read QMI8658C (accel + gyro)
    if (imu_ok) {
        // Check status register: is data ready?
        uint8_t status = i2c_read_byte(0x6B, 0x2E);
        if (status & 0x03) {  // Bits 0-1 = accel/gyro ready
            
            // Read 14 bytes: temp + accel + gyro
            uint8_t buffer[14];
            i2c_read_bytes(0x6B, 0x33, buffer, 14);
            
            // Parse temperature (bytes 0-1)
            int16_t temp_raw = (buffer[1] << 8) | buffer[0];
            data.temperature = temp_raw / 256.0f;
            
            // Parse accel (bytes 2-7)
            data.accel_raw[0] = (buffer[3] << 8) | buffer[2];  // X
            data.accel_raw[1] = (buffer[5] << 8) | buffer[4];  // Y
            data.accel_raw[2] = (buffer[7] << 8) | buffer[6];  // Z
            
            // Parse gyro (bytes 8-13)
            data.gyro_raw[0] = (buffer[9] << 8) | buffer[8];   // X
            data.gyro_raw[1] = (buffer[11] << 8) | buffer[10]; // Y
            data.gyro_raw[2] = (buffer[13] << 8) | buffer[12]; // Z
            
            // Convert to real units
            for (int i = 0; i < 3; i++) {
                data.accel[i] = data.accel_raw[i] * accel_scale;
                data.gyro[i] = data.gyro_raw[i] * gyro_scale;
            }
        }
    }
    
    // STEP 2: Read AK09918C (magnetometer)
    if (mag_ok) {
        // Check data ready
        uint8_t status = i2c_read_byte(0x0C, 0x10);
        if (status & 0x01) {
            // Read 6 bytes: X, Y, Z magnetic field
            uint8_t buffer[8];
            i2c_read_bytes(0x0C, 0x11, buffer, 8);
            
            data.mag_raw[0] = (buffer[1] << 8) | buffer[0];
            data.mag_raw[1] = (buffer[3] << 8) | buffer[2];
            data.mag_raw[2] = (buffer[5] << 8) | buffer[4];
            
            for (int i = 0; i < 3; i++) {
                data.mag[i] = data.mag_raw[i] * 0.15f;  // Fixed scale
            }
        }
    }
    
    // STEP 3: Calculate simple orientation
    calculate_orientation();
    
    return true;
}
```

**Why this works:**
- Data-ready flags prevent reading stale data
- Burst reads are faster than individual reads
- Convert once, use many times
- Keep both raw (for calibration) and scaled (for use) values

---

## How to Build and Upload

### Option 1: Using PlatformIO (Recommended)

1. **Install PlatformIO:**
   ```bash
   # If using VS Code:
   # Install "PlatformIO IDE" extension
   ```

2. **Open Project:**
   ```bash
   cd minimal_driver
   code .  # Opens VS Code
   ```

3. **Build:**
   - Click ✓ icon at bottom (Build)
   - OR: Press `Ctrl+Alt+B`
   - OR: Terminal: `pio run`

4. **Upload:**
   - Click → icon (Upload)
   - OR: Press `Ctrl+Alt+U`
   - OR: Terminal: `pio run --target upload`

5. **Monitor:**
   - Click 🔌 icon (Serial Monitor)
   - OR: Terminal: `pio device monitor`

### Option 2: Using Arduino IDE

1. **Copy files to Arduino project folder**

2. **Select Board:**
   - Tools → Board → ESP32 Dev Module

3. **Select Port:**
   - Tools → Port → (your ESP32 port)

4. **Upload:**
   - Click → button
   - OR: Sketch → Upload

---

## Testing Your System

### Test 1: Basic Connectivity

**Expected Output After Upload:**
```
======================
  MINIMAL IMU DRIVER  
======================

Initializing I2C...
[I2C] Initialized

=== Initializing IMU ===
[IMU] QMI8658C OK
[IMU] AK09918C OK

=== Initializing Battery Monitor ===
[Battery] INA219 OK

=== Initializing Motors ===
[Motors] Initialized

=== Available Commands ===
...

[System] Ready!
```

**If IMU fails:**
```
[IMU] QMI8658C wrong ID: 0xFF
```
→ Check I2C connections (SDA, SCL)
→ Check I2C address (might be different)
→ Try I2C scanner to find devices

**If Battery fails:**
```
[Battery] INA219 read test failed!
```
→ Check if INA219 connected
→ Verify address (0x42 or 0x40?)

### Test 2: Motor Control

**Send command:**
```
M100,100
```

**Expected:**
```
[Motors] Set: L=100 R=100
```

**Motors should spin forward at ~40% speed**

**Test reverse:**
```
M-100,-100
```

**Test differential:**
```
M100,-100
```
→ Should spin in place

**Stop:**
```
S
```
→ Motors should stop

### Test 3: Sensor Data

**Request RAW data:**
```
R
```

**Expected:**
```
Raw:123,-456,8192,15,22,-8,450,320,-380
```
→ These are scaled for MotionCal

**Request Orientation:**
```
O
```

**Expected:**
```
Ori: 2.3,-1.8
```
→ Roll and Pitch in degrees

**Request All Data:**
```
A
```

**Expected:**
```
--- All Data ---
IMU:
  Accel (g): 0.012, -0.003, 1.002
  Gyro (°/s): 0.5, -0.2, 0.1
  Mag (µT): 23.4, -12.1, -42.3
  Orientation: Roll=2.3° Pitch=-1.8°
  Temperature: 28.5 °C
Battery:
  Voltage: 7.42 V
  Current: 125.3 mA
  Power: 929.5 mW
---------------
```

---

## Debugging Techniques

### Problem: Code Won't Compile

**Error:** `undefined reference to 'IMU::begin()'`

**Cause:** .cpp file not included in build

**Fix:** Check platformio.ini has correct source filter
```ini
build_src_filter = +<*> -<.git/> -<svn/>
```

**Error:** `'imu' was not declared in this scope`

**Cause:** Missing `extern IMU imu;`

**Fix:** Add to top of file:
```cpp
#include "imu.h"
extern IMU imu;
```

### Problem: IMU Not Responding

**Debug steps:**

1. **Add debug prints:**
```cpp
bool IMU::init_qmi8658c() {
    Serial.println("[DEBUG] Reading WHO_AM_I...");
    uint8_t who_am_i = i2c_read_byte(QMI8658C_ADDR, QMI_WHO_AM_I);
    Serial.print("[DEBUG] Got: 0x");
    Serial.println(who_am_i, HEX);
    
    if (who_am_i != QMI_WHO_AM_I_VALUE) {
        Serial.println("[DEBUG] WRONG VALUE!");
        return false;
    }
    // ...
}
```

2. **Check I2C:**
```cpp
void setup() {
    // ... after i2c_init() ...
    
    Serial.println("[DEBUG] Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[DEBUG] Found device at 0x");
            Serial.println(addr, HEX);
        }
    }
}
```

3. **Verify pins:**
- Wrong SDA/SCL? → No devices found
- Bad connection? → Intermittent errors
- Missing pullups? → Unreliable communication

### Problem: Motors Don't Move

**Check:**

1. **Power:**
   - Is motor driver powered? (separate from ESP32!)
   - Voltage high enough? (7-12V typical)

2. **Connections:**
   - PWM pins correct?
   - Direction pins correct?
   - Common ground between ESP32 and motor driver?

3. **Code:**
```cpp
void Motors::set_motor(bool is_left, int16_t speed) {
    Serial.print("[DEBUG] Setting motor: ");
    Serial.print(is_left ? "LEFT" : "RIGHT");
    Serial.print(" Speed: ");
    Serial.println(speed);
    
    // ... rest of function ...
}
```

### Problem: Data Looks Wrong

**Accelerometer should read ~1g when stationary:**
```cpp
void loop() {
    if (imu.update()) {
        const IMU_Data& data = imu.get_data();
        float magnitude = sqrt(data.accel[0]*data.accel[0] + 
                              data.accel[1]*data.accel[1] + 
                              data.accel[2]*data.accel[2]);
        
        if (magnitude < 0.9 || magnitude > 1.1) {
            Serial.print("[WARNING] Accel magnitude wrong: ");
            Serial.println(magnitude);
            // Should be ~1.0 (gravity)
        }
    }
}
```

---

## Extending the Code

### Adding a New Feature: Automatic Data Streaming

**Goal:** Continuously send data without typing commands

**Implementation:**

```cpp
// In config.h:
#define AUTO_STREAM_ENABLED true
#define AUTO_STREAM_INTERVAL_MS 100  // 10Hz

// In main.cpp:
unsigned long last_auto_stream = 0;

void loop() {
    // ... existing code ...
    
    #if AUTO_STREAM_ENABLED
    unsigned long now = millis();
    if (now - last_auto_stream >= AUTO_STREAM_INTERVAL_MS) {
        last_auto_stream = now;
        send_raw_data();  // Or send_all_data()
    }
    #endif
}
```

**Why this works:**
- Uses same timing pattern as IMU/battery
- Conditional compilation (`#if`) lets you turn it on/off
- Doesn't interfere with command processing

### Adding a New Sensor

**Example: Adding BMP280 (pressure sensor)**

**Step 1: Create module files**

```cpp
// bmp280.h
#ifndef BMP280_H
#define BMP280_H

struct BMP280_Data {
    float temperature;
    float pressure;
    float altitude;
};

class BMP280 {
public:
    BMP280();
    bool begin();
    bool update();
    const BMP280_Data& get_data() const { return data; }
    bool is_ok() const { return sensor_ok; }
    
private:
    BMP280_Data data;
    bool sensor_ok;
};

extern BMP280 bmp280;

#endif
```

**Step 2: Implement in .cpp**

```cpp
// bmp280.cpp
#include "bmp280.h"
#include "i2c_helpers.h"

#define BMP280_ADDR 0x76

BMP280 bmp280;

// ... implementation ...
```

**Step 3: Add to main.cpp**

```cpp
#include "bmp280.h"

void setup() {
    // ... existing code ...
    
    if (!bmp280.begin()) {
        Serial.println("[WARNING] BMP280 not found");
    }
}

void loop() {
    // ... existing code ...
    
    if (now - last_bmp_update >= BMP_UPDATE_MS) {
        last_bmp_update = now;
        if (bmp280.is_ok()) {
            bmp280.update();
        }
    }
}
```

**That's it!** New sensor integrated with minimal impact on existing code.

---

## Preparing for RTOS

### What is RTOS?

**RTOS = Real-Time Operating System**

**What we have now (bare metal):**
```
loop() {
    update_imu();       // Takes 2ms
    update_battery();   // Takes 1ms
    process_serial();   // Takes ???ms
    // All sequential - one thing blocks the next
}
```

**What RTOS gives us:**
```
Task 1 (IMU):          [████] [████] [████]
Task 2 (Battery):   [█]       [█]       [█]
Task 3 (Serial):    [█████][█][██][█][███]
                    ↑
                    Scheduler manages who runs when
```

**Benefits:**
- ✅ Tasks run independently
- ✅ Priorities (IMU more important than battery)
- ✅ Better CPU utilization
- ✅ Guaranteed timing (hard real-time)

### How Our Code is Already RTOS-Ready

**1. Modular design:**
```cpp
// Each module = future task
IMU imu;        → Task: IMU_Task
Battery battery; → Task: Battery_Task
Motors motors;   → Task: Motor_Task
```

**2. Global objects:**
```cpp
// RTOS tasks can access globals easily
extern IMU imu;  // All tasks see same IMU
```

**3. No shared state problems:**
```cpp
// Each module owns its data
// No race conditions (yet!)
```

### Next Steps for RTOS (FreeRTOS)

**Convert loop() to tasks:**

```cpp
// Instead of loop(), create tasks:

void imu_task(void* parameter) {
    while (true) {
        imu.update();
        vTaskDelay(20 / portTICK_PERIOD_MS);  // 50Hz
    }
}

void battery_task(void* parameter) {
    while (true) {
        battery.update();
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // 1Hz
    }
}

void setup() {
    // ... initialize hardware ...
    
    // Create tasks
    xTaskCreate(imu_task, "IMU", 2048, NULL, 2, NULL);
    xTaskCreate(battery_task, "Battery", 2048, NULL, 1, NULL);
    
    // Start scheduler
    vTaskStartScheduler();
}

void loop() {
    // Never runs - scheduler takes over
}
```

**But wait! Need to add synchronization:**

```cpp
// Protect shared data with mutex
SemaphoreHandle_t imu_mutex;

void imu_task(void* parameter) {
    while (true) {
        xSemaphoreTake(imu_mutex, portMAX_DELAY);
        imu.update();
        xSemaphoreGive(imu_mutex);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void process_command() {
    xSemaphoreTake(imu_mutex, portMAX_DELAY);
    send_raw_data();  // Read IMU data safely
    xSemaphoreGive(imu_mutex);
}
```

**That's for later!** First master this simple version.

---

## Summary: What You Learned

### Technical Skills

1. **C++ Basics:**
   - Header vs implementation files
   - Classes and objects
   - Public vs private
   - References vs pointers

2. **Memory Management:**
   - Stack vs heap vs global
   - When to use each
   - Object lifetime
   - Extern declarations

3. **Embedded Programming:**
   - I2C communication
   - GPIO control
   - PWM generation
   - Non-blocking timing
   - Serial communication

4. **Software Architecture:**
   - Modular design
   - Separation of concerns
   - Single responsibility
   - Dependency management

### Debugging Skills

1. **Systematic approach:**
   - Add debug prints
   - Check hardware first
   - Verify assumptions
   - Isolate problems

2. **Tools:**
   - Serial monitor
   - I2C scanner
   - Logic analyzer (next step)

### Next Steps

1. **Practice:**
   - Add LED status indicators
   - Implement safety features (battery low warning)
   - Add data logging to SD card

2. **Learn:**
   - Study FreeRTOS basics
   - Read about interrupts
   - Understand DMA

3. **Build:**
   - This is your foundation
   - Every complex system starts simple
   - Add features one at a time

**You're ready to grow from here!** 🚀
