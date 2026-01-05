# System Architecture Diagram

## High-Level Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        MAIN LOOP                            │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐           │
│  │  Update    │  │  Update    │  │  Process   │           │
│  │  IMU       │  │  Battery   │  │  Serial    │           │
│  │  (50Hz)    │  │  (1Hz)     │  │  Commands  │           │
│  └──────┬─────┘  └──────┬─────┘  └──────┬─────┘           │
│         │                │                │                 │
└─────────┼────────────────┼────────────────┼─────────────────┘
          │                │                │
          ▼                ▼                ▼
    ┌─────────┐      ┌─────────┐      ┌─────────┐
    │   IMU   │      │ Battery │      │ Motors  │
    │  Module │      │  Module │      │  Module │
    └────┬────┘      └────┬────┘      └────┬────┘
         │                │                 │
         │                │                 │
    ┌────▼────┐      ┌────▼────┐      ┌────▼────┐
    │   I2C   │      │   I2C   │      │   PWM   │
    │ Helpers │      │ Helpers │      │  GPIO   │
    └────┬────┘      └────┬────┘      └────┬────┘
         │                │                 │
         ▼                ▼                 ▼
    ┌─────────┐      ┌─────────┐      ┌─────────┐
    │QMI8658C │      │ INA219  │      │ L298N   │
    │AK09918C │      │         │      │ H-Bridge│
    └─────────┘      └─────────┘      └─────────┘
```

## Module Dependencies

```
main.cpp
  ├─► config.h          (constants only, no dependencies)
  ├─► i2c_helpers.h     (depends on: Wire library)
  ├─► imu.h
  │    ├─► i2c_helpers.h
  │    └─► config.h
  ├─► battery.h
  │    ├─► i2c_helpers.h
  │    └─► config.h
  └─► motors.h
       └─► config.h
```

## Data Flow

```
SENSOR READING:
Hardware → I2C Bus → i2c_helpers → IMU Module → IMU_Data struct → main.cpp

MOTOR CONTROL:
main.cpp → Motors Module → PWM/GPIO → Hardware

SERIAL COMMUNICATION:
User → USB → Serial → Command Buffer → process_command() → Module functions
Module data → Formatting → Serial → USB → User
```

## Memory Layout

```
GLOBAL MEMORY (allocated at startup, lives forever):
┌──────────────────────────────────────┐
│ IMU imu;           (in imu.cpp)      │
│ Battery battery;   (in battery.cpp)  │
│ Motors motors;     (in motors.cpp)   │
└──────────────────────────────────────┘

STACK MEMORY (created/destroyed in functions):
┌──────────────────────────────────────┐
│ void loop() {                        │
│   unsigned long now;     ← stack     │
│   char c;                ← stack     │
│ }                                    │
│ // now and c destroyed here          │
└──────────────────────────────────────┘

HEAP MEMORY (not used in this project):
┌──────────────────────────────────────┐
│ (unused - no dynamic allocation)     │
└──────────────────────────────────────┘
```

## Timing Diagram

```
Time →
0ms         20ms        40ms        60ms        1000ms
│           │           │           │           │
├─ IMU ─────┼─ IMU ─────┼─ IMU ─────┼─ IMU ─────┼─ IMU + Battery
│           │           │           │           │
└─ Serial processing happens anytime a character arrives

IMU updates:     Every 20ms  (50Hz)
Battery updates: Every 1000ms (1Hz)
Serial:          Asynchronous (whenever data arrives)
```

## File Relationships

```
PROJECT ROOT
│
├── DOCUMENTATION
│   ├── README.md          ← Start here
│   ├── TUTORIAL.md        ← Learning guide
│   └── QUICK_REFERENCE.md ← Command reference
│
├── CONFIGURATION
│   ├── platformio.ini     ← Build settings
│   └── config.h           ← Pin & constant definitions
│
├── UTILITIES
│   ├── i2c_helpers.h      ← Interface
│   └── i2c_helpers.cpp    ← Implementation
│
├── HARDWARE MODULES
│   ├── imu.h / imu.cpp            ← Sensor reading
│   ├── battery.h / battery.cpp    ← Power monitoring
│   └── motors.h / motors.cpp      ← Motor control
│
└── APPLICATION
    └── main.cpp           ← Program entry point
```

## Communication Protocols

```
I2C BUS (400kHz):
ESP32 Pin 32 (SDA) ───┬─── QMI8658C (0x6B)
                      ├─── AK09918C (0x0C)
                      └─── INA219   (0x42)
ESP32 Pin 33 (SCL) ───┘

PWM (20kHz):
ESP32 Pin 25 ──► Motor A Speed
ESP32 Pin 26 ──► Motor B Speed

GPIO (Digital):
ESP32 Pin 17 ──► Motor A Direction 1
ESP32 Pin 21 ──► Motor A Direction 2
ESP32 Pin 22 ──► Motor B Direction 1
ESP32 Pin 23 ──► Motor B Direction 2

UART (115200 baud):
ESP32 USB ◄──► Computer (commands & data)
```

## Execution Flow

```
STARTUP:
1. Power On
2. ESP32 Bootloader runs
3. Global constructors called:
   - IMU::IMU()
   - Battery::Battery()
   - Motors::Motors()
4. setup() runs:
   - Serial.begin()
   - i2c_init()
   - imu.begin()
   - battery.begin()
   - motors.begin()
5. loop() starts

LOOP (repeats forever):
┌─────────────────────────────────┐
│ 1. Check timing                 │
│ 2. Update IMU if 20ms passed    │
│ 3. Update battery if 1s passed  │
│ 4. Process serial if data ready │
│ 5. Repeat                       │
└─────────────────────────────────┘
```

## Future RTOS Architecture (Preview)

```
CURRENT (Bare Metal):
loop() {
  update_imu();      ← blocks everything
  update_battery();  ← blocks everything
  process_serial();  ← blocks everything
}

FUTURE (FreeRTOS):
Task 1: IMU_Task (Priority 2)
  ├─ Update IMU every 20ms
  └─ Can be preempted by higher priority

Task 2: Battery_Task (Priority 1)
  ├─ Update battery every 1000ms
  └─ Lower priority than IMU

Task 3: Serial_Task (Priority 3)
  ├─ Process commands as they arrive
  └─ Highest priority (user responsiveness)

Scheduler manages task execution
  ↓
Better CPU utilization
More responsive system
Guaranteed timing
```
