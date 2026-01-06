# Minimal Motor Driver - Learning Edition

A clean, educational ESP32 motor driver designed for **learning embedded systems** step-by-step.

## 🎯 What This Is

A **minimal, well-documented** motor driver that:
- ✅ Reads 9-axis IMU data (accelerometer, gyroscope, magnetometer)
- ✅ Monitors battery voltage and current
- ✅ Controls two motors via PWM
- ✅ Communicates over serial (USB)
- ✅ Streams data to MotionCal for calibration

**What makes it different:**
- 📚 Extensive inline comments explaining **WHY**, not just WHAT
- 🧩 Modular architecture (each component separate)
- 🔍 Easy to debug (know exactly where problems are)
- 📖 Complete tutorial included
- 🚀 RTOS-ready (prepared for FreeRTOS migration)

## 🚫 What This Doesn't Have

Deliberately removed for clarity:
- ❌ OLED display code
- ❌ WiFi/WebSocket complexity  
- ❌ Madgwick filter (complex orientation math)
- ❌ Multiple configuration files
- ❌ Unclear dependencies

**Philosophy:** Start simple, add features incrementally.

## 📁 Project Structure

```
minimal_driver/
├── config.h              # Pin definitions and constants
├── i2c_helpers.h/cpp     # I2C communication utilities
├── imu.h/cpp             # 9-axis IMU interface
├── battery.h/cpp         # Battery monitoring
├── motors.h/cpp          # PWM motor control
├── main.cpp              # Main program entry point
├── platformio.ini        # Build configuration
├── TUTORIAL.md           # Step-by-step learning guide
├── QUICK_REFERENCE.md    # Command reference
└── README.md             # This file
```

## 🚀 Quick Start

### Prerequisites

- **Hardware:**
  - ESP32 development board
  - QMI8658C IMU (accelerometer + gyroscope)
  - AK09918C magnetometer
  - INA219 battery monitor
  - L298N motor driver (or similar)
  - 2x DC motors

- **Software:**
  - PlatformIO (recommended) OR Arduino IDE
  - Serial terminal program

### Installation

1. **Clone or download this project**

2. **Open in PlatformIO:**
   ```bash
   cd minimal_driver
   code .  # Opens VS Code with PlatformIO
   ```

3. **Build and upload:**
   - Click ✓ (Build)
   - Click → (Upload)
   - Click 🔌 (Serial Monitor)

4. **Verify it works:**
   - You should see startup messages
   - Try command: `A` (all data)
   - Try command: `M50,50` (motors at 20% speed)

### First Steps

1. **Read the [TUTORIAL.md](TUTORIAL.md)** - Comprehensive guide
2. **Reference [QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Command list
3. **Experiment** - Modify the code, break things, learn!

## 📚 Learning Path

This project is designed for **progressive learning**:

### Level 1: Understand the Code (Week 1)
- Read all source files
- Understand header vs implementation
- Trace program flow from setup() to loop()
- Follow the tutorial step-by-step

### Level 2: Basic Modifications (Week 2)
- Change pin definitions
- Adjust timing intervals
- Add debug prints
- Modify motor speeds

### Level 3: Add Features (Week 3-4)
- Add LED status indicators
- Implement safety features (low battery warning)
- Add new serial commands
- Log data to SD card

### Level 4: Architecture Changes (Month 2)
- Add new sensors
- Implement simple filters
- Create state machines
- Add closed-loop control

### Level 5: RTOS Migration (Month 3)
- Convert to FreeRTOS tasks
- Add synchronization (mutexes)
- Implement queues
- Handle priorities

## 🎓 What You'll Learn

### Programming Concepts
- **C++ Fundamentals:** Classes, objects, references, pointers
- **Memory Management:** Stack, heap, global storage
- **File Organization:** Headers, implementation, includes
- **Build Systems:** Compilation, linking, dependencies

### Embedded Concepts
- **Hardware Communication:** I2C, UART, PWM
- **Timing:** Non-blocking loops, periodic tasks
- **State Management:** Sensor data, control flow
- **Debugging:** Serial debug, systematic troubleshooting

### Architecture Concepts
- **Modularity:** Separation of concerns
- **Abstraction:** Hardware interfaces
- **Scalability:** Adding features cleanly
- **Real-Time:** Preparing for RTOS

## 📖 Documentation

| Document | Purpose |
|----------|---------|
| [TUTORIAL.md](TUTORIAL.md) | Complete step-by-step guide |
| [QUICK_REFERENCE.md](QUICK_REFERENCE.md) | Command reference & troubleshooting |
| Code comments | Explain WHY decisions were made |

## 🔧 Hardware Connections

```
ESP32               IMU (QMI8658C + AK09918C)
Pin 32  (SDA) ───── SDA
Pin 33  (SCL) ───── SCL
3.3V          ───── VCC
GND           ───── GND

ESP32               Battery Monitor (INA219)
Pin 32  (SDA) ───── SDA
Pin 33  (SCL) ───── SCL
3.3V          ───── VCC
GND           ───── GND

ESP32               Motor Driver (L298N)
Pin 25        ───── PWM A (speed)
Pin 17        ───── IN1 A (direction)
Pin 21        ───── IN2 A (direction)
Pin 26        ───── PWM B (speed)
Pin 22        ───── IN1 B (direction)
Pin 23        ───── IN2 B (direction)
GND           ───── GND (common ground!)
```

**⚠️ Important:**
- Use 3.3V for sensors (NOT 5V!)
- Common ground between ESP32 and motor driver
- Motor driver needs separate power (7-12V)

## 🎮 Serial Commands

| Command | Description | Example |
|---------|-------------|---------|
| `M<L>,<R>` | Set motor speeds | `M100,100` |
| `S` | Stop motors | `S` |
| `R` | RAW data (MotionCal) | `R` |
| `O` | Orientation angles | `O` |
| `T` | Temperature | `T` |
| `B` | Battery status | `B` |
| `A` | All data | `A` |

**Baud rate:** 115200

## 🐛 Troubleshooting

### Motors don't move
- Check power supply (7-12V to motor driver)
- Verify connections (see wiring diagram)
- Test with low speed first: `M25,25`

### IMU not found
- Check I2C connections (SDA, SCL)
- Verify 3.3V power (NOT 5V!)
- Run I2C scanner (see tutorial)

### Compilation errors
- Check all .cpp files are in build
- Verify `extern` declarations
- See QUICK_REFERENCE.md troubleshooting section

## 🚀 Next Steps

After mastering this project:

1. **Add complexity gradually:**
   - Implement Madgwick filter for orientation
   - Add WiFi for remote control
   - Create autonomous behaviors

2. **Learn RTOS:**
   - Port to FreeRTOS
   - Implement tasks and queues
   - Handle synchronization

3. **Build something:**
   - Self-balancing robot
   - Line follower
   - Autonomous rover
   - Robotic arm

## 🤝 Contributing

This is a learning project! Improvements welcome:
- Better explanations
- Additional examples
- Tutorial enhancements
- Bug fixes

## 📄 License

MIT License - Free to use, modify, learn from!

## 🙏 Acknowledgments

- Original codebase: Waveshare rover driver
- Sensor libraries: Based on vendor datasheets
- Educational approach: Designed for learners

## 💡 Philosophy

**Why this project exists:**

Too many embedded tutorials either:
1. Show you what to copy (no understanding)
2. Assume you already know (too advanced)

This project **explains every decision**:
- WHY we use headers and implementation files
- WHY we need non-blocking timing
- WHY modules are separated
- WHAT happens if we do it differently

**Goal:** Not just working code, but **understanding** how and why it works.

## 📧 Questions?

Read the code comments - they explain the WHY behind every decision.

Still stuck? Check:
1. TUTORIAL.md (comprehensive guide)
2. QUICK_REFERENCE.md (specific problems)
3. Code inline comments (design decisions)

**Good luck on your embedded journey!** 🚀

---

**Remember:** Every expert was once a beginner. Take your time, experiment, break things, and learn from the process.
