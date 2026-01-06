#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// HARDWARE PIN DEFINITIONS
// =============================================================================
// WHY: Centralizing pins makes it easy to change hardware without hunting
//      through code. If you move a wire, change it here only.

// I2C Pins
#define I2C_SDA 32
#define I2C_SCL 33
#define I2C_FREQ 400000  // 400kHz - standard fast mode

// Motor Driver Pins (L298N or similar dual H-bridge)
#define MOTOR_A_PWM  25    // Speed control for Motor A
#define MOTOR_A_IN1  17    // Direction pin 1 for Motor A
#define MOTOR_A_IN2  21    // Direction pin 2 for Motor A
#define MOTOR_B_PWM  26    // Speed control for Motor B
#define MOTOR_B_IN1  22    // Direction pin 1 for Motor B
#define MOTOR_B_IN2  23    // Direction pin 2 for Motor B

// =============================================================================
// I2C DEVICE ADDRESSES
// =============================================================================
// WHY: These are fixed by the hardware chips. Keep them here so we know
//      what's on the I2C bus at a glance.

#define QMI8658C_ADDR  0x6B  // IMU (Accelerometer + Gyroscope)
#define AK09918C_ADDR  0x0C  // Magnetometer
#define INA219_ADDR    0x42  // Battery monitor

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================
// WHY: These control the range and sensitivity of sensors.
//      Higher range = can measure bigger values but less precise.
//      Lower range = more precise but can't measure as high.

// Accelerometer full-scale range
// 0=±2g, 1=±4g, 2=±8g, 3=±16g
// WHAT: "g" = gravity (1g = 9.8 m/s²)
// WHY ±16g: Robot might experience high acceleration during motion
#define ACCEL_RANGE 3  // ±16g

// Gyroscope full-scale range  
// 0=±16°/s, 1=±32°/s, 2=±64°/s, 3=±128°/s
// 4=±256°/s, 5=±512°/s, 6=±1024°/s, 7=±2048°/s
// WHAT: Measures rotation speed in degrees per second
// WHY ±2048°/s: Fast spinning needs high range
#define GYRO_RANGE 7   // ±2048°/s

// Output Data Rate (how often sensor updates)
// 0=8KHz, 1=4KHz, 2=2KHz, 3=1KHz, 4=500Hz, 5=250Hz, 6=125Hz, 7=62.5Hz
// WHY 1000Hz: Good balance - fast enough for motion, not too much data
#define SENSOR_ODR 3   // 1000Hz

// =============================================================================
// TIMING CONFIGURATION
// =============================================================================
// WHY: Control how often we read sensors and send data
//      Too fast = waste CPU, flood serial port
//      Too slow = miss important changes

#define SERIAL_BAUD     115200  // Bits per second for UART

// How often to read and send IMU data (milliseconds)
#define IMU_UPDATE_MS   20      // 50Hz (every 20ms)

// How often to read battery (milliseconds)  
// WHY slower: Battery changes slowly, no need to check often
#define BATTERY_UPDATE_MS 1000  // 1Hz (every 1000ms = 1 second)

// =============================================================================
// MOTOR PWM CONFIGURATION
// =============================================================================
// WHY PWM: Motors need variable speed. PWM = Pulse Width Modulation
//          Fast on/off switching that averages to a voltage.
//          More "on" time = faster motor

#define PWM_FREQ      20000     // 20kHz - above human hearing range
#define PWM_RESOLUTION 8        // 8 bits = 0-255 range
#define PWM_CHANNEL_A  5        // ESP32 PWM channel for motor A
#define PWM_CHANNEL_B  6        // ESP32 PWM channel for motor B

// =============================================================================
// MOTIONCAL SCALING FACTORS
// =============================================================================
// WHY: MotionCal calibration tool expects specific number formats
//      We convert our actual units to what MotionCal wants

#define MOTIONCAL_ACCEL_SCALE 8192.0f   // LSB per g
#define MOTIONCAL_GYRO_SCALE  16.0f     // LSB per °/s  
#define MOTIONCAL_MAG_SCALE   10.0f     // LSB per µT

#endif // CONFIG_H
