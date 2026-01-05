#ifndef CONFIG_H
#define CONFIG_H

#define SDA 32
#define SCL 33
#define I2CFREQUENCY 100000
#define BAUDRATE 115200

#define SYS_ID "B:"

// =============================================================================
// Sensor refresh intervals
// =============================================================================

#define IMU_UPDATE_MS 500 // Update IMU every 20 Ms

// =============================================================================
// I2C DEVICE ADDRESSES
// =============================================================================

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
#define ACCEL_RANGE 3

// Gyroscope full-scale range  
// 0=±16°/s, 1=±32°/s, 2=±64°/s, 3=±128°/s
// 4=±256°/s, 5=±512°/s, 6=±1024°/s, 7=±2048°/s
// WHAT: Measures rotation speed in degrees per second
#define GYRO_RANGE 2048

// Output Data Rate (how often sensor updates)
// 0=8000Hz, 1=4000Hz, 2=2000Hz, 3=1000Hz, 4=500Hz, 5=250Hz, 6=125Hz, 7=62.5Hz
#define SENSOR_ODR 7 // Recommended ODR to use QMI AttitudeEngine 


#endif