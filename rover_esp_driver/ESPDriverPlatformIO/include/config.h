#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// Hardware Configuration
// =============================================================================

// I2C pins (adjust for your board)
#define I2C_SDA 32
#define I2C_SCL 33
#define I2C_FREQ 400000

// Serial configuration
#define SERIAL_BAUD 115200

// =============================================================================
// I2C Addresses
// =============================================================================

#define QMI8658C_ADDR 0x6B
#define AK09918C_ADDR 0x0C

// =============================================================================
// Sensor Configuration
// =============================================================================

// Accelerometer full-scale selection
// 0 = +/-2g, 1 = +/-4g, 2 = +/-8g, 3 = +/-16g
#define ACCEL_FS_SEL 3 //0

// Gyroscope full-scale selection
// 0 = +/-16dps, 1 = +/-32dps, 2 = +/-64dps, 3 = +/-128dps
// 4 = +/-256dps, 5 = +/-512dps, 6 = +/-1024dps, 7 = +/-2048dps
#define GYRO_FS_SEL 7 // 5

// Output data rate selection
// 0 = 8000Hz, 1 = 4000Hz, 2 = 2000Hz, 3 = 1000Hz
// 4 = 500Hz, 5 = 250Hz, 6 = 117.5Hz, 7 = 58.75Hz
#define ODR_SEL 3 //6

// Magnetometer scale factor (AK09918C)
#define MAG_SCALE 0.5f  // uT/LSB

// =============================================================================
// Output Configuration
// =============================================================================

// Output rate (Hz) - MotionCal works well at 20-50 Hz
#define OUTPUT_RATE_HZ 50
#define OUTPUT_INTERVAL_MS (1000 / OUTPUT_RATE_HZ)

// =============================================================================
// Filter Configuration
// =============================================================================

// Madgwick filter beta parameter
// Higher = faster convergence but more noise
// Lower = smoother but slower response
#define MADGWICK_BETA 0.05f

// =============================================================================
// IMU Stream Mode and Options
// =============================================================================

bool stream_as_json = false;
bool stream_orientiation = true;

#endif // CONFIG_H
