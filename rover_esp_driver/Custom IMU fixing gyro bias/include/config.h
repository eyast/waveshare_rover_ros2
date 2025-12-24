#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// Hardware Configuration
// =============================================================================

#define I2C_SDA 32
#define I2C_SCL 33
#define I2C_FREQ 400000

#define SERIAL_BAUD 115200

// =============================================================================
// I2C Addresses
// =============================================================================

#define QMI8658C_ADDR 0x6B
#define AK09918C_ADDR 0x0C

// =============================================================================
// Sensor Configuration
// =============================================================================

// Accelerometer: 0=±2g, 1=±4g, 2=±8g, 3=±16g
#define ACCEL_FS_SEL 3

// Gyroscope: 0=±16dps, 1=±32dps, 2=±64dps, 3=±128dps
//            4=±256dps, 5=±512dps, 6=±1024dps, 7=±2048dps
#define GYRO_FS_SEL 7

// ODR: 0=8000Hz, 1=4000Hz, 2=2000Hz, 3=1000Hz
//      4=500Hz, 5=250Hz, 6=117.5Hz, 7=58.75Hz
#define ODR_SEL 3

#define MAG_SCALE 0.15f  // uT/LSB

// =============================================================================
// Output Configuration
// =============================================================================

#define OUTPUT_RATE_HZ 50
#define OUTPUT_INTERVAL_MS (1000 / OUTPUT_RATE_HZ)

// =============================================================================
// Filter Configuration
// =============================================================================

#define MADGWICK_BETA 0.1f

// Adaptive beta settings (motion-based gain adjustment)
#define ADAPTIVE_BETA_ENABLED     true
#define ADAPTIVE_BETA_STATIONARY  0.5f    // When device is still
#define ADAPTIVE_BETA_MOTION      0.05f   // When device is moving
#define MOTION_THRESHOLD_RADS     0.05f   // ~3 deg/s

// Fast convergence settings
#define FAST_CONVERGENCE_DURATION_MS  2000
#define FAST_CONVERGENCE_BETA         2.5f

// =============================================================================
// JSON Command Types (must match json_cmd.h)
// =============================================================================

// Motor commands (existing - from json_cmd.h)
#define CMD_SPEED_CTRL      1
#define CMD_PWM_INPUT       11
#define CMD_SET_MOTOR_PID   2

// OLED commands (existing)
#define CMD_OLED_CTRL       3
#define CMD_OLED_DEFAULT    -3

// System commands (existing)
#define CMD_REBOOT          600
#define CMD_MM_TYPE_SET     900

// IMU Stream commands (existing - from json_cmd.h)
#define CMD_IMU_STREAM_CTRL 325   // {"T":325,"cmd":1}
#define CMD_STREAM_FORMAT   400   // {"T":400,"cmd":1}

// =============================================================================
// IMU Control Commands (330-360 range)
// =============================================================================

// Calibration commands
#define CMD_IMU_CALIBRATE_GYRO   330   // {"T":330}
#define CMD_IMU_CALIBRATE_ACCEL  331   // {"T":331}
#define CMD_IMU_CALIBRATE_ALL    332   // {"T":332}

// Filter commands
#define CMD_IMU_FILTER_RESET     335   // {"T":335}
#define CMD_IMU_SET_BETA         336   // {"T":336,"beta":0.1}

// Magnetometer axis correction
#define CMD_IMU_SET_MAG_SIGNS    340   // {"T":340,"x":1,"y":1,"z":-1}

// Debug/status commands
#define CMD_IMU_DEBUG            345   // {"T":345}
#define CMD_IMU_GET_STATUS       346   // {"T":346}

// Orientation query
#define CMD_IMU_GET_ORIENTATION  350   // {"T":350} -> {"T":350,"yaw":x,"pitch":y,"roll":z}

// =============================================================================
// NEW: Fast Initialization & Adaptive Beta Commands (351-360)
// =============================================================================

// Initialize filter from current sensor readings (instant alignment!)
// This bypasses the slow convergence entirely
#define CMD_IMU_INIT_FROM_SENSORS  351   // {"T":351}

// Start fast convergence mode (high beta for quick alignment)
// duration: milliseconds (0 = until stable)
// beta: fast mode beta (default 2.5)
#define CMD_IMU_FAST_CONVERGENCE   352   // {"T":352,"duration":2000,"beta":2.5}

// Configure adaptive beta (motion-based gain adjustment)
// enabled: 0/1
// stationary: beta when still (default 0.5)
// motion: beta when moving (default 0.05)
// threshold: motion threshold in rad/s (default 0.05)
#define CMD_IMU_ADAPTIVE_BETA      353   // {"T":353,"enabled":1,"stationary":0.5,"motion":0.05,"threshold":0.05}

// Get filter convergence status
#define CMD_IMU_CONVERGENCE_STATUS 354   // {"T":354} -> {"T":354,"rate":0.01,"converged":true,"active_beta":0.1}

#endif // CONFIG_H
