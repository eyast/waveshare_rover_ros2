#include "imu.h"
#include "i2c_helpers.h"
#include "config.h"
#include <math.h>

// =============================================================================
// QMI8658C REGISTER ADDRESSES
// =============================================================================
// WHY: Each sensor has internal "registers" - like memory addresses
// WHAT: To read/write sensor settings, we access these addresses

#define QMI_WHO_AM_I    0x00  // Device ID register (should read 0x05)
#define QMI_CTRL1       0x02  // Control register 1 (basic config)
#define QMI_CTRL2       0x03  // Accelerometer config
#define QMI_CTRL3       0x04  // Gyroscope config  
#define QMI_CTRL5       0x06  // Filter config
#define QMI_CTRL7       0x08  // Enable/disable sensors
#define QMI_CTRL9       0x0A  // Calibration commands
#define QMI_STATUS0     0x2E  // Data ready status
#define QMI_TEMP_L      0x33  // Temperature (low byte)
#define QMI_AX_L        0x35  // Accelerometer X (low byte) - data starts here

#define QMI_RESET       0x60  // Reset command register

// Expected WHO_AM_I value - confirms we're talking to right chip
#define QMI_WHO_AM_I_VALUE  0x05

// =============================================================================
// AK09918C REGISTER ADDRESSES  
// =============================================================================

#define AK_WIA1         0x00  // Company ID (should be 0x48)
#define AK_WIA2         0x01  // Device ID (should be 0x0C)
#define AK_ST1          0x10  // Status 1 (data ready flag)
#define AK_HXL          0x11  // Magnetic field X (low byte)
#define AK_CNTL2        0x31  // Control: operating mode
#define AK_CNTL3        0x32  // Control: software reset

// Expected WHO_AM_I values
#define AK_WIA1_VALUE   0x48
#define AK_WIA2_VALUE   0x0C

// Operating modes
#define AK_MODE_CONT_100HZ  0x08  // Continuous measurement at 100Hz

// =============================================================================
// GLOBAL INSTANCE
// =============================================================================
// WHY here: This is the actual object creation (not just declaration)
IMU imu;

// =============================================================================
// CONSTRUCTOR
// =============================================================================
// WHAT: Constructor is called automatically when object is created
// WHY: Sets everything to a known, safe starting state

IMU::IMU() {
    // Initialize flags to false - assume sensors not working until proven
    imu_ok = false;
    mag_ok = false;
    
    // Zero out all data
    // WHY: Start with clean slate, avoid using uninitialized values
    memset(&data, 0, sizeof(data));
    
    // Scales will be calculated during begin()
    accel_scale = 0;
    gyro_scale = 0;
    mag_scale = 0.15f;  // AK09918C fixed at 0.15 µT per LSB
}

// =============================================================================
// INITIALIZATION
// =============================================================================

bool IMU::begin() {
    Serial.println("\n=== Initializing IMU ===");
    
    // Try to initialize both sensors
    // WHAT: Each init function checks if device responds and configures it
    // WHY separate functions: Easier to debug which sensor has problems
    imu_ok = init_qmi8658c();
    mag_ok = init_ak09918c();
    
    // Report results
    if (imu_ok) {
        Serial.println("[IMU] QMI8658C OK");
    } else {
        Serial.println("[IMU] QMI8658C FAILED!");
    }
    
    if (mag_ok) {
        Serial.println("[IMU] AK09918C OK");
    } else {
        Serial.println("[IMU] AK09918C FAILED!");
    }
    
    // Return true only if both sensors OK
    // WHY: We need both for full 9-axis data
    return (imu_ok && mag_ok);
}

bool IMU::init_qmi8658c() {
    // STEP 1: Check if device is there
    // WHAT: Read WHO_AM_I register - should return specific value
    // WHY: Confirms we're talking to right chip at right address
    uint8_t who_am_i = i2c_read_byte(QMI8658C_ADDR, QMI_WHO_AM_I);
    
    if (who_am_i != QMI_WHO_AM_I_VALUE) {
        Serial.print("[IMU] QMI8658C wrong ID: 0x");
        Serial.println(who_am_i, HEX);
        return false;
    }
    
    // STEP 2: Software reset
    // WHY: Puts chip in known state, clears any previous configuration
    i2c_write_byte(QMI8658C_ADDR, QMI_RESET, 0xB0);
    delay(10);  // Wait for reset to complete
    
    // STEP 3: Configure CTRL1
    // WHAT: Sets up how data is formatted
    // 0x60 = address auto-increment (read multiple registers sequentially)
    // 0x18 = big-endian format
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL1, 0x60 | 0x18);
    
    // STEP 4: Disable sensors while configuring (safe practice)
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL7, 0x00);
    
    // STEP 5: Configure accelerometer
    // WHAT: Sets measurement range and update rate
    // Format: (range << 4) | output_data_rate
    // WHY shift by 4: Upper 4 bits = range, lower 4 bits = ODR
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL2, (ACCEL_RANGE << 4) | SENSOR_ODR);
    
    // STEP 6: Configure gyroscope (same format)
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL3, (GYRO_RANGE << 4) | SENSOR_ODR);
    
    // STEP 7: Enable low-pass filters
    // WHY: Reduces noise, makes data smoother
    // 0x11 = enable filters for both accel and gyro
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL5, 0x11);
    
    // STEP 8: Enable both sensors
    // 0x03 = enable accel (bit 0) and gyro (bit 1)
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL7, 0x03);
    
    // STEP 9: Calculate conversion scales
    // WHY: Raw data is in arbitrary units - we need real world units
    
    // Accelerometer: depends on range setting
    const float accel_ranges[] = {2.0f, 4.0f, 8.0f, 16.0f};  // in g
    // WHY divide by 32768: 16-bit signed = -32768 to +32767
    // So ±range maps to ±32768
    accel_scale = accel_ranges[ACCEL_RANGE] / 32768.0f;
    
    // Gyroscope: depends on range setting
    const float gyro_ranges[] = {16.0f, 32.0f, 64.0f, 128.0f, 
                                 256.0f, 512.0f, 1024.0f, 2048.0f};  // in °/s
    gyro_scale = gyro_ranges[GYRO_RANGE] / 32768.0f;
    
    Serial.print("[IMU] Accel scale: ±");
    Serial.print(accel_ranges[ACCEL_RANGE]);
    Serial.println("g");
    Serial.print("[IMU] Gyro scale: ±");
    Serial.print(gyro_ranges[GYRO_RANGE]);
    Serial.println("°/s");
    
    // STEP 10: Let sensor stabilize
    delay(100);
    
    return true;
}

bool IMU::init_ak09918c() {
    // STEP 1: Check device IDs
    // WHY two IDs: One for company (AKM), one for specific chip
    uint8_t wia1 = i2c_read_byte(AK09918C_ADDR, AK_WIA1);
    uint8_t wia2 = i2c_read_byte(AK09918C_ADDR, AK_WIA2);
    
    if (wia1 != AK_WIA1_VALUE || wia2 != AK_WIA2_VALUE) {
        Serial.print("[IMU] AK09918C wrong IDs: 0x");
        Serial.print(wia1, HEX);
        Serial.print(", 0x");
        Serial.println(wia2, HEX);
        return false;
    }
    
    // STEP 2: Soft reset
    i2c_write_byte(AK09918C_ADDR, AK_CNTL3, 0x01);
    delay(100);
    
    // STEP 3: Set to continuous measurement mode at 100Hz
    // WHY continuous: Automatically updates - don't need to trigger each read
    // WHY 100Hz: Matches our update rate well
    i2c_write_byte(AK09918C_ADDR, AK_CNTL2, AK_MODE_CONT_100HZ);
    delay(10);
    
    return true;
}

// =============================================================================
// DATA READING
// =============================================================================

bool IMU::update() {
    // Try to read both sensors
    // WHY separate: If one fails, we still get data from the other
    bool imu_success = false;
    bool mag_success = false;
    
    if (imu_ok) {
        imu_success = read_qmi8658c();
    }
    
    if (mag_ok) {
        mag_success = read_ak09918c();
    }
    
    // Calculate simple orientation if we have accel data
    if (imu_success) {
        calculate_orientation();
    }
    
    // Return true if we got at least some data
    return (imu_success || mag_success);
}

bool IMU::read_qmi8658c() {
    // STEP 1: Check if new data is available
    // WHY: Sensor updates at fixed rate - don't want to read same data twice
    uint8_t status = i2c_read_byte(QMI8658C_ADDR, QMI_STATUS0);
    
    // Bit 0 = accel ready, bit 1 = gyro ready
    // WHY check both: Make sure we have fresh data from both
    if (!(status & 0x03)) {
        return false;  // Data not ready yet
    }
    
    // STEP 2: Read all data in one burst
    // WHY: Faster and ensures all readings are from same moment
    // 14 bytes: 2 temp + 6 accel + 6 gyro
    uint8_t buffer[14];
    if (!i2c_read_bytes(QMI8658C_ADDR, QMI_TEMP_L, buffer, 14)) {
        return false;  // Read failed
    }
    
    // STEP 3: Parse temperature (bytes 0-1)
    // WHAT: Combine low and high bytes into 16-bit value
    // WHY little-endian: Low byte first, then high byte
    // Format: (high_byte << 8) | low_byte
    int16_t temp_raw = (buffer[1] << 8) | buffer[0];
    data.temperature = temp_raw / 256.0f;  // Scale: 256 LSB per °C
    
    // STEP 4: Parse accelerometer (bytes 2-7)
    // X = bytes 2-3, Y = bytes 4-5, Z = bytes 6-7
    data.accel_raw[0] = (buffer[3] << 8) | buffer[2];  // X
    data.accel_raw[1] = (buffer[5] << 8) | buffer[4];  // Y
    data.accel_raw[2] = (buffer[7] << 8) | buffer[6];  // Z
    
    // STEP 5: Parse gyroscope (bytes 8-13)
    data.gyro_raw[0] = (buffer[9] << 8) | buffer[8];   // X
    data.gyro_raw[1] = (buffer[11] << 8) | buffer[10]; // Y
    data.gyro_raw[2] = (buffer[13] << 8) | buffer[12]; // Z
    
    // STEP 6: Convert to physical units
    // WHY separate raw and converted: Keep raw for calibration,
    //     use converted for calculations
    for (int i = 0; i < 3; i++) {
        data.accel[i] = data.accel_raw[i] * accel_scale;
        data.gyro[i] = data.gyro_raw[i] * gyro_scale;
    }
    
    return true;
}

bool IMU::read_ak09918c() {
    // STEP 1: Check data ready flag
    // WHY: Magnetometer measures continuously, need to know when fresh data
    uint8_t status = i2c_read_byte(AK09918C_ADDR, AK_ST1);
    if (!(status & 0x01)) {
        return false;  // Data not ready
    }
    
    // STEP 2: Read magnetic field data (6 bytes: X, Y, Z)
    // Plus 2 more bytes: temperature and status
    uint8_t buffer[8];
    if (!i2c_read_bytes(AK09918C_ADDR, AK_HXL, buffer, 8)) {
        return false;
    }
    
    // STEP 3: Parse X, Y, Z (bytes 0-5)
    data.mag_raw[0] = (buffer[1] << 8) | buffer[0];  // X
    data.mag_raw[1] = (buffer[3] << 8) | buffer[2];  // Y  
    data.mag_raw[2] = (buffer[5] << 8) | buffer[4];  // Z
    
    // STEP 4: Convert to microtesla (µT)
    // AK09918C has fixed scale of 0.15 µT per LSB
    for (int i = 0; i < 3; i++) {
        data.mag[i] = data.mag_raw[i] * mag_scale;
    }
    
    return true;
}

// =============================================================================
// ORIENTATION CALCULATION
// =============================================================================
// WHY: Simple roll/pitch from gravity direction
// LIMITATION: Only works when STATIONARY! Motion adds acceleration that
//            corrupts the gravity measurement.

void IMU::calculate_orientation() {
    // WHAT: Accelerometer measures gravity + any motion acceleration
    // WHEN STATIONARY: Accel reads pure gravity direction
    // Gravity always points down, so accel tells us which way is "up"
    
    float ax = data.accel[0];
    float ay = data.accel[1];
    float az = data.accel[2];
    
    // Roll: Rotation around X axis (forward/back tilt)
    // FORMULA: atan2(y, z) gives angle in radians
    // WHY atan2: Handles all quadrants correctly (unlike regular atan)
    data.roll = atan2(ay, az) * 180.0f / M_PI;  // Convert to degrees
    
    // Pitch: Rotation around Y axis (left/right tilt)
    // FORMULA: atan2(-x, sqrt(y² + z²))
    // WHY sqrt: Get magnitude of Y-Z components
    data.pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0f / M_PI;
    
    // NOTE: No yaw calculated here - need magnetometer or gyro integration
    // WHY: Gravity doesn't tell us compass direction, only tilt
}
