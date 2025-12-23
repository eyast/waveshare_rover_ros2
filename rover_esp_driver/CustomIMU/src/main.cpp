#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/**
 * =============================================================================
 * QMI8658C + AK09918C IMU Driver - MotionCal Compatible
 * =============================================================================
 * 
 * Outputs data in PJRC MotionCal format:
 *   - ASCII: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r" (for calibration)
 *   - Binary: 0x7E framed packets with quaternion (for orientation display)
 * 
 * =============================================================================
 */

// =============================================================================
// CONFIGURATION
// =============================================================================

// I2C pins (adjust for your board)
#define I2C_SDA 32
#define I2C_SCL 33
#define I2C_FREQ 400000

// Serial - MotionCal typically uses 115200
#define SERIAL_BAUD 115200

// I2C Addresses
#define QMI8658C_ADDR 0x6B
#define AK09918C_ADDR 0x0C

// Sensor Configuration
#define ACCEL_FS_SEL  0     // ±2g
#define GYRO_FS_SEL   5     // ±512dps
#define ODR_SEL       6     // 117.5 Hz

// Output rate (Hz) - MotionCal works well at 20-50 Hz
#define OUTPUT_RATE_HZ 50
#define OUTPUT_INTERVAL_MS (1000 / OUTPUT_RATE_HZ)

// Madgwick filter
#define MADGWICK_BETA 0.05f

// =============================================================================
// QMI8658C REGISTERS
// =============================================================================

#define QMI_WHO_AM_I       0x00
#define QMI_CTRL1          0x02
#define QMI_CTRL2          0x03
#define QMI_CTRL3          0x04
#define QMI_CTRL5          0x06
#define QMI_CTRL7          0x08
#define QMI_TEMP_L         0x33
#define QMI_RESET          0x60

// =============================================================================
// AK09918C REGISTERS
// =============================================================================

#define AK_WIA1            0x00
#define AK_WIA2            0x01
#define AK_ST1             0x10
#define AK_HXL             0x11
#define AK_CNTL2           0x31
#define AK_CNTL3           0x32
#define AK_MODE_CONT_100HZ 0x08

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Scale factors
float accelScale;
float gyroScale;
const float magScale = 0.15f;  // µT/LSB

// Calibration (will be updated by MotionCal)
float accelBias[3] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0};
float magBias[3] = {0, 0, 0};
float magSoftIron[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};

// Raw sensor data (int16)
int16_t accelRaw[3], gyroRaw[3], magRaw[3];
int16_t tempRaw;

// Calibrated data
float accel[3], gyro[3], mag[3];

// Madgwick quaternion
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Timing
uint32_t lastOutputMs = 0;
uint32_t lastUpdateMicros = 0;

// =============================================================================
// I2C HELPERS
// =============================================================================

bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t readRegister(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.read();
}

bool readRegisters(uint8_t addr, uint8_t startReg, uint8_t* buffer, uint8_t count) {
    Wire.beginTransmission(addr);
    Wire.write(startReg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(addr, count);
    if (Wire.available() < count) return false;
    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}

// =============================================================================
// SENSOR INITIALIZATION
// =============================================================================

bool initQMI8658C() {
    uint8_t whoami = readRegister(QMI8658C_ADDR, QMI_WHO_AM_I);
    if (whoami != 0x05) {
        Serial.print("QMI8658C WHO_AM_I failed: 0x");
        Serial.println(whoami, HEX);
        return false;
    }
    
    // Soft reset
    writeRegister(QMI8658C_ADDR, QMI_RESET, 0xB0);
    delay(50);
    
    // CTRL1: Auto-increment, big-endian
    writeRegister(QMI8658C_ADDR, QMI_CTRL1, 0x60);
    
    // CTRL2: Accelerometer config
    writeRegister(QMI8658C_ADDR, QMI_CTRL2, (ACCEL_FS_SEL << 4) | ODR_SEL);
    
    // CTRL3: Gyroscope config
    writeRegister(QMI8658C_ADDR, QMI_CTRL3, (GYRO_FS_SEL << 4) | ODR_SEL);
    
    // CTRL5: Low pass filter
    writeRegister(QMI8658C_ADDR, QMI_CTRL5, 0x11);
    
    // CTRL7: Enable accel + gyro
    writeRegister(QMI8658C_ADDR, QMI_CTRL7, 0x03);
    
    // Calculate scales
    const float accelFS[] = {2.0f, 4.0f, 8.0f, 16.0f};
    accelScale = accelFS[ACCEL_FS_SEL] / 32768.0f;
    
    const float gyroFS[] = {16.0f, 32.0f, 64.0f, 128.0f, 256.0f, 512.0f, 1024.0f, 2048.0f};
    gyroScale = gyroFS[GYRO_FS_SEL] / 32768.0f;
    
    delay(50);
    return true;
}

bool initAK09918C() {
    uint8_t wia1 = readRegister(AK09918C_ADDR, AK_WIA1);
    uint8_t wia2 = readRegister(AK09918C_ADDR, AK_WIA2);
    
    if (wia1 != 0x48 || wia2 != 0x0C) {
        Serial.println("AK09918C ID check failed");
        return false;
    }
    
    // Soft reset
    writeRegister(AK09918C_ADDR, AK_CNTL3, 0x01);
    delay(100);
    
    // Continuous mode at 100 Hz
    writeRegister(AK09918C_ADDR, AK_CNTL2, AK_MODE_CONT_100HZ);
    delay(10);
    
    return true;
}

// =============================================================================
// SENSOR READING
// =============================================================================

bool readIMUData() {
    uint8_t buffer[14];
    
    if (!readRegisters(QMI8658C_ADDR, QMI_TEMP_L, buffer, 14)) {
        return false;
    }
    
    tempRaw = (int16_t)((buffer[1] << 8) | buffer[0]);
    
    accelRaw[0] = (int16_t)((buffer[3] << 8) | buffer[2]);
    accelRaw[1] = (int16_t)((buffer[5] << 8) | buffer[4]);
    accelRaw[2] = (int16_t)((buffer[7] << 8) | buffer[6]);
    
    gyroRaw[0] = (int16_t)((buffer[9] << 8) | buffer[8]);
    gyroRaw[1] = (int16_t)((buffer[11] << 8) | buffer[10]);
    gyroRaw[2] = (int16_t)((buffer[13] << 8) | buffer[12]);
    
    // Convert to physical units
    accel[0] = (accelRaw[0] * accelScale) - accelBias[0];
    accel[1] = (accelRaw[1] * accelScale) - accelBias[1];
    accel[2] = (accelRaw[2] * accelScale) - accelBias[2];
    
    gyro[0] = ((gyroRaw[0] * gyroScale) - gyroBias[0]) * (PI / 180.0f);
    gyro[1] = ((gyroRaw[1] * gyroScale) - gyroBias[1]) * (PI / 180.0f);
    gyro[2] = ((gyroRaw[2] * gyroScale) - gyroBias[2]) * (PI / 180.0f);
    
    return true;
}

bool readMagData() {
    uint8_t st1 = readRegister(AK09918C_ADDR, AK_ST1);
    if (!(st1 & 0x01)) {
        return false;  // Data not ready
    }

    // Read 8 bytes: HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2
    // Must read through ST2 (0x18) to clear DRDY and enable next measurement
    uint8_t buffer[8];
    if (!readRegisters(AK09918C_ADDR, AK_HXL, buffer, 8)) {
        Serial.println("MAG DEBUG: Failed to read HXL registers");
        return false;
    }

    magRaw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    magRaw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    magRaw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    // buffer[6] = TMPS (temperature), buffer[7] = ST2 (status 2, clears DRDY)
    
    // Apply calibration (soft iron matrix * (raw - hard iron offset))
    float mx = (magRaw[0] * magScale) - magBias[0];
    float my = (magRaw[1] * magScale) - magBias[1];
    float mz = (magRaw[2] * magScale) - magBias[2];
    
    mag[0] = magSoftIron[0][0]*mx + magSoftIron[0][1]*my + magSoftIron[0][2]*mz;
    mag[1] = magSoftIron[1][0]*mx + magSoftIron[1][1]*my + magSoftIron[1][2]*mz;
    mag[2] = magSoftIron[2][0]*mx + magSoftIron[2][1]*my + magSoftIron[2][2]*mz;
    
    return true;
}

// =============================================================================
// MADGWICK FILTER
// =============================================================================

void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az,
                    float mx, float my, float mz,
                    float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        recipNorm = 1.0f / sqrtf(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        s0 = -_2q2 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2 * q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2 * q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2 * q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0 * q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2 * q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        qDot1 -= MADGWICK_BETA * s0;
        qDot2 -= MADGWICK_BETA * s1;
        qDot3 -= MADGWICK_BETA * s2;
        qDot4 -= MADGWICK_BETA * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 * recipNorm;
    q[1] = q1 * recipNorm;
    q[2] = q2 * recipNorm;
    q[3] = q3 * recipNorm;
}

// =============================================================================
// MOTIONCAL OUTPUT FUNCTIONS
// =============================================================================

/**
 * Send raw sensor data in MotionCal ASCII format
 * Format: "Raw:ax,ay,az,gx,gy,gz,mx,my,mz\r"
 * 
 * Note: MotionCal expects raw integer values from the sensors
 */
void sendRawData() {
    Serial.print("Raw:");
    Serial.print(accelRaw[0]); Serial.print(",");
    Serial.print(accelRaw[1]); Serial.print(",");
    Serial.print(accelRaw[2]); Serial.print(",");
    Serial.print(gyroRaw[0]); Serial.print(",");
    Serial.print(gyroRaw[1]); Serial.print(",");
    Serial.print(gyroRaw[2]); Serial.print(",");
    Serial.print(magRaw[0]); Serial.print(",");
    Serial.print(magRaw[1]); Serial.print(",");
    Serial.print(magRaw[2]);
    Serial.print("\r\n");
}

/**
 * Send a byte with 0x7D escape encoding if needed
 */
void sendEncodedByte(uint8_t b) {
    if (b == 0x7E) {
        Serial.write(0x7D);
        Serial.write(0x5E);
    } else if (b == 0x7D) {
        Serial.write(0x7D);
        Serial.write(0x5D);
    } else {
        Serial.write(b);
    }
}

/**
 * Send quaternion in MotionCal binary packet format
 * 
 * Packet structure (34 bytes, type 1):
 *   Byte 0: Packet type (0x01)
 *   Bytes 1-23: Reserved/other data
 *   Bytes 24-25: q0 (int16, scaled by 30000)
 *   Bytes 26-27: q1 (int16, scaled by 30000)
 *   Bytes 28-29: q2 (int16, scaled by 30000)
 *   Bytes 30-31: q3 (int16, scaled by 30000)
 *   Bytes 32-33: Reserved
 * 
 * Framed with 0x7E start/end, escaped with 0x7D
 */
void sendQuaternionPacket() {
    uint8_t packet[34];
    memset(packet, 0, sizeof(packet));
    
    // Packet type
    packet[0] = 0x01;
    
    // Convert quaternion to int16 scaled by 30000
    int16_t q0_i = (int16_t)(q[0] * 30000.0f);
    int16_t q1_i = (int16_t)(q[1] * 30000.0f);
    int16_t q2_i = (int16_t)(q[2] * 30000.0f);
    int16_t q3_i = (int16_t)(q[3] * 30000.0f);
    
    // Pack quaternion at offset 24 (little-endian)
    packet[24] = q0_i & 0xFF;
    packet[25] = (q0_i >> 8) & 0xFF;
    packet[26] = q1_i & 0xFF;
    packet[27] = (q1_i >> 8) & 0xFF;
    packet[28] = q2_i & 0xFF;
    packet[29] = (q2_i >> 8) & 0xFF;
    packet[30] = q3_i & 0xFF;
    packet[31] = (q3_i >> 8) & 0xFF;
    
    // Send with framing
    Serial.write(0x7E);  // Start frame
    for (int i = 0; i < 34; i++) {
        sendEncodedByte(packet[i]);
    }
    Serial.write(0x7E);  // End frame
}

/**
 * Send magnetic calibration data packet (type 6)
 * This sends the current magnetometer reading as calibration point
 */
void sendMagCalPacket(int16_t id, int16_t x, int16_t y, int16_t z) {
    uint8_t packet[14];
    memset(packet, 0, sizeof(packet));
    
    // Packet type
    packet[0] = 0x06;
    
    // ID at offset 6
    packet[6] = id & 0xFF;
    packet[7] = (id >> 8) & 0xFF;
    
    // XYZ at offset 8
    packet[8] = x & 0xFF;
    packet[9] = (x >> 8) & 0xFF;
    packet[10] = y & 0xFF;
    packet[11] = (y >> 8) & 0xFF;
    packet[12] = z & 0xFF;
    packet[13] = (z >> 8) & 0xFF;
    
    // Send with framing
    Serial.write(0x7E);
    for (int i = 0; i < 14; i++) {
        sendEncodedByte(packet[i]);
    }
    Serial.write(0x7E);
}

// =============================================================================
// GYRO CALIBRATION
// =============================================================================

void calibrateGyro(uint16_t samples = 500) {
    Serial.println("Calibrating gyro - keep still...");
    
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        readIMUData();
        sumX += gyroRaw[0] * gyroScale;
        sumY += gyroRaw[1] * gyroScale;
        sumZ += gyroRaw[2] * gyroScale;
        delay(2);
    }
    
    gyroBias[0] = sumX / samples;
    gyroBias[1] = sumY / samples;
    gyroBias[2] = sumZ / samples;
    
    Serial.println("Gyro calibration done");
}

// =============================================================================
// COMMAND PROCESSING
// =============================================================================

void processSerialCommands() {
    static char cmdBuf[64];
    static uint8_t cmdIdx = 0;
    
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';
                
                // Process command
                if (strncmp(cmdBuf, "Cal1:", 5) == 0) {
                    // Parse Cal1 data from MotionCal
                    // Format: Cal1:ax,ay,az,mx,my,mz,s00,s01,s02,s10
                    // (This is calibration feedback from MotionCal)
                    Serial.println("Cal1 received");
                } else if (strncmp(cmdBuf, "Cal2:", 5) == 0) {
                    // Parse Cal2 data from MotionCal
                    Serial.println("Cal2 received");
                }
                
                cmdIdx = 0;
            }
        } else if (cmdIdx < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }
}

// =============================================================================
// SETUP & LOOP
// =============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);
    
    Serial.println("\n========================================");
    Serial.println("IMU Driver - MotionCal Compatible");
    Serial.println("========================================\n");
    
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    
    if (!initQMI8658C()) {
        Serial.println("ERROR: QMI8658C init failed!");
    } else {
        Serial.println("QMI8658C OK");
    }
    
    if (!initAK09918C()) {
        Serial.println("ERROR: AK09918C init failed!");
    } else {
        Serial.println("AK09918C OK");
    }
    
    calibrateGyro();
    
    Serial.println("\nStreaming data for MotionCal...\n");
    
    lastUpdateMicros = micros();
    lastOutputMs = millis();
}

void loop() {
    // Process any incoming commands
    processSerialCommands();
    
    // Read sensors
    bool imuOk = readIMUData();
    bool magOk = readMagData();
    
    if (imuOk) {
        // Calculate dt
        uint32_t now = micros();
        float dt = (now - lastUpdateMicros) / 1000000.0f;
        lastUpdateMicros = now;
        
        if (dt <= 0 || dt > 0.1f) {
            dt = 0.01f;
        }
        
        // Update filter
        if (magOk) {
            madgwickUpdate(gyro[0], gyro[1], gyro[2],
                          accel[0], accel[1], accel[2],
                          mag[0], mag[1], mag[2],
                          dt);
        }
    }
    
    // Output at fixed rate
    uint32_t nowMs = millis();
    if (nowMs - lastOutputMs >= OUTPUT_INTERVAL_MS) {
        lastOutputMs = nowMs;
        
        // Send raw data (ASCII) - for MotionCal calibration
        sendRawData();
        
        // Note: Binary quaternion packets removed - the ASCII Raw format
        // is what MotionCal needs for magnetometer calibration
    }
}
