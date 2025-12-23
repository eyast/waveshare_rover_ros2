#include "qmi8658c.h"
#include "i2c_helpers.h"

QMI8658C::QMI8658C(uint8_t addr) : addr_(addr), accel_scale_(0), gyro_scale_(0) {
    memset(&data_, 0, sizeof(data_));
}

bool QMI8658C::begin(uint8_t accel_fs, uint8_t gyro_fs, uint8_t odr) {
    // Verify device ID
    uint8_t whoami = i2c_read_register(addr_, QMI_WHO_AM_I);
    if (whoami != QMI_WHO_AM_I_VALUE) {
        Serial.print("QMI8658C WHO_AM_I failed: 0x");
        Serial.println(whoami, HEX);
        return false;
    }

    // Soft reset
    i2c_write_register(addr_, QMI_RESET, 0xB0);
    delay(50);

    // CTRL1: SPI 4-wire, Address auto-increment enabled, Big-endian
    i2c_write_register(addr_, QMI_CTRL1, 0x60);

    // CTRL2: Accelerometer config (full-scale << 4 | ODR)
    i2c_write_register(addr_, QMI_CTRL2, (accel_fs << 4) | odr);

    // CTRL3: Gyroscope config (full-scale << 4 | ODR)
    i2c_write_register(addr_, QMI_CTRL3, (gyro_fs << 4) | odr);

    // CTRL5: Enable low-pass filter for accel and gyro
    i2c_write_register(addr_, QMI_CTRL5, 0x11);

    // CTRL7: Enable accelerometer and gyroscope
    i2c_write_register(addr_, QMI_CTRL7, 0x03);

    // Calculate scale factors
    const float accel_fs_values[] = {2.0f, 4.0f, 8.0f, 16.0f};
    accel_scale_ = accel_fs_values[accel_fs & 0x03] / 32768.0f;

    const float gyro_fs_values[] = {16.0f, 32.0f, 64.0f, 128.0f, 256.0f, 512.0f, 1024.0f, 2048.0f};
    gyro_scale_ = gyro_fs_values[gyro_fs & 0x07] / 32768.0f;

    delay(50);
    return true;
}

bool QMI8658C::read() {
    uint8_t buffer[14];

    // Read temperature (2 bytes) + accel (6 bytes) + gyro (6 bytes)
    if (!i2c_read_registers(addr_, QMI_TEMP_L, buffer, 14)) {
        return false;
    }

    // Parse temperature (offset 0-1)
    data_.temp_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    data_.temperature = data_.temp_raw / 256.0f;  // Convert to Celsius

    // Parse accelerometer (offset 2-7)
    data_.accel_raw[0] = (int16_t)((buffer[3] << 8) | buffer[2]);
    data_.accel_raw[1] = (int16_t)((buffer[5] << 8) | buffer[4]);
    data_.accel_raw[2] = (int16_t)((buffer[7] << 8) | buffer[6]);

    // Parse gyroscope (offset 8-13)
    data_.gyro_raw[0] = (int16_t)((buffer[9] << 8) | buffer[8]);
    data_.gyro_raw[1] = (int16_t)((buffer[11] << 8) | buffer[10]);
    data_.gyro_raw[2] = (int16_t)((buffer[13] << 8) | buffer[12]);

    // Convert to physical units with bias correction
    for (int i = 0; i < 3; i++) {
        data_.accel[i] = (data_.accel_raw[i] * accel_scale_) - data_.accel_bias[i];
        // Gyro in dps (for MotionCal)
        data_.gyro_dps[i] = (data_.gyro_raw[i] * gyro_scale_) - data_.gyro_bias[i];
        // Gyro in rad/s (for Madgwick filter)
        data_.gyro[i] = data_.gyro_dps[i] * (PI / 180.0f);
    }

    return true;
}

void QMI8658C::calibrate_gyro(uint16_t samples) {
    Serial.println("Calibrating gyro - keep still...");

    float sum[3] = {0, 0, 0};

    for (uint16_t i = 0; i < samples; i++) {
        if (read()) {
            // Sum raw values converted to dps (before bias)
            for (int j = 0; j < 3; j++) {
                sum[j] += data_.gyro_raw[j] * gyro_scale_;
            }
        }
        delay(2);
    }

    // Calculate average bias
    for (int i = 0; i < 3; i++) {
        data_.gyro_bias[i] = sum[i] / samples;
    }

    Serial.print("Gyro bias: ");
    Serial.print(data_.gyro_bias[0], 4);
    Serial.print(", ");
    Serial.print(data_.gyro_bias[1], 4);
    Serial.print(", ");
    Serial.println(data_.gyro_bias[2], 4);
    Serial.println("Gyro calibration done");
}
