#include "qmi8658c.h"
#include "i2c_helpers.h"
#include "config.h"

QMI8658C imu(QMI8658C_ADDR);
bool imu_ok = false;

QMI8658C::QMI8658C(uint8_t addr) : addr_(addr), accel_scale_(0), gyro_scale_(0) {
    memset(&data_, 0, sizeof(data_));
}

bool QMI8658C::begin(uint8_t accel_fs, uint8_t gyro_fs, uint8_t odr) {
    uint8_t whoami = i2c_read_register(addr_, QMI_WHO_AM_I);
    if (whoami != QMI_WHO_AM_I_VALUE) {
        Serial.print("QMI8658C WHO_AM_I failed: 0x");
        Serial.println(whoami, HEX);
        return false;
    }

    // On-demand calibration (hardware gyro bias calibration)
    Serial.println("QMI8658C: Starting on-demand calibration...");
    i2c_write_register(addr_, QMI_RESET, 0xB0);
    delay(10);
    i2c_write_register(addr_, QMI_CTRL9, QMI_CTRL9_CMD_ON_DEMAND_CALI);
    delay(3200);
    i2c_write_register(addr_, QMI_CTRL9, QMI_CTRL9_CMD_NOP);
    delay(100);
    Serial.println("QMI8658C: On-demand calibration done");

    // CTRL1: Address auto-increment, Big-endian
    i2c_write_register(addr_, QMI_CTRL1, 0x60 | 0x18);

    // CTRL7: Disable sensors before config
    i2c_write_register(addr_, QMI_CTRL7, 0x00);

    // CTRL8: Motion detection settings
    i2c_write_register(addr_, QMI_CTRL8, QMI_CTRL8_DEFAULT);

    // CTRL2: Accelerometer config
    i2c_write_register(addr_, QMI_CTRL2, (accel_fs << 4) | odr);

    // CTRL3: Gyroscope config
    i2c_write_register(addr_, QMI_CTRL3, (gyro_fs << 4) | odr);

    // CTRL5: Low-pass filters enabled
    i2c_write_register(addr_, QMI_CTRL5, 0x11);

    // CTRL7: Enable accelerometer and gyroscope
    i2c_write_register(addr_, QMI_CTRL7, 0x03);

    // Calculate scale factors
    const float accel_fs_values[] = {2.0f, 4.0f, 8.0f, 16.0f};
    accel_scale_ = accel_fs_values[accel_fs & 0x03] / 32768.0f;

    const float gyro_fs_values[] = {16.0f, 32.0f, 64.0f, 128.0f, 256.0f, 512.0f, 1024.0f, 2048.0f};
    gyro_scale_ = gyro_fs_values[gyro_fs & 0x07] / 32768.0f;

    // Let sensor stabilize
    delay(100);
    
    // Flush initial readings
    for (int i = 0; i < 20; i++) {
        read();
        delay(5);
    }

    return true;
}

bool QMI8658C::read() {
    uint8_t status = i2c_read_register(addr_, QMI_STATUS0);
    if (!(status & (QMI_STATUS0_ACCEL_AVAIL | QMI_STATUS0_GYRO_AVAIL))) {
        return false;
    }

    uint8_t buffer[14];
    if (!i2c_read_registers(addr_, QMI_TEMP_L, buffer, 14)) {
        return false;
    }

    // Parse temperature
    data_.temp_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    data_.temperature = data_.temp_raw / 256.0f;

    // Parse accelerometer
    data_.accel_raw[0] = (int16_t)((buffer[3] << 8) | buffer[2]);
    data_.accel_raw[1] = (int16_t)((buffer[5] << 8) | buffer[4]);
    data_.accel_raw[2] = (int16_t)((buffer[7] << 8) | buffer[6]);

    // Parse gyroscope
    data_.gyro_raw[0] = (int16_t)((buffer[9] << 8) | buffer[8]);
    data_.gyro_raw[1] = (int16_t)((buffer[11] << 8) | buffer[10]);
    data_.gyro_raw[2] = (int16_t)((buffer[13] << 8) | buffer[12]);

    // Convert to physical units with bias correction
    for (int i = 0; i < 3; i++) {
        data_.accel[i] = (data_.accel_raw[i] * accel_scale_) - data_.accel_bias[i];
        data_.gyro_dps[i] = (data_.gyro_raw[i] * gyro_scale_) - data_.gyro_bias[i];
        data_.gyro[i] = data_.gyro_dps[i] * (PI / 180.0f);
    }

    return true;
}

void QMI8658C::calibrate_gyro(uint16_t samples) {
    Serial.println("\n=== Gyroscope Calibration ===");
    Serial.println("Keep device STATIONARY...");
    delay(500);

    // Temporarily clear bias to measure raw values
    float old_bias[3];
    for (int i = 0; i < 3; i++) {
        old_bias[i] = data_.gyro_bias[i];
        data_.gyro_bias[i] = 0;
    }

    double sum[3] = {0, 0, 0};
    double sum_sq[3] = {0, 0, 0};
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; i++) {
        // Wait for data
        uint32_t start = millis();
        while (!read() && (millis() - start < 50)) {
            delayMicroseconds(500);
        }

        for (int j = 0; j < 3; j++) {
            float val = data_.gyro_raw[j] * gyro_scale_;
            sum[j] += val;
            sum_sq[j] += val * val;
        }
        valid++;

        if (i % 100 == 0) Serial.print(".");
        delay(2);
    }
    Serial.println(" Done!");

    if (valid < samples / 2) {
        Serial.println("WARNING: Too few samples, keeping old bias");
        for (int i = 0; i < 3; i++) {
            data_.gyro_bias[i] = old_bias[i];
        }
        return;
    }

    // Calculate bias (mean)
    Serial.println("Gyro calibration results:");
    for (int i = 0; i < 3; i++) {
        data_.gyro_bias[i] = sum[i] / valid;
        float variance = (sum_sq[i] / valid) - (data_.gyro_bias[i] * data_.gyro_bias[i]);
        float noise = sqrtf(variance > 0 ? variance : 0);
        
        Serial.print("  "); Serial.print("XYZ"[i]);
        Serial.print(": bias="); Serial.print(data_.gyro_bias[i], 4);
        Serial.print(" dps, noise="); Serial.print(noise, 4);
        Serial.println(" dps");
    }
}

void QMI8658C::calibrate_accel(uint16_t samples) {
    Serial.println("\n=== Accelerometer Calibration ===");
    Serial.println("Keep device STATIONARY and LEVEL (Z-up)...");
    delay(500);

    // Temporarily clear bias
    float old_bias[3];
    for (int i = 0; i < 3; i++) {
        old_bias[i] = data_.accel_bias[i];
        data_.accel_bias[i] = 0;
    }

    double sum[3] = {0, 0, 0};
    double sum_sq[3] = {0, 0, 0};
    uint16_t valid = 0;

    for (uint16_t i = 0; i < samples; i++) {
        uint32_t start = millis();
        while (!read() && (millis() - start < 50)) {
            delayMicroseconds(500);
        }

        for (int j = 0; j < 3; j++) {
            float val = data_.accel_raw[j] * accel_scale_;
            sum[j] += val;
            sum_sq[j] += val * val;
        }
        valid++;

        if (i % 100 == 0) Serial.print(".");
        delay(2);
    }
    Serial.println(" Done!");

    if (valid < samples / 2) {
        Serial.println("WARNING: Too few samples, keeping old bias");
        for (int i = 0; i < 3; i++) {
            data_.accel_bias[i] = old_bias[i];
        }
        return;
    }

    // Calculate mean
    float mean[3];
    for (int i = 0; i < 3; i++) {
        mean[i] = sum[i] / valid;
    }

    // For level device with Z-up: expected ax=0, ay=0, az=+1g
    // Bias = measured - expected
    data_.accel_bias[0] = mean[0] - 0.0f;
    data_.accel_bias[1] = mean[1] - 0.0f;
    data_.accel_bias[2] = mean[2] - 1.0f;

    float magnitude = sqrtf(mean[0]*mean[0] + mean[1]*mean[1] + mean[2]*mean[2]);

    Serial.println("Accel calibration results:");
    Serial.print("  Raw mean (g): X="); Serial.print(mean[0], 4);
    Serial.print(" Y="); Serial.print(mean[1], 4);
    Serial.print(" Z="); Serial.println(mean[2], 4);
    Serial.print("  Magnitude: "); Serial.print(magnitude, 4);
    Serial.println(" (should be 1.0)");
    
    Serial.print("  Bias (g): X="); Serial.print(data_.accel_bias[0], 4);
    Serial.print(" Y="); Serial.print(data_.accel_bias[1], 4);
    Serial.print(" Z="); Serial.println(data_.accel_bias[2], 4);

    for (int i = 0; i < 3; i++) {
        float variance = (sum_sq[i] / valid) - (mean[i] * mean[i]);
        float noise = sqrtf(variance > 0 ? variance : 0);
        Serial.print("  Noise "); Serial.print("XYZ"[i]);
        Serial.print(": "); Serial.print(noise, 6); Serial.println(" g");
    }
}
