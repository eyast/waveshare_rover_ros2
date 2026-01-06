#include "ak09918c.h"
#include "config.h"
#include "i2c_helpers.h"

AK09918C mag(AK09918C_ADDR);
bool mag_ok = false;

AK09918C::AK09918C(uint8_t addr) : addr_(addr), scale_(0.15f) {
    memset(&data_, 0, sizeof(data_));
    // Initialize soft iron matrix to identity
    data_.soft_iron[0][0] = 1.0f;
    data_.soft_iron[1][1] = 1.0f;
    data_.soft_iron[2][2] = 1.0f;
    // Default axis signs (no correction)
    axis_sign_[0] = 1;
    axis_sign_[1] = 1;
    axis_sign_[2] = 1;
}

bool AK09918C::begin(uint8_t mode) {
    uint8_t wia1 = i2c_read_register(addr_, AK_WIA1);
    uint8_t wia2 = i2c_read_register(addr_, AK_WIA2);

    if (wia1 != AK_WIA1_VALUE || wia2 != AK_WIA2_VALUE) {
        Serial.print("AK09918C ID check failed: WIA1=0x");
        Serial.print(wia1, HEX);
        Serial.print(", WIA2=0x");
        Serial.println(wia2, HEX);
        return false;
    }

    // Soft reset
    i2c_write_register(addr_, AK_CNTL3, 0x01);
    delay(100);

    // Set operation mode
    i2c_write_register(addr_, AK_CNTL2, mode);
    delay(10);

    return true;
}

bool AK09918C::data_ready() {
    uint8_t st1 = i2c_read_register(addr_, AK_ST1);
    return (st1 & AK_ST1_DRDY) != 0;
}

bool AK09918C::read() {
    uint8_t st1 = i2c_read_register(addr_, AK_ST1);
    if (!(st1 & AK_ST1_DRDY)) {
        return false;
    }

    uint8_t buffer[8];
    if (!i2c_read_registers(addr_, AK_HXL, buffer, 8)) {
        return false;
    }

    // Parse raw values (little-endian)
    data_.mag_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    data_.mag_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    data_.mag_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Apply hard iron correction (offset subtraction)
    float mx = (data_.mag_raw[0] * scale_) - data_.mag_bias[0];
    float my = (data_.mag_raw[1] * scale_) - data_.mag_bias[1];
    float mz = (data_.mag_raw[2] * scale_) - data_.mag_bias[2];

    // Apply soft iron correction (matrix multiplication)
    float mx_si = data_.soft_iron[0][0] * mx + data_.soft_iron[0][1] * my + data_.soft_iron[0][2] * mz;
    float my_si = data_.soft_iron[1][0] * mx + data_.soft_iron[1][1] * my + data_.soft_iron[1][2] * mz;
    float mz_si = data_.soft_iron[2][0] * mx + data_.soft_iron[2][1] * my + data_.soft_iron[2][2] * mz;

    // Apply axis sign corrections (CRITICAL for IMU/Mag alignment!)
    data_.mag[0] = mx_si * axis_sign_[0];
    data_.mag[1] = my_si * axis_sign_[1];
    data_.mag[2] = mz_si * axis_sign_[2];

    return true;
}

void AK09918C::set_hard_iron(float bx, float by, float bz) {
    data_.mag_bias[0] = bx;
    data_.mag_bias[1] = by;
    data_.mag_bias[2] = bz;
}

void AK09918C::set_soft_iron(const float matrix[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            data_.soft_iron[i][j] = matrix[i][j];
        }
    }
}

void AK09918C::set_axis_signs(int8_t x_sign, int8_t y_sign, int8_t z_sign) {
    axis_sign_[0] = (x_sign >= 0) ? 1 : -1;
    axis_sign_[1] = (y_sign >= 0) ? 1 : -1;
    axis_sign_[2] = (z_sign >= 0) ? 1 : -1;
    
    Serial.print("Mag axis signs set: X=");
    Serial.print(axis_sign_[0]);
    Serial.print(" Y=");
    Serial.print(axis_sign_[1]);
    Serial.print(" Z=");
    Serial.println(axis_sign_[2]);
}

