#include "ak09918c.h"
#include "i2c_helpers.h"

AK09918C::AK09918C(uint8_t addr) : addr_(addr), scale_(0.15f) {
    memset(&data_, 0, sizeof(data_));
    // Initialize soft iron matrix to identity
    data_.soft_iron[0][0] = 1.0f;
    data_.soft_iron[1][1] = 1.0f;
    data_.soft_iron[2][2] = 1.0f;
}

bool AK09918C::begin(uint8_t mode) {
    // Verify device ID
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
    // Check if data is ready
    uint8_t st1 = i2c_read_register(addr_, AK_ST1);
    if (!(st1 & AK_ST1_DRDY)) {
        return false;  // Data not ready
    }

    // Read 8 bytes: HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2
    // IMPORTANT: Must read through ST2 to clear DRDY and enable next measurement
    uint8_t buffer[8];
    if (!i2c_read_registers(addr_, AK_HXL, buffer, 8)) {
        return false;
    }

    // Parse raw values (little-endian)
    data_.mag_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    data_.mag_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    data_.mag_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    // buffer[6] = TMPS (temperature, dummy on AK09918C)
    // buffer[7] = ST2 (reading this clears DRDY)

    // Apply hard iron correction (offset)
    float mx = (data_.mag_raw[0] * scale_) - data_.mag_bias[0];
    float my = (data_.mag_raw[1] * scale_) - data_.mag_bias[1];
    float mz = (data_.mag_raw[2] * scale_) - data_.mag_bias[2];

    // Apply soft iron correction (scale matrix)
    data_.mag[0] = data_.soft_iron[0][0] * mx + data_.soft_iron[0][1] * my + data_.soft_iron[0][2] * mz;
    data_.mag[1] = data_.soft_iron[1][0] * mx + data_.soft_iron[1][1] * my + data_.soft_iron[1][2] * mz;
    data_.mag[2] = data_.soft_iron[2][0] * mx + data_.soft_iron[2][1] * my + data_.soft_iron[2][2] * mz;

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
