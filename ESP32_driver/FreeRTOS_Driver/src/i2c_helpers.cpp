/*
 * I2C Helper Functions Implementation
 */

#include "i2c_helpers.h"

void i2c_init(uint8_t sda, uint8_t scl, uint32_t freq) {
    Wire.begin(sda, scl);
    Wire.setClock(freq);
}

bool i2c_write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

uint8_t i2c_read_register(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0xFF;  // Return error value if no data
}

bool i2c_read_registers(uint8_t addr, uint8_t start_reg, uint8_t* buffer, uint8_t count) {
    Wire.beginTransmission(addr);
    Wire.write(start_reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    uint8_t received = Wire.requestFrom(addr, count);
    if (received < count) {
        return false;
    }

    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = Wire.read();
    }
    return true;
}
