#ifndef AK09918C_H
#define AK09918C_H

#include <Arduino.h>

// =============================================================================
// AK09918C Register Definitions
// =============================================================================

#define AK_WIA1            0x00  // Company ID
#define AK_WIA2            0x01  // Device ID
#define AK_ST1             0x10  // Status 1 (data ready)
#define AK_HXL             0x11  // X-axis data low byte
#define AK_HXH             0x12  // X-axis data high byte
#define AK_HYL             0x13  // Y-axis data low byte
#define AK_HYH             0x14  // Y-axis data high byte
#define AK_HZL             0x15  // Z-axis data low byte
#define AK_HZH             0x16  // Z-axis data high byte
#define AK_TMPS            0x17  // Temperature (dummy)
#define AK_ST2             0x18  // Status 2 (data overflow, must read to clear DRDY)
#define AK_CNTL2           0x31  // Control 2 (operation mode)
#define AK_CNTL3           0x32  // Control 3 (soft reset)

// Expected ID values
#define AK_WIA1_VALUE      0x48
#define AK_WIA2_VALUE      0x0C

// Operation modes
#define AK_MODE_POWER_DOWN 0x00
#define AK_MODE_SINGLE     0x01
#define AK_MODE_CONT_10HZ  0x02
#define AK_MODE_CONT_20HZ  0x04
#define AK_MODE_CONT_50HZ  0x06
#define AK_MODE_CONT_100HZ 0x08

// Status bits
#define AK_ST1_DRDY        0x01  // Data ready
#define AK_ST1_DOR         0x02  // Data overrun

// =============================================================================
// AK09918C Data Structure
// =============================================================================

struct AK09918C_Data {
    // Raw sensor values
    int16_t mag_raw[3];

    // Calibrated values in physical units (uT)
    float mag[3];

    // Hard iron calibration (offset)
    float mag_bias[3];

    // Soft iron calibration (scale matrix)
    float soft_iron[3][3];
};

// =============================================================================
// AK09918C Class
// =============================================================================

class AK09918C {
public:
    AK09918C(uint8_t addr = 0x0C);

    // Initialize the sensor
    bool begin(uint8_t mode = AK_MODE_CONT_100HZ);

    // Read sensor data (returns false if data not ready)
    bool read();

    // Check if new data is available
    bool data_ready();

    // Get data
    const AK09918C_Data& get_data() const { return data_; }

    // Set calibration values
    void set_hard_iron(float bx, float by, float bz);
    void set_soft_iron(const float matrix[3][3]);

private:
    uint8_t addr_;
    float scale_;  // uT per LSB
    AK09918C_Data data_;
};

#endif // AK09918C_H
