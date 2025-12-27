#ifndef AK09918C_H
#define AK09918C_H

#include <Arduino.h>

extern bool mag_ok;

// =============================================================================
// AK09918C Register Definitions
// =============================================================================

#define AK_WIA1            0x00
#define AK_WIA2            0x01
#define AK_ST1             0x10
#define AK_HXL             0x11
#define AK_HXH             0x12
#define AK_HYL             0x13
#define AK_HYH             0x14
#define AK_HZL             0x15
#define AK_HZH             0x16
#define AK_TMPS            0x17
#define AK_ST2             0x18
#define AK_CNTL2           0x31
#define AK_CNTL3           0x32

#define AK_WIA1_VALUE      0x48
#define AK_WIA2_VALUE      0x0C

#define AK_MODE_POWER_DOWN 0x00
#define AK_MODE_SINGLE     0x01
#define AK_MODE_CONT_10HZ  0x02
#define AK_MODE_CONT_20HZ  0x04
#define AK_MODE_CONT_50HZ  0x06
#define AK_MODE_CONT_100HZ 0x08

#define AK_ST1_DRDY        0x01
#define AK_ST1_DOR         0x02

// =============================================================================
// AK09918C Data Structure
// =============================================================================

struct AK09918C_Data {
    int16_t mag_raw[3];
    float mag[3];           // Calibrated values in µT
    float mag_bias[3];      // Hard iron offsets
    float soft_iron[3][3];  // Soft iron matrix
};

// =============================================================================
// AK09918C Class
// =============================================================================

class AK09918C {
public:
    AK09918C(uint8_t addr = 0x0C);

    bool begin(uint8_t mode = AK_MODE_CONT_100HZ);
    bool read();
    bool data_ready();

    const AK09918C_Data& get_data() const { return data_; }

    // Calibration
    void set_hard_iron(float bx, float by, float bz);
    void set_soft_iron(const float matrix[3][3]);
    
    // Axis alignment correction (set to -1 to invert an axis)
    // CRITICAL: Use this to align magnetometer axes with IMU axes!
    void set_axis_signs(int8_t x_sign, int8_t y_sign, int8_t z_sign);

private:
    uint8_t addr_;
    float scale_;
    AK09918C_Data data_;
    
    // Axis sign corrections (default: no correction)
    int8_t axis_sign_[3] = {1, 1, 1};
};

extern AK09918C mag;

#endif // AK09918C_H
