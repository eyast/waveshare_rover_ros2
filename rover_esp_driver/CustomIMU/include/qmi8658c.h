#ifndef QMI8658C_H
#define QMI8658C_H

#include <Arduino.h>

// =============================================================================
// QMI8658C Register Definitions
// =============================================================================

#define QMI_WHO_AM_I       0x00
#define QMI_REVISION_ID    0x01
#define QMI_CTRL1          0x02
#define QMI_CTRL2          0x03
#define QMI_CTRL3          0x04
#define QMI_CTRL4          0x05
#define QMI_CTRL5          0x06
#define QMI_CTRL6          0x07
#define QMI_CTRL7          0x08
#define QMI_CTRL8          0x09
#define QMI_CTRL9          0x0A
#define QMI_STATUS0        0x2E
#define QMI_STATUS1        0x2F
#define QMI_TEMP_L         0x33
#define QMI_TEMP_H         0x34
#define QMI_AX_L           0x35
#define QMI_RESET          0x60

// Expected WHO_AM_I value
#define QMI_WHO_AM_I_VALUE 0x05

// =============================================================================
// QMI8658C Data Structure
// =============================================================================

struct QMI8658C_Data {
    // Raw sensor values
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;

    // Calibrated values in physical units
    float accel[3];      // g
    float gyro[3];       // rad/s (for Madgwick filter)
    float gyro_dps[3];   // degrees/s (for MotionCal output)
    float temperature;   // Celsius

    // Calibration biases
    float accel_bias[3];
    float gyro_bias[3];
};

// =============================================================================
// QMI8658C Class
// =============================================================================

class QMI8658C {
public:
    QMI8658C(uint8_t addr = 0x6B);

    // Initialize the sensor with given configuration
    bool begin(uint8_t accel_fs = 0, uint8_t gyro_fs = 5, uint8_t odr = 6);

    // Read sensor data
    bool read();

    // Calibrate gyroscope (device must be stationary)
    void calibrate_gyro(uint16_t samples = 500);

    // Get data
    const QMI8658C_Data& get_data() const { return data_; }

    // Get scale factors
    float get_accel_scale() const { return accel_scale_; }
    float get_gyro_scale() const { return gyro_scale_; }

private:
    uint8_t addr_;
    float accel_scale_;
    float gyro_scale_;
    QMI8658C_Data data_;
};

#endif // QMI8658C_H
