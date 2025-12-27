#ifndef QMI8658C_H
#define QMI8658C_H

#include <Arduino.h>
#include "config.h"

extern bool imu_ok;

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

#define QMI_WHO_AM_I_VALUE 0x05

#define QMI_CTRL9_CMD_NOP              0x00
#define QMI_CTRL9_CMD_ON_DEMAND_CALI   0xA2
#define QMI_CTRL8_DEFAULT              0xC0

#define QMI_STATUS0_ACCEL_AVAIL        0x01
#define QMI_STATUS0_GYRO_AVAIL         0x02

// =============================================================================
// QMI8658C Data Structure
// =============================================================================

struct QMI8658C_Data {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;

    float accel[3];      // g (calibrated)
    float gyro[3];       // rad/s (calibrated, for Madgwick)
    float gyro_dps[3];   // degrees/s (calibrated, for MotionCal)
    float temperature;   // Celsius

    float accel_bias[3]; // Bias in g
    float gyro_bias[3];  // Bias in dps
};

// =============================================================================
// QMI8658C Class
// =============================================================================

class QMI8658C {
public:
    QMI8658C(uint8_t addr = 0x6B);

    bool begin(uint8_t accel_fs = 0, uint8_t gyro_fs = 5, uint8_t odr = 6);
    bool read();

    // Calibration (device must be stationary)
    void calibrate_gyro(uint16_t samples = 500);
    
    // Accelerometer calibration (device must be stationary AND level, Z-up)
    void calibrate_accel(uint16_t samples = 500);

    const QMI8658C_Data& get_data() const { return data_; }
    float get_accel_scale() const { return accel_scale_; }
    float get_gyro_scale() const { return gyro_scale_; }

private:
    uint8_t addr_;
    float accel_scale_;
    float gyro_scale_;
    QMI8658C_Data data_;
};

extern QMI8658C imu;

#endif // QMI8658C_H
