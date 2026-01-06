#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

struct IMU_DATA {
    int16_t accel_raw[3]; // from QMI8658C
    int16_t gyro_raw[3]; // from QMI8658C
    int16_t mag_raw[3];
    int16_t dq[4]; // changes in quaternion (with AttitudeEngine)
    int16_t dv[3]; // changes in velocity

    float temperature;
    // Converted to physical units
    // WHY convert: Real units are meaningful (1g, 90°/s, 50µT)
    float accel[3];         // X, Y, Z in g (gravity units)
    float gyro[3];          // X, Y, Z in degrees/second
    float mag[3];           // X, Y, Z in microtesla (µT)
};

class IMU{
public:
    IMU();
    bool begin();
    bool update();

private:
    IMU_DATA data;
    bool imu_ok;
    bool mag_ok;
    bool init_qmi8658c(bool ae_enabled);
    bool qmi_soft_reset();
    bool qmi_who_am_i() const;
    bool read_qmi8658c();

    // Conversion scales (calculated during initialization)
    float accel_scale;      // Multiply raw value to get g
    float gyro_scale;       // Multiply raw value to get °/s
    float mag_scale;        // Multiply raw value to get µT
};

void print_imu_data(IMU_DATA data);

extern IMU imu;


#endif