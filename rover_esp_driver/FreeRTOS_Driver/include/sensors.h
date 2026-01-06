/*
 * Sensors Module
 * 
 * Unified driver for all sensors:
 *   - QMI8658C (6-axis IMU: accelerometer + gyroscope)
 *   - AK09918C (3-axis magnetometer)
 *   - INA219 (power monitor)
 * 
 * Each sensor has its own data structure and driver class.
 * This module provides high-level access to all sensor data.
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "config.h"

// =============================================================================
// QMI8658C IMU - Accelerometer & Gyroscope
// =============================================================================

// Register definitions
#define QMI_WHO_AM_I        0x00
#define QMI_CTRL1           0x02
#define QMI_CTRL2           0x03
#define QMI_CTRL3           0x04
#define QMI_CTRL5           0x06
#define QMI_CTRL7           0x08
#define QMI_CTRL9           0x0A
#define QMI_STATUSINT       0x2D
#define QMI_STATUS0         0x2E
#define QMI_STATUS1         0x2F
#define QMI_TEMP_L          0x33
#define QMI_RESULTS         0x51
#define QMI_RESET           0x60

#define QMI_WHO_AM_I_VALUE  0x05
#define QMI_CMD_NOP         0x00
#define QMI_CTRL9_CMD_CALI  0xA2
//#define QMI_CTRL8_DEFAULT   0xC0
#define QMI_STATUS0_AVAIL   0x03

struct IMU_Data {
    // Raw sensor values
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;
    
    // Calibrated physical units
    float accel[3];         // g (gravity)
    float gyro[3];          // rad/s (for Madgwick filter)
    float gyro_dps[3];      // deg/s (for display/MotionCal)
    float temperature;      // Celsius
    
    // Calibration offsets
    float accel_bias[3];
    float accel_calib_matrix[3][3];    
    float gyro_bias[3];     
};

class QMI8658C {
public:
    QMI8658C(uint8_t addr = ADDR_QMI8658C);
    
    bool begin(uint8_t accel_fs = IMU_ACCEL_FS,
               uint8_t gyro_fs = IMU_GYRO_FS,
               uint8_t odr = IMU_ODR);
    bool read();
    const IMU_Data& data() const { return data_; }
    bool is_ok() const { return ok_; }
    void set_accel_calib(float bx,
                         float by,
                         float bz,
                         float acc_calib_matrix[3][3]);

private:
    uint8_t addr_;
    float accel_scale_;
    float gyro_scale_;
    IMU_Data data_;
    bool ok_;
};

// =============================================================================
// AK09918C Magnetometer
// =============================================================================

// Register definitions
#define AK_WIA1             0x00
#define AK_WIA2             0x01
#define AK_ST1              0x10
#define AK_ST2              0x18
#define AK_HXL              0x11
#define AK_CNTL2            0x31
#define AK_CNTL3            0x32

#define AK_WIA1_VALUE       0x48
#define AK_WIA2_VALUE       0x0C
#define AK_MODE_CONT_100HZ  0x08
#define AK_ST1_DRDY         0x01

struct MAG_Data {
    // Raw sensor values
    int16_t mag_raw[3];
    
    // Calibrated physical units (µT)
    float mag[3];
    
    // Calibration
    float hard_iron[3];     // Hard iron offsets
    float soft_iron[3][3];  // Soft iron correction matrix
};

class AK09918C {
public:
    AK09918C(uint8_t addr = ADDR_AK09918C);
    
    bool begin(uint8_t mode = AK_MODE_CONT_100HZ);
    bool read();
    bool data_ready();
    void is_overflow(uint8_t hofl_register);
    
    // Calibration
    void set_hard_iron(float bx, float by, float bz);
    void set_soft_iron(const float matrix[3][3]);
    
    const MAG_Data& data() const { return data_; }
    bool is_ok() const { return ok_; }

private:
    uint8_t addr_;
    float scale_;
    MAG_Data data_;
    bool ok_;
    bool overflow_;
};

// =============================================================================
// INA219 Power Monitor
// =============================================================================

struct PWR_Data {
    float bus_voltage_V;
    float shunt_voltage_mV;
    float load_voltage_V;
    float current_mA;
    float power_mW;
    bool overflow;
};

class INA219 {
public:
    INA219(uint8_t addr = ADDR_INA219);
    
    bool begin();
    bool read();
    
    const PWR_Data& data() const { return data_; }
    bool is_ok() const { return ok_; }

private:
    uint8_t addr_;
    PWR_Data data_;
    bool ok_;
    
    // I2C helpers for 16-bit registers
    bool write_register(uint8_t reg, uint16_t value);
    int16_t read_register(uint8_t reg);
};

// =============================================================================
// Global Sensor Instances
// =============================================================================

extern QMI8658C imu;
extern AK09918C mag;
extern INA219 pwr;

// =============================================================================
// High-Level Sensor Functions
// =============================================================================

// Initialize all sensors
void sensors_init();

// Read all sensors (call in sensor task)
void sensors_read_imu();  // Read IMU and Mag
void sensors_read_power();  // Read INA219

// Get sensor status
bool sensors_imu_ok();
bool sensors_mag_ok();
bool sensors_pwr_ok();

#endif // SENSORS_H
