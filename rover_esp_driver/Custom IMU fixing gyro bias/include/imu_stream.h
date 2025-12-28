#ifndef IMU_STREAM_H
#define IMU_STREAM_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "config.h"
#include "qmi8658c.h"
#include "ak09918c.h"
#include "madgwick.h"
#include "motioncal_output.h"

// =============================================================================
// External references (defined in main.cpp)
// =============================================================================
extern QMI8658C imu;
extern AK09918C mag;
extern MadgwickFilter filter;
extern bool imu_ok;
extern bool mag_ok;
extern uint32_t last_update_us;

// Stream control (defined in uart_ctrl.h)
extern bool imu_stream_enabled;
extern bool stream_as_json;

//static uint32_t last_stream_ms = 0;

extern float temp;
void updateIMUFilter();
void sendIMUStreamData();
float getYaw();
float getPitch();
float getRoll();

void verify_mag_dip();

#endif // IMU_STREAM_H