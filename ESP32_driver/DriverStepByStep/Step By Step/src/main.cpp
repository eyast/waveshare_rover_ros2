#include <Arduino.h>
#include "config.h"
#include "syshelpers.h"
#include "i2c_helpers.h"
#include "imu.h"

unsigned long last_imu_update = 0;

void setup(){
  start_serial(BAUDRATE);
  i2c_init(SDA, SCL, I2CFREQUENCY);
  imu.begin();
  last_imu_update = millis();
  sysecho("[Main]:Ready");
}

void loop(){
  unsigned long now = millis();
  if (now - last_imu_update >= IMU_UPDATE_MS) {
        last_imu_update = now;
        imu.update();
  }
}