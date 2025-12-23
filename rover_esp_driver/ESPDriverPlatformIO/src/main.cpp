#include <ArduinoJson.h>
JsonDocument jsonCmdReceive;
JsonDocument jsonInfoSend;
JsonDocument jsonInfoHttp;

#include <SCServo.h>
// #include <Arduino.h>
// #include <Wire.h>
// #include <i2c_scanner.h> // custom I2C scanner
// call with scanI2CDevices();
#include <Adafruit_SSD1306.h> // OLED driver
#include <INA219_WE.h>
#include <PID_v2.h>
#include "battery_ctrl.h" // custom functions to initialize and function INA
#include "oled_ctrl.h" // Important, because timekeeping is kept here...
#include "ugv_config.h" // config variables (addresses etc)
#include "json_cmd.h" // List of JSON commands
#include "IMU_ctrl.h"
#include "motors.h"


void setup() {
    Serial.begin(115200);
    Wire.begin(S_SDA, S_SCL);
    while(!Serial) {}
    delay(1000);  
    // Devices found
    // 0C - Must be the compass, AKAK09918
    // 3C - OLED
    // 6B - QMI8658_ADDR - https://www.waveshare.com/w/upload/3/36/QMI8658C_datasheet_rev_0.8.pdf
    // 6B provides temperature, ax, ay, az, Gx, Gy, Gz
    // 42 - INA219: Voltage/Current
    
    ina219_init();
    inaDataUpdate();
    // scanI2CDevices();
    mm_settings(mainType, moduleType);
    init_oled();
    screenLine_0 = "Custom Waveshare";
    screenLine_1 = "Rover Driver";
    screenLine_2 = "";
    screenLine_3 = "Starting...";
    oled_update();
    delay(1000);
    imu_init();

}

void loop() {

}