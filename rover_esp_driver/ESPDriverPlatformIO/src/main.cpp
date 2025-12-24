#include <ArduinoJson.h>
//JsonDocument jsonCmdReceive;
//JsonDocument jsonInfoSend;
//JsonDocument jsonInfoHttp;
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<512> jsonInfoHttp;

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
#include "ugv_led_ctrl.h" // not sure if needed or not, keeping it for the time being
#include "json_cmd.h" // List of JSON commands
#include "motors.h"
#include "uart_ctrl.h" // Serial bus

#define MAIN_LOOP_RATE_HZ     100
#define MAIN_LOOP_PERIOD_MS   (1000 / MAIN_LOOP_RATE_HZ)  // 10ms

// For debugging: track actual loop rate
static unsigned long loop_count = 0;
static unsigned long last_rate_print = 0;
static float actual_loop_rate = 0.0f;

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
    // init the funcs in switch_module.h
    screenLine_2 = screenLine_3;
    screenLine_3 = "Initialize 12V-switch ctrl";
    oled_update();
    if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
    motionPinInit();
    pidControllerInit();
    led_pwm_ctrl(0, 0);

}

void loop() {
  static unsigned long last_loop_time = 0;
  unsigned long loop_start = millis();

  serialCtrl();


  oledInfoUpdate();
  heartBeatCtrl();
  loop_count++;
}