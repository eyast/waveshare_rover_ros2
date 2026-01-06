#include <Arduino.h>
#include "config.h"
#include "syshelpers.h"

void start_serial(uint32_t baud){
    Serial.begin(baud);
    sysecho("[Main]:Serial OK");
}

void sysecho(const char* message){
    Serial.print(SYS_ID);
    Serial.println(message);
}


void sysecho(const char* message, u_int8_t value){
    Serial.print(SYS_ID);
    Serial.print(message);
    Serial.print(":");
    Serial.println(value, HEX);
}
