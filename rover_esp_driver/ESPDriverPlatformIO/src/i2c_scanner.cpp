#include "i2c_scanner.h"

uint8_t scanI2CDevices() {
    uint8_t devicesFound = 0;
    
    Serial.println("Scanning I2C bus...");
    Serial.println("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
    
    for (uint8_t row = 0; row < 8; row++) {
        Serial.printf("%02X: ", row * 16);
        
        for (uint8_t col = 0; col < 16; col++) {
            uint8_t address = row * 16 + col;
            
            // Skip reserved addresses (0x00-0x07 and 0x78-0x7F)
            if (address < 0x08 || address > 0x77) {
                Serial.print("   ");
                continue;
            }
            
            Wire.beginTransmission(address);
            uint8_t error = Wire.endTransmission();
            
            if (error == 0) {
                Serial.printf("%02X ", address);
                devicesFound++;
            } else {
                Serial.print("-- ");
            }
        }
        Serial.println();
    }
    
    Serial.printf("\nFound %d device(s)\n", devicesFound);
    return devicesFound;
}