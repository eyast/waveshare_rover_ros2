#include "i2c_helpers.h"

void i2c_init(uint8_t sda, uint8_t scl, uint32_t frequency){
    Wire.begin(sda, scl, frequency);
    Wire.setClock(frequency);
    sysecho("[I2C]:Initialized");
}
bool i2c_write_byte(uint8_t device_addr, uint8_t register_addr, uint8_t value) {
    // STEP 1: Start communication with device
    // WHAT: Wire.beginTransmission() opens communication channel
    // WHY: Tells I2C bus we want to talk to this specific device
    Wire.beginTransmission(device_addr);
    
    // STEP 2: Tell device which register we want to write to
    // WHY: Devices have many registers - we must specify which one
    Wire.write(register_addr);
    
    // STEP 3: Send the actual data
    Wire.write(value);
    
    // STEP 4: End transmission and check for errors
    // WHAT: endTransmission() sends the data and waits for acknowledgment
    // RETURNS: 0 = success, non-zero = error
    // WHY: Device must acknowledge it received data - if not, something's wrong
    uint8_t result = Wire.endTransmission();
    
    return (result == 0);  // true if success (0), false if error
}

uint8_t i2c_read_byte(uint8_t device_addr, uint8_t register_addr) {
    // STEP 1: Tell device which register we want to read
    Wire.beginTransmission(device_addr);
    Wire.write(register_addr);
    
    // WHAT: false parameter means "don't release the bus yet"
    // WHY: We're going to read immediately - keep control of the bus
    Wire.endTransmission(false);
    
    // STEP 2: Request 1 byte from the device
    // WHAT: requestFrom() asks device to send data back
    Wire.requestFrom(device_addr, (uint8_t)1);
    
    // STEP 3: Check if data is available and read it
    // WHAT: available() returns how many bytes are waiting
    // WHY: Check before reading to avoid reading garbage
    if (Wire.available()) {
        return Wire.read();  // Read and return the byte
    }
    
    // If we get here, something went wrong
    return 0xFF;  // Error indicator
}

bool i2c_read_bytes(uint8_t device_addr, uint8_t start_register, 
                    uint8_t* buffer, uint8_t count) {
    // STEP 1: Tell device where to start reading from
    Wire.beginTransmission(device_addr);
    Wire.write(start_register);
    
    // Check if device acknowledged
    if (Wire.endTransmission(false) != 0) {
        return false;  // Device didn't respond
    }
    
    // STEP 2: Request multiple bytes
    // WHAT: Device will automatically increment register address
    // WHY: Faster than reading one byte at a time
    uint8_t received = Wire.requestFrom(device_addr, count);
    
    // STEP 3: Check we got what we asked for
    if (received < count) {
        return false;  // Didn't get enough bytes
    }
    
    // STEP 4: Read all the bytes into our buffer
    // WHAT: Loop through and read each byte
    // WHY: Wire library stores bytes in internal buffer - we copy to ours
    for (uint8_t i = 0; i < count; i++) {
        buffer[i] = Wire.read();
    }
    
    return true;  // Success!
}
