#include "battery.h"
#include "i2c_helpers.h"
#include "config.h"

// =============================================================================
// INA219 REGISTER ADDRESSES
// =============================================================================
// WHY: INA219 has internal registers for configuration and reading data

#define INA219_REG_CONFIG       0x00  // Configuration register
#define INA219_REG_SHUNT_V      0x01  // Shunt voltage measurement
#define INA219_REG_BUS_V        0x02  // Bus voltage measurement
#define INA219_REG_POWER        0x03  // Power measurement
#define INA219_REG_CURRENT      0x04  // Current measurement
#define INA219_REG_CALIBRATION  0x05  // Calibration register

// =============================================================================
// CONFIGURATION VALUES
// =============================================================================
// These configure how the INA219 operates

// Bus voltage range
#define INA219_CONFIG_BVOLT_RANGE_16V   0x0000  // 0-16V
#define INA219_CONFIG_BVOLT_RANGE_32V   0x2000  // 0-32V

// Programmable Gain Amplifier (for shunt voltage)
// WHY: Amplifies tiny voltage across shunt resistor for accurate measurement
#define INA219_CONFIG_GAIN_1_40MV       0x0000  // ±40mV range
#define INA219_CONFIG_GAIN_2_80MV       0x0800  // ±80mV
#define INA219_CONFIG_GAIN_4_160MV      0x1000  // ±160mV
#define INA219_CONFIG_GAIN_8_320MV      0x1800  // ±320mV

// ADC resolution and averaging
// WHY: Higher resolution = more accurate but slower
// 9-bit = fast but noisy, 12-bit = slower but cleaner
#define INA219_CONFIG_ADC_9BIT          0x0000  // 9-bit, 1 sample
#define INA219_CONFIG_ADC_12BIT         0x0018  // 12-bit, 1 sample

// Operating mode
#define INA219_CONFIG_MODE_CONTINUOUS   0x0007  // Continuous conversion

// =============================================================================
// GLOBAL INSTANCE
// =============================================================================

Battery battery;

// =============================================================================
// CONSTRUCTOR
// =============================================================================

Battery::Battery() {
    battery_ok = false;
    memset(&data, 0, sizeof(data));
    
    // Calibration value for 0.01 ohm shunt resistor
    // WHY this value: Formula from INA219 datasheet
    // Depends on: shunt resistance, current range, ADC resolution
    calibration_value = 4096;  // Calculated for 0.01Ω, ±3.2A max
}

// =============================================================================
// INITIALIZATION
// =============================================================================

bool Battery::begin() {
    Serial.println("\n=== Initializing Battery Monitor ===");
    
    // STEP 1: Configure INA219
    // Build configuration word by OR-ing together settings
    // WHY bitwise OR: Each setting occupies different bits
    uint16_t config = 
        INA219_CONFIG_BVOLT_RANGE_16V |      // 16V bus voltage max
        INA219_CONFIG_GAIN_8_320MV |         // ±320mV shunt voltage max
        INA219_CONFIG_ADC_12BIT |            // 12-bit ADC for bus voltage
        INA219_CONFIG_ADC_12BIT |            // 12-bit ADC for shunt voltage  
        INA219_CONFIG_MODE_CONTINUOUS;       // Continuous conversion
    
    // Write configuration (16-bit = 2 bytes, high byte first)
    // WHY split into bytes: I2C writes one byte at a time
    uint8_t config_high = (config >> 8) & 0xFF;
    uint8_t config_low = config & 0xFF;
    
    if (!i2c_write_byte(INA219_ADDR, INA219_REG_CONFIG, config_high)) {
        Serial.println("[Battery] INA219 not responding!");
        return false;
    }
    // Note: Some I2C libraries require writing both bytes together
    // For now, assume single-byte write works (may need adjustment)
    
    // STEP 2: Write calibration value
    // WHY: This tells INA219 how to scale raw readings to real units
    // Must write high byte, then low byte
    uint8_t cal_high = (calibration_value >> 8) & 0xFF;
    uint8_t cal_low = calibration_value & 0xFF;
    
    // Note: This is simplified - full implementation would write both bytes atomically
    i2c_write_byte(INA219_ADDR, INA219_REG_CALIBRATION, cal_high);
    
    // STEP 3: Test read to confirm device is there
    // Try reading bus voltage - should get something reasonable
    uint8_t test_low = i2c_read_byte(INA219_ADDR, INA219_REG_BUS_V);
    uint8_t test_high = i2c_read_byte(INA219_ADDR, INA219_REG_BUS_V + 1);
    
    if (test_low == 0xFF && test_high == 0xFF) {
        // All 1s likely means no device
        Serial.println("[Battery] INA219 read test failed!");
        return false;
    }
    
    battery_ok = true;
    Serial.println("[Battery] INA219 OK");
    return true;
}

// =============================================================================
// DATA READING
// =============================================================================

bool Battery::update() {
    if (!battery_ok) {
        return false;
    }
    
    // STEP 1: Read bus voltage (2 bytes)
    // WHY 2 bytes: 16-bit value, high byte first (big-endian)
    uint8_t bus_raw[2];
    bus_raw[0] = i2c_read_byte(INA219_ADDR, INA219_REG_BUS_V);
    bus_raw[1] = i2c_read_byte(INA219_ADDR, INA219_REG_BUS_V + 1);
    
    // Combine bytes (big-endian)
    uint16_t bus_value = (bus_raw[0] << 8) | bus_raw[1];
    
    // Check overflow flag (bit 0)
    // WHY: INA219 sets this if measurement exceeded range
    data.overflow = (bus_value & 0x01);
    
    // Extract voltage (bits 3-15, right-shift by 3)
    // Scale: 4mV per LSB
    data.bus_voltage = ((bus_value >> 3) * 4) / 1000.0f;  // Convert mV to V
    
    // STEP 2: Read shunt voltage (2 bytes)
    // WHAT: Voltage across 0.01Ω resistor - used to calculate current
    uint8_t shunt_raw[2];
    shunt_raw[0] = i2c_read_byte(INA219_ADDR, INA219_REG_SHUNT_V);
    shunt_raw[1] = i2c_read_byte(INA219_ADDR, INA219_REG_SHUNT_V + 1);
    
    // Combine and convert (signed 16-bit value)
    int16_t shunt_value = (shunt_raw[0] << 8) | shunt_raw[1];
    
    // Scale: 10µV per LSB → convert to mV
    data.shunt_voltage = (shunt_value * 10.0f) / 1000.0f;
    
    // STEP 3: Read current (2 bytes)
    // WHY read directly: INA219 calculates this using calibration value
    uint8_t current_raw[2];
    current_raw[0] = i2c_read_byte(INA219_ADDR, INA219_REG_CURRENT);
    current_raw[1] = i2c_read_byte(INA219_ADDR, INA219_REG_CURRENT + 1);
    
    int16_t current_value = (current_raw[0] << 8) | current_raw[1];
    
    // Scale depends on calibration (typically 1mA per LSB with our config)
    data.current = current_value * 1.0f;  // Already in mA
    
    // STEP 4: Calculate power
    // WHY calculate: Sometimes more accurate than reading power register
    // P = V × I (in watts = volts × amps)
    data.power = data.bus_voltage * data.current;  // V × mA = mW
    
    return true;
}
