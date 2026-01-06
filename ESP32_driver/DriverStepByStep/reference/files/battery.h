#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

// =============================================================================
// BATTERY DATA STRUCTURE
// =============================================================================
// WHY: Groups all battery-related measurements together
// WHAT: INA219 measures voltage, current, and power on a DC bus

struct Battery_Data {
    float bus_voltage;      // Voltage on the main power bus (Volts)
    float shunt_voltage;    // Voltage across current sense resistor (mV)
    float current;          // Current flowing (mA)
    float power;            // Power consumption (mW)
    bool overflow;          // true if current exceeded measurement range
};

// =============================================================================
// BATTERY MONITOR CLASS
// =============================================================================
// WHAT: Interfaces with INA219 current/voltage sensor
// WHY: Know battery status - critical for preventing damage and unexpected shutdown
// WHAT IF we didn't monitor battery:
//   - ❌ Run battery too low → damage
//   - ❌ Sudden shutdown → data loss, potential hardware damage
//   - ❌ Don't know power budget for motors

class Battery {
public:
    Battery();
    
    // Initialize INA219
    // WHEN: Call once in setup()
    // RETURNS: true if device responds
    bool begin();
    
    // Read fresh battery data
    // WHEN: Call periodically (e.g., once per second)
    // WHY not more often: Battery changes slowly, frequent reads waste CPU
    bool update();
    
    // Get current data
    const Battery_Data& get_data() const { return data; }
    
    // Check if working
    bool is_ok() const { return battery_ok; }
    
private:
    Battery_Data data;
    bool battery_ok;
    
    // INA219 calibration value
    // WHY: Tells chip how to convert measurements to real units
    // Depends on shunt resistor value (0.01 ohms on Wave Rover)
    uint16_t calibration_value;
};

// =============================================================================
// GLOBAL INSTANCE
// =============================================================================
extern Battery battery;

#endif // BATTERY_H
