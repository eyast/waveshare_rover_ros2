#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// =============================================================================
// IMU DATA STRUCTURE
// =============================================================================
// WHY: Group related data together. Makes it easy to pass all IMU data around.
// WHAT IF we didn't use a struct: Would need 9+ separate variables, easy to
//      mix up or forget one. Struct keeps everything organized.

struct IMU_Data {
    // Raw sensor readings (straight from hardware)
    // WHY keep raw: Useful for calibration and debugging
    int16_t accel_raw[3];   // X, Y, Z in raw ADC counts
    int16_t gyro_raw[3];    // X, Y, Z in raw ADC counts
    int16_t mag_raw[3];     // X, Y, Z in raw ADC counts
    
    // Converted to physical units
    // WHY convert: Real units are meaningful (1g, 90°/s, 50µT)
    float accel[3];         // X, Y, Z in g (gravity units)
    float gyro[3];          // X, Y, Z in degrees/second
    float mag[3];           // X, Y, Z in microtesla (µT)
    
    // Additional useful values
    float temperature;      // Chip temperature in Celsius
    
    // Orientation (simple tilt angles from accelerometer only)
    // WHY: Quick estimate of orientation without complex filter
    // NOTE: These are NOT accurate during motion - only when stationary!
    float roll;             // Rotation around X axis (degrees)
    float pitch;            // Rotation around Y axis (degrees)
};

// =============================================================================
// IMU CLASS
// =============================================================================
// WHY use a class: Encapsulates all IMU functionality in one place
// WHAT: A class is like a blueprint - contains data and functions together
// WHAT IF we used global variables and functions instead:
//   - ❌ Hard to manage multiple IMUs (can't easily add a second one)
//   - ❌ Functions scattered everywhere
//   - ❌ No clear ownership of data
//   - ✅ Class keeps everything organized and self-contained

class IMU {
public:
    // Constructor - called when creating an IMU object
    // WHY: Sets up initial state
    IMU();
    
    // Initialize hardware
    // WHEN: Call once in setup() before using IMU
    // RETURNS: true if both IMU and magnetometer initialized OK
    bool begin();
    
    // Read fresh data from sensors
    // WHEN: Call regularly (e.g., every 20ms in loop)
    // WHAT: Gets new readings from hardware, updates internal data
    // RETURNS: true if read successful
    bool update();
    
    // Get the current sensor data
    // WHY const: Promises not to modify data, just read it
    // RETURNS: Reference to internal data structure (no copying!)
    const IMU_Data& get_data() const { return data; }
    
    // Check if sensors are working
    bool is_ok() const { return imu_ok && mag_ok; }
    bool is_imu_ok() const { return imu_ok; }
    bool is_mag_ok() const { return mag_ok; }
    
private:
    // Private member variables - only this class can access
    // WHY private: Protects data from accidental modification elsewhere
    
    IMU_Data data;          // Stores all sensor readings
    bool imu_ok;            // Is QMI8658C working?
    bool mag_ok;            // Is AK09918C working?
    
    // Conversion scales (calculated during initialization)
    float accel_scale;      // Multiply raw value to get g
    float gyro_scale;       // Multiply raw value to get °/s
    float mag_scale;        // Multiply raw value to get µT
    
    // Private helper functions
    // WHY private: Internal implementation details, users don't need them
    
    bool init_qmi8658c();   // Initialize accelerometer/gyroscope
    bool init_ak09918c();   // Initialize magnetometer
    bool read_qmi8658c();   // Read accel/gyro data
    bool read_ak09918c();   // Read magnetometer data
    void calculate_orientation();  // Compute roll/pitch from accel
};

// =============================================================================
// GLOBAL IMU INSTANCE
// =============================================================================
// WHY global: Need to access from multiple files (main, motor control, etc.)
// WHAT IF not global: Would need to pass IMU object to every function - messy
// NOTE: "extern" means "defined elsewhere" - actual object created in imu.cpp
extern IMU imu;

#endif // IMU_H
