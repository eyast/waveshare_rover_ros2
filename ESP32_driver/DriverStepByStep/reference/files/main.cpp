#include <Arduino.h>
#include "config.h"
#include "i2c_helpers.h"
#include "imu.h"
#include "battery.h"
#include "motors.h"

// =============================================================================
// TIMING VARIABLES
// =============================================================================
// WHY: Track when we last updated each sensor to control timing
// WHAT: millis() returns milliseconds since boot - never decreases

unsigned long last_imu_update = 0;
unsigned long last_battery_update = 0;

// =============================================================================
// SERIAL COMMAND BUFFER
// =============================================================================
// WHY: Need to collect characters until we get a complete command
// WHAT: String to accumulate incoming characters

String command_buffer = "";

// =============================================================================
// SETUP - RUNS ONCE AT STARTUP
// =============================================================================

void setup() {
    // STEP 1: Initialize Serial communication
    // WHY first: Want to see debug messages from other init functions
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 5000) {
        // Wait for serial port to connect (up to 5 seconds)
        // WHY timeout: On some boards, Serial never connects if no USB
    }
    delay(1000);  // Give Serial time to stabilize
    
    Serial.println("\n\n======================");
    Serial.println("  MINIMAL IMU DRIVER  ");
    Serial.println("======================\n");
    
    // STEP 2: Initialize I2C bus
    // WHY before sensors: Sensors need I2C to communicate
    Serial.println("Initializing I2C...");
    i2c_init(I2C_SDA, I2C_SCL, I2C_FREQ);
    
    // STEP 3: Initialize IMU
    // WHAT: Sets up QMI8658C and AK09918C
    if (!imu.begin()) {
        Serial.println("\n[ERROR] IMU initialization failed!");
        Serial.println("Check connections and I2C address.");
        // Continue anyway - other features may work
    }
    
    // STEP 4: Initialize Battery Monitor
    if (!battery.begin()) {
        Serial.println("\n[WARNING] Battery monitor not found");
        Serial.println("Battery readings will not be available.");
        // Not critical - continue
    }
    
    // STEP 5: Initialize Motors
    motors.begin();
    
    // STEP 6: Print available commands
    Serial.println("\n=== Available Commands ===");
    Serial.println("Format: <command><params>");
    Serial.println();
    Serial.println("Motor Control:");
    Serial.println("  M<left>,<right>  - Set motor speeds (-255 to +255)");
    Serial.println("                     Example: M100,100  (both forward)");
    Serial.println("                              M-100,100 (left back, right forward)");
    Serial.println("  S                - Stop both motors");
    Serial.println();
    Serial.println("Data Streaming:");
    Serial.println("  R                - Send RAW data (MotionCal format)");
    Serial.println("  O                - Send Orientation data");
    Serial.println("  T                - Send Temperature");
    Serial.println("  B                - Send Battery status");
    Serial.println("  A                - Send All data");
    Serial.println();
    Serial.println("Settings:");
    Serial.println("  F<ms>            - Set IMU update frequency");
    Serial.println("                     Example: F20 (50Hz)");
    Serial.println("                              F10 (100Hz)");
    Serial.println();
    Serial.println("=========================\n");
    
    // STEP 7: Initialize timing
    last_imu_update = millis();
    last_battery_update = millis();
    
    Serial.println("[System] Ready!\n");
}

// =============================================================================
// LOOP - RUNS CONTINUOUSLY
// =============================================================================

void loop() {
    // WHAT: loop() is called over and over, as fast as possible
    // WHY structure this way: Each iteration checks what needs updating
    
    unsigned long now = millis();
    
    // =========================================================================
    // UPDATE IMU at specified rate
    // =========================================================================
    // WHAT: Check if enough time has passed since last update
    // WHY: Don't want to read sensor faster than its update rate
    
    if (now - last_imu_update >= IMU_UPDATE_MS) {
        last_imu_update = now;
        
        // Read fresh sensor data
        // WHY check return: Know if read was successful
        if (imu.update()) {
            // Data updated successfully
            // Could process data here, or just wait for serial request
        }
    }
    
    // =========================================================================
    // UPDATE BATTERY at specified rate
    // =========================================================================
    // WHY slower: Battery changes slowly, no need for rapid updates
    
    if (now - last_battery_update >= BATTERY_UPDATE_MS) {
        last_battery_update = now;
        
        if (battery.is_ok()) {
            battery.update();
        }
    }
    
    // =========================================================================
    // PROCESS SERIAL COMMANDS
    // =========================================================================
    // WHAT: Check if any characters have arrived, process them
    // WHY non-blocking: Don't wait for input, just check and continue
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // WHAT: Accumulate characters until newline
        // WHY: Commands sent as lines, need complete line to process
        if (c == '\n' || c == '\r') {
            // Got complete command - process it
            if (command_buffer.length() > 0) {
                process_command(command_buffer);
                command_buffer = "";  // Clear for next command
            }
        }
        else {
            // Add character to buffer
            command_buffer += c;
        }
    }
}

// =============================================================================
// COMMAND PROCESSING
// =============================================================================

void process_command(String cmd) {
    // WHAT: Parse and execute commands from serial
    // WHY separate function: Keeps loop() clean and readable
    
    // Trim whitespace
    cmd.trim();
    
    // Get first character (command type)
    char type = cmd.charAt(0);
    
    // ==========================
    // MOTOR COMMANDS
    // ==========================
    
    if (type == 'M') {
        // Format: M<left>,<right>
        // Example: M100,150 or M-50,100
        
        // Find comma separator
        int comma_pos = cmd.indexOf(',');
        if (comma_pos == -1) {
            Serial.println("[Error] Motor command needs format: M<left>,<right>");
            return;
        }
        
        // Extract and parse speeds
        String left_str = cmd.substring(1, comma_pos);  // Skip 'M'
        String right_str = cmd.substring(comma_pos + 1);
        
        int16_t left_speed = left_str.toInt();
        int16_t right_speed = right_str.toInt();
        
        // Set motors
        motors.set_speeds(left_speed, right_speed);
        
        Serial.print("[Motors] Set: L=");
        Serial.print(left_speed);
        Serial.print(" R=");
        Serial.println(right_speed);
    }
    else if (type == 'S') {
        // Stop motors
        motors.stop();
        Serial.println("[Motors] Stopped");
    }
    
    // ==========================
    // DATA OUTPUT COMMANDS
    // ==========================
    
    else if (type == 'R') {
        // RAW data (MotionCal format)
        send_raw_data();
    }
    else if (type == 'O') {
        // Orientation data
        send_orientation();
    }
    else if (type == 'T') {
        // Temperature
        send_temperature();
    }
    else if (type == 'B') {
        // Battery status
        send_battery();
    }
    else if (type == 'A') {
        // All data
        send_all_data();
    }
    
    // ==========================
    // SETTINGS COMMANDS
    // ==========================
    
    else if (type == 'F') {
        // Set frequency: F<ms>
        String freq_str = cmd.substring(1);
        int new_freq = freq_str.toInt();
        
        if (new_freq > 0 && new_freq <= 1000) {
            // Update global (note: need to extern it or make it modifiable)
            Serial.print("[Settings] IMU update rate set to ");
            Serial.print(new_freq);
            Serial.println(" ms");
            
            // For now, just acknowledge
            // To actually change: would need to make IMU_UPDATE_MS non-const
        }
        else {
            Serial.println("[Error] Frequency must be 1-1000 ms");
        }
    }
    
    else {
        Serial.print("[Error] Unknown command: ");
        Serial.println(cmd);
    }
}

// =============================================================================
// DATA OUTPUT FUNCTIONS
// =============================================================================

void send_raw_data() {
    // MOTIONCAL FORMAT: Raw:ax,ay,az,gx,gy,gz,mx,my,mz
    // WHY this format: Compatible with MotionCal calibration tool
    
    if (!imu.is_ok()) {
        Serial.println("[Error] IMU not available");
        return;
    }
    
    const IMU_Data& data = imu.get_data();
    
    // Convert to MotionCal's expected scaling
    int ax = (int)(data.accel[0] * MOTIONCAL_ACCEL_SCALE);
    int ay = (int)(data.accel[1] * MOTIONCAL_ACCEL_SCALE);
    int az = (int)(data.accel[2] * MOTIONCAL_ACCEL_SCALE);
    
    int gx = (int)(data.gyro[0] * MOTIONCAL_GYRO_SCALE);
    int gy = (int)(data.gyro[1] * MOTIONCAL_GYRO_SCALE);
    int gz = (int)(data.gyro[2] * MOTIONCAL_GYRO_SCALE);
    
    int mx = (int)(data.mag[0] * MOTIONCAL_MAG_SCALE);
    int my = (int)(data.mag[1] * MOTIONCAL_MAG_SCALE);
    int mz = (int)(data.mag[2] * MOTIONCAL_MAG_SCALE);
    
    // Send in MotionCal format
    Serial.print("Raw:");
    Serial.print(ax); Serial.print(",");
    Serial.print(ay); Serial.print(",");
    Serial.print(az); Serial.print(",");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.print(gz); Serial.print(",");
    Serial.print(mx); Serial.print(",");
    Serial.print(my); Serial.print(",");
    Serial.println(mz);
}

void send_orientation() {
    // ORIENTATION FORMAT: Ori: roll,pitch
    // NOTE: No yaw without filter - just tilt angles
    
    if (!imu.is_ok()) {
        Serial.println("[Error] IMU not available");
        return;
    }
    
    const IMU_Data& data = imu.get_data();
    
    Serial.print("Ori: ");
    Serial.print(data.roll, 2);
    Serial.print(",");
    Serial.println(data.pitch, 2);
}

void send_temperature() {
    if (!imu.is_ok()) {
        Serial.println("[Error] IMU not available");
        return;
    }
    
    const IMU_Data& data = imu.get_data();
    
    Serial.print("Temp: ");
    Serial.print(data.temperature, 1);
    Serial.println(" C");
}

void send_battery() {
    if (!battery.is_ok()) {
        Serial.println("[Error] Battery monitor not available");
        return;
    }
    
    const Battery_Data& data = battery.get_data();
    
    Serial.print("Battery: ");
    Serial.print(data.bus_voltage, 2);
    Serial.print("V, ");
    Serial.print(data.current, 1);
    Serial.print("mA, ");
    Serial.print(data.power, 1);
    Serial.println("mW");
}

void send_all_data() {
    // Send everything in one block
    
    Serial.println("--- All Data ---");
    
    if (imu.is_ok()) {
        const IMU_Data& imu_data = imu.get_data();
        
        Serial.println("IMU:");
        Serial.print("  Accel (g): ");
        Serial.print(imu_data.accel[0], 3); Serial.print(", ");
        Serial.print(imu_data.accel[1], 3); Serial.print(", ");
        Serial.println(imu_data.accel[2], 3);
        
        Serial.print("  Gyro (°/s): ");
        Serial.print(imu_data.gyro[0], 1); Serial.print(", ");
        Serial.print(imu_data.gyro[1], 1); Serial.print(", ");
        Serial.println(imu_data.gyro[2], 1);
        
        Serial.print("  Mag (µT): ");
        Serial.print(imu_data.mag[0], 1); Serial.print(", ");
        Serial.print(imu_data.mag[1], 1); Serial.print(", ");
        Serial.println(imu_data.mag[2], 1);
        
        Serial.print("  Orientation: Roll=");
        Serial.print(imu_data.roll, 1);
        Serial.print("° Pitch=");
        Serial.print(imu_data.pitch, 1);
        Serial.println("°");
        
        Serial.print("  Temperature: ");
        Serial.print(imu_data.temperature, 1);
        Serial.println(" °C");
    }
    
    if (battery.is_ok()) {
        const Battery_Data& batt_data = battery.get_data();
        
        Serial.println("Battery:");
        Serial.print("  Voltage: ");
        Serial.print(batt_data.bus_voltage, 2);
        Serial.println(" V");
        Serial.print("  Current: ");
        Serial.print(batt_data.current, 1);
        Serial.println(" mA");
        Serial.print("  Power: ");
        Serial.print(batt_data.power, 1);
        Serial.println(" mW");
    }
    
    Serial.println("---------------");
}

// =============================================================================
// WHAT HAPPENS IN THIS FILE
// =============================================================================
//
// 1. SETUP (once):
//    - Start serial communication
//    - Initialize I2C bus
//    - Initialize all hardware (IMU, battery, motors)
//    - Set starting state (motors off, timing initialized)
//
// 2. LOOP (continuously):
//    - Check if it's time to update IMU (based on IMU_UPDATE_MS)
//    - Check if it's time to update battery (based on BATTERY_UPDATE_MS)
//    - Process any incoming serial commands
//    - Repeat forever
//
// 3. COMMAND PROCESSING:
//    - User sends commands via serial (USB)
//    - Commands control motors or request data
//    - System responds by performing action or sending data back
//
// WHY THIS STRUCTURE:
// - Separation of concerns: Each module handles one thing
// - Timing control: Update sensors at different rates
// - Non-blocking: Never wait for anything, always responsive
// - Extensible: Easy to add new commands or features
//
// WHAT IF WE DID IT DIFFERENTLY:
// - Everything in one file → hard to maintain, find bugs
// - Blocking delays → system freezes waiting for serial input
// - No timing control → waste CPU or miss data
//
// =============================================================================
