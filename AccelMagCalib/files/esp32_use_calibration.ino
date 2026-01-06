/*
 * ESP32 Example: Using Calibration Parameters
 * 
 * This shows how to apply calibration parameters obtained from
 * the ellipsoid_calibration.py script to your sensor readings.
 * 
 * Workflow:
 * 1. Collect calibration data using esp32_calibration_data_collection.ino
 * 2. Run Python calibration: python ellipsoid_calibration.py data.txt
 * 3. Copy the bias and calibration matrix values into this code
 * 4. Upload and run - sensor is now calibrated!
 * 
 * The calibration does two steps:
 *   1. Subtract bias (offset correction)
 *   2. Multiply by calibration matrix (scale & orthogonality correction)
 */

#include <Wire.h>

// ========== CALIBRATION PARAMETERS ==========
// REPLACE THESE with values from your calibration!
// These are example values - yours will be different!

// Accelerometer calibration example
float accel_bias[3] = {0.123456, -0.098765, 0.054321};  // [bx, by, bz]

float accel_cal_matrix[3][3] = {
  {0.987654, 0.012345, 0.006789},  // Row 0
  {0.012345, 1.023456, 0.009876},  // Row 1
  {0.006789, 0.009876, 0.995432}   // Row 2
};

// Magnetometer calibration example (if using magnetometer)
float mag_bias[3] = {12.345678, -8.765432, 5.432109};

float mag_cal_matrix[3][3] = {
  {1.234567, 0.012345, 0.006789},
  {0.012345, 0.987654, 0.009876},
  {0.006789, 0.009876, 1.123456}
};

// ========== I2C CONFIGURATION ==========
#define I2C_SDA 21
#define I2C_SCL 22
#define SENSOR_ADDR 0x68  // Adjust for your sensor

// ========== CALIBRATION FUNCTIONS ==========

void calibrateAccelerometer(float raw_x, float raw_y, float raw_z,
                           float &cal_x, float &cal_y, float &cal_z) {
  /*
   * Apply accelerometer calibration.
   * 
   * Steps:
   *   1. Subtract bias: centered = raw - bias
   *   2. Apply calibration matrix: calibrated = matrix × centered
   * 
   * Parameters:
   *   raw_x, raw_y, raw_z: Raw sensor readings
   *   cal_x, cal_y, cal_z: Output calibrated values
   */
  
  // Step 1: Remove bias
  float centered_x = raw_x - accel_bias[0];
  float centered_y = raw_y - accel_bias[1];
  float centered_z = raw_z - accel_bias[2];
  
  // Step 2: Apply calibration matrix
  // [cal_x]   [m00 m01 m02]   [centered_x]
  // [cal_y] = [m10 m11 m12] × [centered_y]
  // [cal_z]   [m20 m21 m22]   [centered_z]
  
  cal_x = accel_cal_matrix[0][0] * centered_x +
          accel_cal_matrix[0][1] * centered_y +
          accel_cal_matrix[0][2] * centered_z;
  
  cal_y = accel_cal_matrix[1][0] * centered_x +
          accel_cal_matrix[1][1] * centered_y +
          accel_cal_matrix[1][2] * centered_z;
  
  cal_z = accel_cal_matrix[2][0] * centered_x +
          accel_cal_matrix[2][1] * centered_y +
          accel_cal_matrix[2][2] * centered_z;
}

void calibrateMagnetometer(float raw_x, float raw_y, float raw_z,
                          float &cal_x, float &cal_y, float &cal_z) {
  /*
   * Apply magnetometer calibration.
   * Same process as accelerometer.
   */
  
  // Step 1: Remove bias
  float centered_x = raw_x - mag_bias[0];
  float centered_y = raw_y - mag_bias[1];
  float centered_z = raw_z - mag_bias[2];
  
  // Step 2: Apply calibration matrix
  cal_x = mag_cal_matrix[0][0] * centered_x +
          mag_cal_matrix[0][1] * centered_y +
          mag_cal_matrix[0][2] * centered_z;
  
  cal_y = mag_cal_matrix[1][0] * centered_x +
          mag_cal_matrix[1][1] * centered_y +
          mag_cal_matrix[1][2] * centered_z;
  
  cal_z = mag_cal_matrix[2][0] * centered_x +
          mag_cal_matrix[2][1] * centered_y +
          mag_cal_matrix[2][2] * centered_z;
}

// ========== SENSOR READING ==========
// Same functions from data collection sketch
// (Adjust for your specific sensor)

void readAccelerometer(float &x, float &y, float &z) {
  // Example for MPU6050
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDR, 6, true);
  
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  
  // Convert to g
  x = ax / 16384.0;
  y = ay / 16384.0;
  z = az / 16384.0;
}

// ========== HELPER FUNCTIONS ==========

float calculateMagnitude(float x, float y, float z) {
  /*
   * Calculate magnitude of 3D vector.
   * magnitude = √(x² + y² + z²)
   */
  return sqrt(x*x + y*y + z*z);
}

void calculateRollPitch(float ax, float ay, float az, float &roll, float &pitch) {
  /*
   * Calculate roll and pitch angles from accelerometer.
   * 
   * Roll: rotation around X-axis
   * Pitch: rotation around Y-axis
   * 
   * Note: This assumes accelerometer is calibrated!
   * Uncalibrated sensors will give wrong angles.
   */
  
  // Roll (rotation around X-axis)
  roll = atan2(ay, az) * 180.0 / PI;
  
  // Pitch (rotation around Y-axis)
  pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
}

float calculateHeading(float mx, float my, float mz, float roll, float pitch) {
  /*
   * Calculate compass heading from magnetometer.
   * 
   * Note: This assumes BOTH accelerometer and magnetometer are calibrated!
   * Requires tilt compensation using roll and pitch.
   */
  
  // Convert angles to radians
  float roll_rad = roll * PI / 180.0;
  float pitch_rad = pitch * PI / 180.0;
  
  // Tilt compensation
  float mx_comp = mx * cos(pitch_rad) + mz * sin(pitch_rad);
  float my_comp = mx * sin(roll_rad) * sin(pitch_rad) +
                  my * cos(roll_rad) -
                  mz * sin(roll_rad) * cos(pitch_rad);
  
  // Calculate heading
  float heading = atan2(my_comp, mx_comp) * 180.0 / PI;
  
  // Normalize to 0-360°
  if (heading < 0) {
    heading += 360.0;
  }
  
  return heading;
}

// ========== VALIDATION FUNCTIONS ==========

void validateCalibration() {
  /*
   * Run validation tests to check calibration quality.
   * 
   * For accelerometer:
   *   - Place sensor flat on table
   *   - Should read: (0, 0, 1.0) with magnitude ≈ 1.0
   * 
   * For magnetometer:
   *   - Rotate sensor 360° in horizontal plane
   *   - Magnitude should stay constant
   *   - Heading should sweep 0-360°
   */
  
  Serial.println("\n========================================");
  Serial.println("CALIBRATION VALIDATION");
  Serial.println("========================================");
  
  // Test 1: Static reading
  Serial.println("\nTest 1: Static Reading");
  Serial.println("Place sensor flat on table and don't move it.");
  Serial.println("Press any key when ready...");
  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read();  // Clear buffer
  
  delay(1000);
  
  // Collect 50 samples
  float sum_x = 0, sum_y = 0, sum_z = 0;
  float sum_mag = 0;
  int n_samples = 50;
  
  for (int i = 0; i < n_samples; i++) {
    float raw_x, raw_y, raw_z;
    float cal_x, cal_y, cal_z;
    
    readAccelerometer(raw_x, raw_y, raw_z);
    calibrateAccelerometer(raw_x, raw_y, raw_z, cal_x, cal_y, cal_z);
    
    sum_x += cal_x;
    sum_y += cal_y;
    sum_z += cal_z;
    sum_mag += calculateMagnitude(cal_x, cal_y, cal_z);
    
    delay(20);
  }
  
  float avg_x = sum_x / n_samples;
  float avg_y = sum_y / n_samples;
  float avg_z = sum_z / n_samples;
  float avg_mag = sum_mag / n_samples;
  
  Serial.println("\nResults:");
  Serial.print("  Average: (");
  Serial.print(avg_x, 4);
  Serial.print(", ");
  Serial.print(avg_y, 4);
  Serial.print(", ");
  Serial.print(avg_z, 4);
  Serial.println(")");
  Serial.print("  Magnitude: ");
  Serial.println(avg_mag, 4);
  
  // Check results
  float error_mag = abs(avg_mag - 1.0);
  
  if (error_mag < 0.02) {
    Serial.println("  ✓ EXCELLENT! Magnitude error < 2%");
  } else if (error_mag < 0.05) {
    Serial.println("  ✓ GOOD. Magnitude error < 5%");
  } else {
    Serial.println("  ✗ WARNING! Magnitude error > 5%");
    Serial.println("    Check your calibration parameters!");
  }
  
  if (abs(avg_z - 1.0) < 0.05) {
    Serial.println("  ✓ Z-axis correct (should be ~1.0 when flat)");
  } else {
    Serial.println("  ✗ WARNING! Z-axis not ~1.0 when flat");
  }
  
  Serial.println("\n========================================");
}

// ========== MAIN PROGRAM ==========

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("CALIBRATED SENSOR EXAMPLE");
  Serial.println("========================================");
  
  // Initialize I2C and sensor
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  
  // Initialize sensor (same as data collection)
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  
  Serial.println("\nSensor initialized!");
  
  // Display calibration parameters
  Serial.println("\nLoaded Calibration Parameters:");
  Serial.println("-------------------------------");
  Serial.println("Bias:");
  Serial.print("  [");
  Serial.print(accel_bias[0], 6);
  Serial.print(", ");
  Serial.print(accel_bias[1], 6);
  Serial.print(", ");
  Serial.print(accel_bias[2], 6);
  Serial.println("]");
  
  Serial.println("\nCalibration Matrix:");
  for (int i = 0; i < 3; i++) {
    Serial.print("  [");
    for (int j = 0; j < 3; j++) {
      Serial.print(accel_cal_matrix[i][j], 6);
      if (j < 2) Serial.print(", ");
    }
    Serial.println("]");
  }
  
  // Run validation
  Serial.println("\nWould you like to run validation? (y/n)");
  while (!Serial.available()) delay(100);
  char choice = Serial.read();
  while (Serial.available()) Serial.read();
  
  if (choice == 'y' || choice == 'Y') {
    validateCalibration();
  }
  
  Serial.println("\nStarting continuous output...");
  Serial.println("Format: Raw(x,y,z) | Cal(x,y,z) | Mag | Roll | Pitch");
  Serial.println("----------------------------------------");
  
  delay(2000);
}

void loop() {
  // Read raw sensor
  float raw_x, raw_y, raw_z;
  readAccelerometer(raw_x, raw_y, raw_z);
  
  // Apply calibration
  float cal_x, cal_y, cal_z;
  calibrateAccelerometer(raw_x, raw_y, raw_z, cal_x, cal_y, cal_z);
  
  // Calculate magnitude
  float raw_mag = calculateMagnitude(raw_x, raw_y, raw_z);
  float cal_mag = calculateMagnitude(cal_x, cal_y, cal_z);
  
  // Calculate angles (from calibrated data)
  float roll, pitch;
  calculateRollPitch(cal_x, cal_y, cal_z, roll, pitch);
  
  // Display
  Serial.print("Raw: (");
  Serial.print(raw_x, 3);
  Serial.print(", ");
  Serial.print(raw_y, 3);
  Serial.print(", ");
  Serial.print(raw_z, 3);
  Serial.print(") | Cal: (");
  Serial.print(cal_x, 3);
  Serial.print(", ");
  Serial.print(cal_y, 3);
  Serial.print(", ");
  Serial.print(cal_z, 3);
  Serial.print(") | Mag: ");
  Serial.print(cal_mag, 3);
  Serial.print(" | Roll: ");
  Serial.print(roll, 1);
  Serial.print("° | Pitch: ");
  Serial.print(pitch, 1);
  Serial.println("°");
  
  delay(100);  // 10 Hz output
}

// ========== ADVANCED: STORING CALIBRATION IN EEPROM/FLASH ==========

#include <Preferences.h>

void saveCalibrationToFlash() {
  /*
   * Save calibration parameters to ESP32 flash memory.
   * Persists across power cycles.
   */
  
  Preferences prefs;
  prefs.begin("sensor_cal", false);  // namespace "sensor_cal", read-write
  
  // Save bias
  prefs.putFloat("accel_bx", accel_bias[0]);
  prefs.putFloat("accel_by", accel_bias[1]);
  prefs.putFloat("accel_bz", accel_bias[2]);
  
  // Save matrix (flatten 3x3 to 9 values)
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      String key = "accel_m" + String(i) + String(j);
      prefs.putFloat(key.c_str(), accel_cal_matrix[i][j]);
    }
  }
  
  prefs.end();
  
  Serial.println("Calibration saved to flash!");
}

void loadCalibrationFromFlash() {
  /*
   * Load calibration parameters from ESP32 flash memory.
   */
  
  Preferences prefs;
  prefs.begin("sensor_cal", true);  // namespace "sensor_cal", read-only
  
  // Load bias
  accel_bias[0] = prefs.getFloat("accel_bx", 0.0);
  accel_bias[1] = prefs.getFloat("accel_by", 0.0);
  accel_bias[2] = prefs.getFloat("accel_bz", 0.0);
  
  // Load matrix
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      String key = "accel_m" + String(i) + String(j);
      accel_cal_matrix[i][j] = prefs.getFloat(key.c_str(), (i == j) ? 1.0 : 0.0);
    }
  }
  
  prefs.end();
  
  Serial.println("Calibration loaded from flash!");
}
