/*
 * ESP32 Accelerometer/Magnetometer Data Collection for Calibration
 * 
 * This sketch collects sensor data and outputs it in the format needed
 * for ellipsoid calibration. Works with various IMU sensors.
 * 
 * Compatible with:
 * - MPU6050 (accelerometer only)
 * - MPU9250 (accelerometer + magnetometer)
 * - QMI8658C (accelerometer + gyro)
 * - AK09918C (magnetometer)
 * - LSM6DS3, LSM9DS1, BMI160, etc.
 * 
 * Output format: x y z (tab-separated) sent over Serial
 * 
 * Instructions:
 * 1. Upload this sketch to ESP32
 * 2. Open Serial Monitor (115200 baud) or use screen/minicom
 * 3. Follow on-screen instructions
 * 4. Save Serial output to text file
 * 5. Run calibration: python ellipsoid_calibration.py <datafile.txt>
 */

#include <Wire.h>

// ========== CONFIGURATION ==========
// Adjust these for your specific sensor

// I2C pins (ESP32)
#define I2C_SDA 21
#define I2C_SCL 22

// Sensor I2C address (common examples)
// MPU6050/MPU9250: 0x68 or 0x69
// QMI8658C: 0x6A or 0x6B
// AK09918C: 0x0C
// LSM6DS3: 0x6A or 0x6B
#define SENSOR_ADDR 0x68

// Data collection settings
#define SAMPLE_RATE_HZ 50          // How fast to sample
#define COLLECTION_TIME_MS 60000   // How long to collect (60 seconds)
#define WARMUP_TIME_MS 5000        // Warmup period before collection

// Sensor type
enum SensorType {
  ACCELEROMETER,
  MAGNETOMETER
};

SensorType sensorType = ACCELEROMETER;  // Change this for your sensor

// ========== SENSOR-SPECIFIC FUNCTIONS ==========
// Modify these functions for your specific sensor

// Example for MPU6050 Accelerometer
void initSensor_MPU6050() {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up MPU6050
  Wire.endTransmission(true);
  
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x00);  // ±2g full scale
  Wire.endTransmission(true);
  
  delay(100);
}

void readSensor_MPU6050(float &x, float &y, float &z) {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x3B);  // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDR, 6, true);
  
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  
  // Convert to g (±2g range, 16384 LSB/g)
  x = ax / 16384.0;
  y = ay / 16384.0;
  z = az / 16384.0;
}

// Example for AK09918C Magnetometer
void initSensor_AK09918C() {
  // Soft reset
  Wire.beginTransmission(0x0C);
  Wire.write(0x32);  // CNTL3 register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  
  // Continuous measurement mode, 100Hz
  Wire.beginTransmission(0x0C);
  Wire.write(0x31);  // CNTL2 register
  Wire.write(0x08);  // 100Hz continuous mode
  Wire.endTransmission();
  delay(10);
}

void readSensor_AK09918C(float &x, float &y, float &z) {
  Wire.beginTransmission(0x0C);
  Wire.write(0x10);  // Start at HXL
  Wire.endTransmission(false);
  Wire.requestFrom(0x0C, 6);
  
  int16_t mx = (Wire.read() | (Wire.read() << 8));
  int16_t my = (Wire.read() | (Wire.read() << 8));
  int16_t mz = (Wire.read() | (Wire.read() << 8));
  
  // Convert to µT (0.15 µT/LSB for AK09918C)
  x = mx * 0.15;
  y = my * 0.15;
  z = mz * 0.15;
}

// ========== GENERIC INTERFACE ==========
// These wrap the sensor-specific functions

void initSensor() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz I2C
  
  Serial.println("Initializing sensor...");
  
  // Call sensor-specific init
  // Uncomment the one for your sensor:
  initSensor_MPU6050();
  // initSensor_AK09918C();
  // Add your sensor's init function here
  
  Serial.println("Sensor initialized!");
}

void readSensor(float &x, float &y, float &z) {
  // Call sensor-specific read
  // Uncomment the one for your sensor:
  readSensor_MPU6050(x, y, z);
  // readSensor_AK09918C(x, y, z);
  // Add your sensor's read function here
}

// ========== MAIN PROGRAM ==========

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n========================================");
  Serial.println("SENSOR CALIBRATION DATA COLLECTION");
  Serial.println("========================================\n");
  
  if (sensorType == ACCELEROMETER) {
    Serial.println("Mode: ACCELEROMETER CALIBRATION");
    Serial.println("\nInstructions:");
    Serial.println("1. Keep sensor STATIONARY in each orientation");
    Serial.println("2. Slowly rotate through ALL orientations:");
    Serial.println("   - Place flat (6 faces like a dice)");
    Serial.println("   - Place on edges (12 edges)");
    Serial.println("   - Place on corners (8 corners)");
    Serial.println("3. Hold each orientation for 2-3 seconds");
    Serial.println("4. Collection will run for 60 seconds");
  } else {
    Serial.println("Mode: MAGNETOMETER CALIBRATION");
    Serial.println("\nInstructions:");
    Serial.println("1. MOVE AWAY from metal objects and electronics!");
    Serial.println("2. Move sensor SLOWLY in figure-8 patterns:");
    Serial.println("   - Horizontal figure-8 (yaw)");
    Serial.println("   - Vertical figure-8 (pitch)");
    Serial.println("   - Side figure-8 (roll)");
    Serial.println("3. Cover ALL orientations smoothly");
    Serial.println("4. Collection will run for 60 seconds");
  }
  
  Serial.println("\n----------------------------------------");
  Serial.println("Setting up I2C and sensor...");
  
  initSensor();
  
  Serial.println("\n----------------------------------------");
  Serial.println("Warming up sensor for 5 seconds...");
  Serial.println("(Let sensor temperature stabilize)");
  
  unsigned long warmupStart = millis();
  while (millis() - warmupStart < WARMUP_TIME_MS) {
    float x, y, z;
    readSensor(x, y, z);
    delay(100);
    if (millis() % 1000 < 100) {
      Serial.print(".");
    }
  }
  
  Serial.println("\n\n========================================");
  Serial.println("STARTING DATA COLLECTION!");
  Serial.println("========================================");
  
  if (sensorType == ACCELEROMETER) {
    Serial.println("Begin rotating sensor through all orientations NOW!");
  } else {
    Serial.println("Begin moving sensor in figure-8 patterns NOW!");
  }
  
  Serial.println("\nCopy ALL the lines below to a text file:");
  Serial.println("----------------------------------------");
}

void loop() {
  static unsigned long startTime = millis();
  static unsigned long lastSample = 0;
  static int sampleCount = 0;
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - startTime;
  
  // Check if collection time is over
  if (elapsed > COLLECTION_TIME_MS) {
    if (sampleCount > 0) {
      Serial.println("----------------------------------------");
      Serial.println("DATA COLLECTION COMPLETE!");
      Serial.println("----------------------------------------");
      Serial.print("Collected ");
      Serial.print(sampleCount);
      Serial.println(" samples");
      Serial.println("\nNext steps:");
      Serial.println("1. Copy the data above (between the dashed lines)");
      Serial.println("2. Save to a text file (e.g., sensor_data.txt)");
      Serial.println("3. Run: python ellipsoid_calibration.py sensor_data.txt");
      Serial.println("\nData collection stopped. Reset to collect again.");
      
      sampleCount = 0;  // Prevent repeated printing
    }
    delay(1000);
    return;
  }
  
  // Sample at specified rate
  if (currentTime - lastSample >= (1000 / SAMPLE_RATE_HZ)) {
    lastSample = currentTime;
    
    float x, y, z;
    readSensor(x, y, z);
    
    // Output in tab-separated format
    Serial.print(x, 6);
    Serial.print("\t");
    Serial.print(y, 6);
    Serial.print("\t");
    Serial.println(z, 6);
    
    sampleCount++;
    
    // Progress indicator every 10 samples
    if (sampleCount % 10 == 0 && sampleCount > 0) {
      // Send progress to stderr-like stream (doesn't interfere with data)
      // Note: In Arduino, we can't easily separate stdout/stderr, 
      // so users should manually remove these lines from saved data
      // Or we can use a different serial port for debugging
    }
  }
}

// ========== HELPER FUNCTIONS ==========

// If you need averaging to reduce noise
void readSensorAveraged(float &x, float &y, float &z, int numSamples = 10) {
  float sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < numSamples; i++) {
    float tx, ty, tz;
    readSensor(tx, ty, tz);
    sumX += tx;
    sumY += ty;
    sumZ += tz;
    delay(1);
  }
  
  x = sumX / numSamples;
  y = sumY / numSamples;
  z = sumZ / numSamples;
}

// Scanner to detect I2C devices (useful for debugging)
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
}
