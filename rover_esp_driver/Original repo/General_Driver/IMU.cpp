#include "IMU.h"

void calibrateMagn();
void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float invSqrt(float x);

/******************************************************************************
 * IMU module                                                                 *
 ******************************************************************************/
// #define S_SCL   33
// #define S_SDA   32

AK09918_err_type_t err;

QMI8658 qmi8658_;
AK09918 magnetometer_;

//int16_t offset_x = -12, offset_y = 0, offset_z = 0;
int16_t offset_x = 37, offset_y = -28, offset_z = -12;
int16_t x, y, z;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
//double declination_shenzhen = -3.22;
double declination_melbourne = 11.92;

#define Kp 0.1f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0005f    // integral gain governs rate of convergence of gyroscope biases

float angles[3];
float q0, q1, q2, q3; 

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================
// Target update rate for the AHRS filter (Hz)
#define IMU_TARGET_RATE_HZ    100
#define IMU_TARGET_PERIOD_US  (1000000 / IMU_TARGET_RATE_HZ)  // 10000us = 10ms

// Expected dt for the filter (seconds)
#define IMU_EXPECTED_DT       (1.0f / IMU_TARGET_RATE_HZ)     // 0.01s

// dt clamping bounds (seconds) - prevents numerical instability
#define DT_MIN 0.001f   // 1ms minimum (1000Hz max)
#define DT_MAX 0.050f   // 50ms maximum (20Hz min) - anything slower is problematic

// dt filter coefficient (0-1, lower = more smoothing)
#define DT_FILTER_ALPHA 0.2f

// Track timing for rate limiting
static unsigned long imu_last_update_us = 0;
static float filtered_dt = IMU_EXPECTED_DT;

void imuInit()
{ 
    if (qmi8658_.begin() == 0)
	      Serial.println("qmi8658_init fail");

    if (magnetometer_.initialize())
        Serial.println("AK09918_init fail") ;
    magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
    err = magnetometer_.isDataReady();
    int retry_times = 0;
    while (err != AK09918_ERR_OK) {
        Serial.println(err);
        Serial.println("Waiting Sensor");
        delay(100);
        magnetometer_.reset();
        delay(100);
        magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
        err = magnetometer_.isDataReady();
        retry_times ++;
        if (retry_times > 10) {
          break;
        }
    }
    q0 = 1.0f;  
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    // Initialize timing
    imu_last_update_us = micros();
    filtered_dt = IMU_EXPECTED_DT;
}

// Returns true if enough time has passed and we should update
// This provides rate limiting for consistent filter behavior
bool imuShouldUpdate() {
  unsigned long now = micros();
  unsigned long elapsed = now - imu_last_update_us;
  
  // Handle micros() overflow (happens every ~70 minutes)
  if (now < imu_last_update_us) {
      // Overflow occurred - just update
      return true;
  }
  
  return (elapsed >= IMU_TARGET_PERIOD_US);
}

void imuDataGet(EulerAngles *pstAngles, 
                IMU_ST_SENSOR_DATA_FLOAT *pstGyroRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData)
{
  if (!imuShouldUpdate()) {}
  float  acc[3], gyro[3];
  float MotionVal[9];

  magnetometer_.getData(&x, &y, &z);

  extern int16_t mx_raw, my_raw, mz_raw;
  mx_raw = x;
  my_raw = y;
  mz_raw = z;

  // Auto-calibration: track min/max to find hard-iron offsets
  extern bool MAG_AUTO_CAL;
  extern int16_t mag_min[3], mag_max[3], mag_offset_auto[3];
  extern int mag_cal_samples;
  extern bool mag_cal_valid;
  extern int MAG_CAL_MIN_SAMPLES;

  if (MAG_AUTO_CAL) {
    // Update min/max tracking
    if (x < mag_min[0]) mag_min[0] = x;
    if (x > mag_max[0]) mag_max[0] = x;
    if (y < mag_min[1]) mag_min[1] = y;
    if (y > mag_max[1]) mag_max[1] = y;
    if (z < mag_min[2]) mag_min[2] = z;
    if (z > mag_max[2]) mag_max[2] = z;
    
    mag_cal_samples++;
    
    // Calculate auto offsets (center of min/max range)
    mag_offset_auto[0] = (mag_min[0] + mag_max[0]) / 2;
    mag_offset_auto[1] = (mag_min[1] + mag_max[1]) / 2;
    mag_offset_auto[2] = (mag_min[2] + mag_max[2]) / 2;
    
    // Check if we have enough spread to consider calibration valid
    int16_t spread_x = mag_max[0] - mag_min[0];
    int16_t spread_y = mag_max[1] - mag_min[1];
    // Need at least 20 µT spread on X and Y axes (indicates rotation occurred)
    mag_cal_valid = (mag_cal_samples >= MAG_CAL_MIN_SAMPLES) && 
                    (spread_x > 30) && (spread_y > 30);
    
    if (mag_cal_valid) {
      pstMagnRawData->s16X = x - mag_offset_auto[0];
      pstMagnRawData->s16Y = y - mag_offset_auto[1];
      pstMagnRawData->s16Z = z - mag_offset_auto[2];
    } else {
      // Fall back to hardcoded offsets until calibration is valid
      pstMagnRawData->s16X = x - offset_x;
      pstMagnRawData->s16Y = y - offset_y;
      pstMagnRawData->s16Z = z - offset_z;
    }
  } else {

      pstMagnRawData->s16X = x- offset_x;
      pstMagnRawData->s16Y = y- offset_y;
      pstMagnRawData->s16Z = z- offset_z;
  }

  qmi8658_.read_sensor_data(acc,gyro);

  MotionVal[0]=gyro[0];
  MotionVal[1]=gyro[1];
  MotionVal[2]=gyro[2];
  MotionVal[3]=acc[0];
  MotionVal[4]=acc[1];
  MotionVal[5]=acc[2];
  MotionVal[6]=pstMagnRawData->s16X;
  MotionVal[7]=pstMagnRawData->s16Y;
  MotionVal[8]=pstMagnRawData->s16Z;
  if (imuShouldUpdate()) {
    imuAHRSupdate((float)MotionVal[0] * 0.0175f, (float)MotionVal[1] * 0.0175f, (float)MotionVal[2] * 0.0175f,
                (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5], 
                (float)MotionVal[6], (float)MotionVal[7], MotionVal[8]);
  }
  pstAngles->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  pstAngles->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
  //pstAngles->yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3; 
  pstAngles->yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3 + declination_melbourne;

  pstGyroRawData->X = gyro[0];
  pstGyroRawData->Y = gyro[1];
  pstGyroRawData->Z = gyro[2];

  pstAccelRawData->X = acc[0];
  pstAccelRawData->Y = acc[1];
  pstAccelRawData->Z = acc[2];

  return;  
}

void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
  // TIMING MEASUREMENT
  //static unsigned long last_call_time = 0;
  //static float measured_halfT = 0.0035f;  // Initial guess
  
  //unsigned long now = micros();
  //float raw_dt;
  //if (last_call_time != 0) {
  //  float dt = (now - last_call_time) / 1000000.0f;  // Convert to seconds
  //  measured_halfT = dt / 2.0f;
  //}
  //last_call_time = now;
  
  unsigned long now = micros();
  float raw_dt;
  
  // Calculate raw dt, handling overflow
  if (now >= imu_last_update_us) {
    raw_dt = (now - imu_last_update_us) / 1000000.0f;
  } else {
    // micros() overflow - use expected dt as fallback
    raw_dt = IMU_EXPECTED_DT;
  }
  
  // Update timestamp for next iteration
  imu_last_update_us = now;
  
  // Clamp dt to reasonable bounds BEFORE filtering
  // This prevents a single bad reading from corrupting the filter
  if (raw_dt < DT_MIN) raw_dt = DT_MIN;
  if (raw_dt > DT_MAX) raw_dt = DT_MAX;
  
  // Apply exponential moving average filter to smooth dt
  // filtered_dt = alpha * new_sample + (1-alpha) * old_filtered
  filtered_dt = DT_FILTER_ALPHA * raw_dt + (1.0f - DT_FILTER_ALPHA) * filtered_dt;
  
  // Use the filtered, clamped dt
  float halfT = filtered_dt / 2.0f;

  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;  // FIXED: made static!
  float ex, ey, ez;
  //float halfT = measured_halfT;

  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;   
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;          

  // Normalize accelerometer
  norm = invSqrt(ax * ax + ay * ay + az * az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  // Estimated direction of gravity
  vx = 2 * (q1q3 - q0q2);
  vy = 2 * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  // Check if magnetometer data is valid (not zeros from startup)
  float mag_magnitude_sq = mx*mx + my*my + mz*mz;
  
  if (mag_magnitude_sq > 100.0f) {
    // MAGNETOMETER IS VALID - use full 9-DOF fusion
    
    // Normalize magnetometer
    norm = invSqrt(mag_magnitude_sq);          
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    // Compute reference direction of magnetic flux
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);         
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;     

    // Estimated direction of magnetic flux
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);  

    // Error is sum of cross product between reference and measured direction
    // Includes BOTH accelerometer AND magnetometer correction
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  } 
  else {
    // MAGNETOMETER INVALID - use 6-DOF fusion (accelerometer only)
    // This prevents garbage magnetometer data from corrupting the estimate
    
    // Error from accelerometer only (no magnetometer contribution)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
  }

  // Apply PI controller
  if(ex != 0.0f || ey != 0.0f || ez != 0.0f)
  {
    exInt = exInt + ex * Ki * halfT;
    eyInt = eyInt + ey * Ki * halfT;  
    ezInt = ezInt + ez * Ki * halfT;

    const float INTEGRAL_LIMIT = 0.1f;
    if (exInt > 0.1f) exInt = INTEGRAL_LIMIT;
    if (exInt < -INTEGRAL_LIMIT) exInt = -INTEGRAL_LIMIT;
    if (eyInt > INTEGRAL_LIMIT) eyInt = INTEGRAL_LIMIT;
    if (eyInt < -INTEGRAL_LIMIT) eyInt = -INTEGRAL_LIMIT;
    if (ezInt > INTEGRAL_LIMIT) ezInt = INTEGRAL_LIMIT;
    if (ezInt < -INTEGRAL_LIMIT) ezInt = -INTEGRAL_LIMIT;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
  }

  // Integrate quaternion rate
  q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;  

  // Normalize quaternion
  norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
}

float invSqrt(float x) 
{
  float halfx = 0.5f * x;
  float y = x;
  
  long i = *(long*)&y;                //get bits for floating value
  i = 0x5f3759df - (i >> 1);          //gives initial guss you
  y = *(float*)&i;                    //convert bits back to float
  y = y * (1.5f - (halfx * y * y));   //newtop step, repeating increases accuracy
  
  return y;
}

void calibrateMagn(void)
{
  int16_t temp[9];
  Serial.printf("keep 10dof-imu device horizontal and it will read x y z axis offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[0] = x;
  temp[1] = y;
  temp[2] = z;
  
  Serial.printf("rotate z axis 180 degrees and it will read all axises offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[3] = x;
  temp[4] = y;
  temp[5] = z;

  Serial.printf("flip 10dof-imu device and keep it horizontal and it will read all axises offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[6] = x;
  temp[7] = y;
  temp[8] = z;

  offset_x = (temp[0]+temp[3])/2;
  offset_y = (temp[1]+temp[4])/2;
  offset_z = (temp[5]+temp[8])/2;
}
