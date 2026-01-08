/*
 * Hardcoded Magnetometer Calibration
 * 
 * CRITICAL: The AK09918C magnetometer axes may not align with the QMI8658C IMU.
 * This is the #1 cause of yaw drift with Madgwick filters!
 *
 * For Melbourne, Australia:
 *   - Magnetic dip angle: ~-65° (field points DOWN in southern hemisphere)
 *   - Total field: ~57 µT
 *
 * If your dip angle shows POSITIVE when it should be NEGATIVE, the Z-axis
 * is inverted. Set MAG_AXIS_Z_SIGN = -1 to fix.
 * 
 * Calibration performed with MotionCal tool.
 */

#ifndef HARDCODED_CALIBRATION_H
#define HARDCODED_CALIBRATION_H

#include "sensors.h"

// Enable/disable hardcoded calibration
// #define USE_HARDCODED_CAL       true
// #define USE_ACCELEROMETER_CAL   true
// #define USE_GYROSCOPE_CALIB     true

// Ue the preferences.h library - load the value from flash
extern bool USE_HARDCODED_CAL;
extern bool USE_ACCELEROMETER_CAL;
extern bool USE_GYROSCOPE_CALIB;

// =============================================================================
// Hard Iron Offsets (in µT)
// =============================================================================
// These compensate for permanent magnetic fields from nearby components

#define MAG_HARD_IRON_X      23.06f // 34.00f
#define MAG_HARD_IRON_Y     -35.88f //-35.76f
#define MAG_HARD_IRON_Z     -5.39f //-31.99f

// =============================================================================
// Soft Iron Correction Matrix
// =============================================================================
// Compensates for distortions in the magnetic field

#define MAG_SOFT_IRON_00    1.009f //1.025f
#define MAG_SOFT_IRON_01    0.007f //0.023f
#define MAG_SOFT_IRON_02    0.011f //-0.057f
#define MAG_SOFT_IRON_10    0.007f //0.023f
#define MAG_SOFT_IRON_11    1.020f //1.066f
#define MAG_SOFT_IRON_12   -0.003f //-0.004f
#define MAG_SOFT_IRON_20    0.011f //-0.057f
#define MAG_SOFT_IRON_21   -0.033f //-0.004f
#define MAG_SOFT_IRON_22    0.973f //0.919f

// Expected field strength after calibration
#define MAG_FIELD_STRENGTH  57.51f // 55.91f  // µT



// =============================================================================
// Apply Calibration Function
// =============================================================================

inline void apply_hardcoded_calibration(AK09918C& mag) {
#if USE_HARDCODED_CAL
    // Hard iron correction
    mag.set_hard_iron(
        MAG_HARD_IRON_X,
        MAG_HARD_IRON_Y,
        MAG_HARD_IRON_Z
    );
    
    // Soft iron correction
    const float soft_iron[3][3] = {
        { MAG_SOFT_IRON_00, MAG_SOFT_IRON_01, MAG_SOFT_IRON_02 },
        { MAG_SOFT_IRON_10, MAG_SOFT_IRON_11, MAG_SOFT_IRON_12 },
        { MAG_SOFT_IRON_20, MAG_SOFT_IRON_21, MAG_SOFT_IRON_22 }
    };
    mag.set_soft_iron(soft_iron);
    
#endif
}

// =============================================================================
// Accelerometer calibration - Extracted from python script
// Inspired by Magneto (code in AccelGyroCalib folder)
//
// Starting calibration...
// ----------------------------------------------------------------------
// Calibrating with 178 samples...
// Data range: X[-0.96, 1.03], Y[-1.06, 0.96], Z[-1.00, 0.99]
// Largest eigenvalue: 0.019405
// Bias (offset): [0.0345, -0.0000, -0.0061]
// Measured field magnitude: 0.7595
// Reference field strength: 1.0000
// Scaling factor: 1.3167

// Calibration Matrix:
// [[ 1.00384913e+00 -2.01243186e-03 -1.39028724e-04]
//  [-2.01243186e-03  9.94378533e-01 -6.27564895e-04]
//  [-1.39028724e-04 -6.27564895e-04  1.00312631e+00]]

// ============================================================
// CALIBRATION QUALITY METRICS
// ============================================================
// Raw data magnitude:        0.9988 ± 0.0189
// Calibrated data magnitude: 1.0000 ± 0.0091
// Improvement in std dev:    52.0%
// ============================================================

#define ACC_CALIB_BIAS_X     0.0345f
#define ACC_CALIB_BIAS_Y     0.0000f
#define ACC_CALIB_BIAS_Z    -0.0061f
#define ACC_CALIB_00         1.00384913e+00f
#define ACC_CALIB_01        -2.01243186e-03f 
#define ACC_CALIB_02        -1.39028724e-04f
#define ACC_CALIB_10        -2.01243186e-03f
#define ACC_CALIB_11         9.94378533e-01f
#define ACC_CALIB_12        -6.27564895e-04f
#define ACC_CALIB_20        -1.39028724e-04f
#define ACC_CALIB_21        -6.27564895e-04f
#define ACC_CALIB_22         1.00312631e+00f


// =============================================================================
// Apply Calibration Function
// =============================================================================

inline void apply_accelerometer_calibration(QMI8658C& imu) {
#if USE_ACCELEROMETER_CAL
    float acc_calib_matrix[3][3] = {
        { ACC_CALIB_00, ACC_CALIB_01, ACC_CALIB_02 },
        { ACC_CALIB_10, ACC_CALIB_11, ACC_CALIB_12 },
        { ACC_CALIB_20, ACC_CALIB_21, ACC_CALIB_22 }
    };

    imu.set_accel_calib(
        ACC_CALIB_BIAS_X,
        ACC_CALIB_BIAS_Y,
        ACC_CALIB_BIAS_Z,
        acc_calib_matrix
    );
#endif
}

// =============================================================================
//  Gyroscope Calibration
// =============================================================================

// Enable/disable hardcoded calibration of accelerometer


#define GYRO_CALIB_BIAS_X     2.222135347351610f 
#define GYRO_CALIB_BIAS_Y    -0.527376674651761f 
#define GYRO_CALIB_BIAS_Z    29.260097817407505f

inline void apply_gyroscope_calibration(QMI8658C& imu) {
#if USE_GYROSCOPE_CALIB
    imu.set_gyro_calib(
        GYRO_CALIB_BIAS_X,
        GYRO_CALIB_BIAS_Y,
        GYRO_CALIB_BIAS_Z
    );
#endif
}

#endif // HARDCODED_CALIBRATION_H
