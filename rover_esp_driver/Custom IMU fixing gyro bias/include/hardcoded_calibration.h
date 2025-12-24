#ifndef HARDCODED_CALIBRATION_H
#define HARDCODED_CALIBRATION_H

#include "ak09918c.h"

// =============================================================================
// Magnetometer Axis Alignment
// =============================================================================
//
// CRITICAL: The AK09918C magnetometer axes may not align with the QMI8658C IMU.
// This is the #1 cause of yaw drift with Madgwick filters!
//
// For Melbourne, Australia:
//   - Magnetic dip angle: ~-65° (field points DOWN in southern hemisphere)
//   - Total field: ~57 µT
//
// If your dip angle shows POSITIVE when it should be NEGATIVE, the Z-axis
// is inverted. Set MAG_AXIS_Z_SIGN = -1 to fix.
//
// =============================================================================

// Axis sign corrections: set to -1 to invert that axis
#define MAG_AXIS_X_SIGN    1
#define MAG_AXIS_Y_SIGN    1
#define MAG_AXIS_Z_SIGN   -1    // ← INVERTED for your hardware!

// =============================================================================
// Hardcoded Magnetometer Calibration
// =============================================================================
// Calibration performed with MotionCal
// =============================================================================

#define HARDCODED_MAG_OFFSET_X  25.15f
#define HARDCODED_MAG_OFFSET_Y -56.42f
#define HARDCODED_MAG_OFFSET_Z -27.62f

#define HARDCODED_SOFT_IRON_00  1.039f
#define HARDCODED_SOFT_IRON_01  0.013f
#define HARDCODED_SOFT_IRON_02 -0.061f
#define HARDCODED_SOFT_IRON_10  0.013f
#define HARDCODED_SOFT_IRON_11  1.061f
#define HARDCODED_SOFT_IRON_12  0.003f
#define HARDCODED_SOFT_IRON_20 -0.061f
#define HARDCODED_SOFT_IRON_21  0.003f
#define HARDCODED_SOFT_IRON_22  0.911f

#define HARDCODED_FIELD_STRENGTH 56.17f

#define USE_HARDCODED_CALIBRATION true

// =============================================================================
// Apply calibration function
// =============================================================================

inline void apply_hardcoded_calibration(AK09918C& mag) {
#if USE_HARDCODED_CALIBRATION
    // Apply hard iron correction
    mag.set_hard_iron(
        HARDCODED_MAG_OFFSET_X,
        HARDCODED_MAG_OFFSET_Y,
        HARDCODED_MAG_OFFSET_Z
    );

    // Apply soft iron correction
    const float soft_iron[3][3] = {
        { HARDCODED_SOFT_IRON_00, HARDCODED_SOFT_IRON_01, HARDCODED_SOFT_IRON_02 },
        { HARDCODED_SOFT_IRON_10, HARDCODED_SOFT_IRON_11, HARDCODED_SOFT_IRON_12 },
        { HARDCODED_SOFT_IRON_20, HARDCODED_SOFT_IRON_21, HARDCODED_SOFT_IRON_22 }
    };
    mag.set_soft_iron(soft_iron);

    // Apply axis sign corrections (CRITICAL for IMU/Mag alignment!)
    mag.set_axis_signs(MAG_AXIS_X_SIGN, MAG_AXIS_Y_SIGN, MAG_AXIS_Z_SIGN);

    Serial.println("Hardcoded magnetometer calibration applied:");
    Serial.print("  Hard Iron: ");
    Serial.print(HARDCODED_MAG_OFFSET_X); Serial.print(", ");
    Serial.print(HARDCODED_MAG_OFFSET_Y); Serial.print(", ");
    Serial.println(HARDCODED_MAG_OFFSET_Z);
    Serial.print("  Axis Signs: X=");
    Serial.print(MAG_AXIS_X_SIGN); Serial.print(", Y=");
    Serial.print(MAG_AXIS_Y_SIGN); Serial.print(", Z=");
    Serial.println(MAG_AXIS_Z_SIGN);
    Serial.print("  Field Strength: ");
    Serial.print(HARDCODED_FIELD_STRENGTH);
    Serial.println(" uT");
#else
    Serial.println("Hardcoded calibration DISABLED");
#endif
}

#endif // HARDCODED_CALIBRATION_H
