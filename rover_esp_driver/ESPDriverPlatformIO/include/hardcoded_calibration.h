#ifndef HARDCODED_CALIBRATION_H
#define HARDCODED_CALIBRATION_H

#include "ak09918c.h"

// =============================================================================
// Hardcoded Magnetometer Calibration
// =============================================================================
// Calibration performed with MotionCal on 2024-12-23
// Fit Error: 1.4%, Gaps: 5.1%, Variance: 1.4%, Wobble: 3.9%
// =============================================================================

// Hard iron offsets (uT) - compensates for permanent magnetic fields
#define HARDCODED_MAG_OFFSET_X  25.15f
#define HARDCODED_MAG_OFFSET_Y -56.42f
#define HARDCODED_MAG_OFFSET_Z -27.62f

// Soft iron matrix - compensates for magnetic field distortion
// This is a symmetric 3x3 matrix
#define HARDCODED_SOFT_IRON_00  1.039f
#define HARDCODED_SOFT_IRON_01  0.013f
#define HARDCODED_SOFT_IRON_02 -0.061f
#define HARDCODED_SOFT_IRON_10  0.013f  // Same as 01 (symmetric)
#define HARDCODED_SOFT_IRON_11  1.061f
#define HARDCODED_SOFT_IRON_12  0.003f
#define HARDCODED_SOFT_IRON_20 -0.061f  // Same as 02 (symmetric)
#define HARDCODED_SOFT_IRON_21  0.003f  // Same as 12 (symmetric)
#define HARDCODED_SOFT_IRON_22  0.911f

// Expected magnetic field strength (uT)
#define HARDCODED_FIELD_STRENGTH 56.17f

// =============================================================================
// Function to apply hardcoded calibration
// =============================================================================

// Set to false to disable hardcoded calibration (for comparison testing)
#define USE_HARDCODED_CALIBRATION true

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

    Serial.println("Hardcoded magnetometer calibration applied:");
    Serial.print("  Hard Iron: ");
    Serial.print(HARDCODED_MAG_OFFSET_X); Serial.print(", ");
    Serial.print(HARDCODED_MAG_OFFSET_Y); Serial.print(", ");
    Serial.println(HARDCODED_MAG_OFFSET_Z);
    Serial.print("  Field Strength: ");
    Serial.print(HARDCODED_FIELD_STRENGTH);
    Serial.println(" uT");
#else
    Serial.println("Hardcoded calibration DISABLED");
#endif
}

#endif // HARDCODED_CALIBRATION_H
