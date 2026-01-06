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

// =============================================================================
// Hard Iron Offsets (in µT)
// =============================================================================
// These compensate for permanent magnetic fields from nearby components

#define MAG_HARD_IRON_X     34.00f
#define MAG_HARD_IRON_Y    -35.76f
#define MAG_HARD_IRON_Z    -31.99f

// =============================================================================
// Soft Iron Correction Matrix
// =============================================================================
// Compensates for distortions in the magnetic field

#define MAG_SOFT_IRON_00    1.025f
#define MAG_SOFT_IRON_01    0.023f
#define MAG_SOFT_IRON_02   -0.057f
#define MAG_SOFT_IRON_10    0.023f
#define MAG_SOFT_IRON_11    1.066f
#define MAG_SOFT_IRON_12   -0.004f
#define MAG_SOFT_IRON_20   -0.057f
#define MAG_SOFT_IRON_21   -0.004f
#define MAG_SOFT_IRON_22    0.919f

// Expected field strength after calibration
#define MAG_FIELD_STRENGTH  55.91f  // µT

// Enable/disable hardcoded calibration
#define USE_HARDCODED_CAL   true

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

#endif // HARDCODED_CALIBRATION_H
