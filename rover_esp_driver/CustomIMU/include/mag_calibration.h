#ifndef MAG_CALIBRATION_H
#define MAG_CALIBRATION_H

#include <Arduino.h>
#include <Preferences.h>

// =============================================================================
// Magnetometer Calibration Data Structure
// =============================================================================

struct MagCalibration {
    // Hard iron offsets (uT)
    float hard_iron[3];

    // Soft iron matrix (inverse)
    float soft_iron[3][3];

    // Magnetic field strength (uT)
    float field_strength;

    // Valid flag
    bool valid;
};

// =============================================================================
// MotionCal Binary Packet Format
// =============================================================================
// Total: 68 bytes
// - Bytes 0-1:   Signature (117, 84)
// - Bytes 2-13:  Accelerometer offsets (3 floats, unused)
// - Bytes 14-25: Gyroscope offsets (3 floats, unused)
// - Bytes 26-37: Magnetometer hard iron offsets (3 floats)
// - Bytes 38-41: Magnetic field strength (1 float)
// - Bytes 42-53: Soft iron diagonal (3 floats: [0][0], [1][1], [2][2])
// - Bytes 54-65: Soft iron off-diagonal (3 floats: [0][1], [0][2], [1][2])
// - Bytes 66-67: CRC16
// =============================================================================

#define MOTIONCAL_PACKET_SIZE 68
#define MOTIONCAL_SIGNATURE_0 117
#define MOTIONCAL_SIGNATURE_1 84

class MagCalibrationReceiver {
public:
    MagCalibrationReceiver();

    // Process incoming serial byte, returns true if calibration was received
    bool process_byte(uint8_t byte);

    // Get the received calibration data
    const MagCalibration& get_calibration() const { return cal_; }

    // Save calibration to NVS
    bool save_to_nvs();

    // Load calibration from NVS
    bool load_from_nvs();

    // Print calibration values
    void print_calibration();

private:
    uint8_t buffer_[MOTIONCAL_PACKET_SIZE];
    uint8_t buf_idx_;
    MagCalibration cal_;

    // CRC16 calculation
    uint16_t crc16(uint16_t crc, uint8_t data);

    // Parse the received packet
    bool parse_packet();

    // Extract float from buffer (little-endian)
    float extract_float(const uint8_t* data);
};

#endif // MAG_CALIBRATION_H
