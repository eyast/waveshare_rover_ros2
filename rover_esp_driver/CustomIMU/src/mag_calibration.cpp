#include "mag_calibration.h"

MagCalibrationReceiver::MagCalibrationReceiver() : buf_idx_(0) {
    memset(&cal_, 0, sizeof(cal_));
    // Initialize soft iron to identity matrix
    cal_.soft_iron[0][0] = 1.0f;
    cal_.soft_iron[1][1] = 1.0f;
    cal_.soft_iron[2][2] = 1.0f;
    cal_.field_strength = 50.0f;  // Default Earth's field ~50 uT
    cal_.valid = false;
}

bool MagCalibrationReceiver::process_byte(uint8_t byte) {
    // Check for signature at start
    if (buf_idx_ == 0) {
        if (byte == MOTIONCAL_SIGNATURE_0) {
            buffer_[buf_idx_++] = byte;
        }
        return false;
    }

    if (buf_idx_ == 1) {
        if (byte == MOTIONCAL_SIGNATURE_1) {
            buffer_[buf_idx_++] = byte;
        } else {
            // Reset if second signature byte doesn't match
            buf_idx_ = 0;
        }
        return false;
    }

    // Accumulate bytes
    buffer_[buf_idx_++] = byte;

    // Check if we have a complete packet
    if (buf_idx_ >= MOTIONCAL_PACKET_SIZE) {
        buf_idx_ = 0;
        return parse_packet();
    }

    return false;
}

uint16_t MagCalibrationReceiver::crc16(uint16_t crc, uint8_t data) {
    crc ^= data;
    for (int i = 0; i < 8; ++i) {
        if (crc & 1) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc = crc >> 1;
        }
    }
    return crc;
}

float MagCalibrationReceiver::extract_float(const uint8_t* data) {
    union {
        float f;
        uint32_t n;
    } conv;
    conv.n = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    return conv.f;
}

bool MagCalibrationReceiver::parse_packet() {
    // Verify CRC
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 66; i++) {
        crc = crc16(crc, buffer_[i]);
    }
    uint16_t received_crc = buffer_[66] | (buffer_[67] << 8);

    if (crc != received_crc) {
        Serial.println("Calibration CRC error!");
        return false;
    }

    // Parse the packet
    // Bytes 2-13: Accel offsets (skip)
    // Bytes 14-25: Gyro offsets (skip)
    // Bytes 26-37: Mag hard iron offsets
    cal_.hard_iron[0] = extract_float(&buffer_[26]);
    cal_.hard_iron[1] = extract_float(&buffer_[30]);
    cal_.hard_iron[2] = extract_float(&buffer_[34]);

    // Byte 38-41: Field strength
    cal_.field_strength = extract_float(&buffer_[38]);

    // Bytes 42-53: Soft iron diagonal
    cal_.soft_iron[0][0] = extract_float(&buffer_[42]);
    cal_.soft_iron[1][1] = extract_float(&buffer_[46]);
    cal_.soft_iron[2][2] = extract_float(&buffer_[50]);

    // Bytes 54-65: Soft iron off-diagonal
    cal_.soft_iron[0][1] = extract_float(&buffer_[54]);
    cal_.soft_iron[1][0] = cal_.soft_iron[0][1];  // Symmetric
    cal_.soft_iron[0][2] = extract_float(&buffer_[58]);
    cal_.soft_iron[2][0] = cal_.soft_iron[0][2];  // Symmetric
    cal_.soft_iron[1][2] = extract_float(&buffer_[62]);
    cal_.soft_iron[2][1] = cal_.soft_iron[1][2];  // Symmetric

    cal_.valid = true;

    Serial.println("\n=== Calibration Received! ===");
    print_calibration();

    return true;
}

bool MagCalibrationReceiver::save_to_nvs() {
    Preferences prefs;
    if (!prefs.begin("magcal", false)) {
        Serial.println("Failed to open NVS for writing");
        return false;
    }

    prefs.putFloat("hi_x", cal_.hard_iron[0]);
    prefs.putFloat("hi_y", cal_.hard_iron[1]);
    prefs.putFloat("hi_z", cal_.hard_iron[2]);
    prefs.putFloat("field", cal_.field_strength);

    prefs.putFloat("si_00", cal_.soft_iron[0][0]);
    prefs.putFloat("si_01", cal_.soft_iron[0][1]);
    prefs.putFloat("si_02", cal_.soft_iron[0][2]);
    prefs.putFloat("si_11", cal_.soft_iron[1][1]);
    prefs.putFloat("si_12", cal_.soft_iron[1][2]);
    prefs.putFloat("si_22", cal_.soft_iron[2][2]);

    prefs.putBool("valid", true);

    prefs.end();
    Serial.println("Calibration saved to NVS");
    return true;
}

bool MagCalibrationReceiver::load_from_nvs() {
    Preferences prefs;
    if (!prefs.begin("magcal", true)) {  // true = read-only
        Serial.println("Failed to open NVS for reading");
        return false;
    }

    if (!prefs.getBool("valid", false)) {
        prefs.end();
        Serial.println("No valid calibration in NVS");
        return false;
    }

    cal_.hard_iron[0] = prefs.getFloat("hi_x", 0.0f);
    cal_.hard_iron[1] = prefs.getFloat("hi_y", 0.0f);
    cal_.hard_iron[2] = prefs.getFloat("hi_z", 0.0f);
    cal_.field_strength = prefs.getFloat("field", 50.0f);

    cal_.soft_iron[0][0] = prefs.getFloat("si_00", 1.0f);
    cal_.soft_iron[0][1] = prefs.getFloat("si_01", 0.0f);
    cal_.soft_iron[0][2] = prefs.getFloat("si_02", 0.0f);
    cal_.soft_iron[1][0] = cal_.soft_iron[0][1];
    cal_.soft_iron[1][1] = prefs.getFloat("si_11", 1.0f);
    cal_.soft_iron[1][2] = prefs.getFloat("si_12", 0.0f);
    cal_.soft_iron[2][0] = cal_.soft_iron[0][2];
    cal_.soft_iron[2][1] = cal_.soft_iron[1][2];
    cal_.soft_iron[2][2] = prefs.getFloat("si_22", 1.0f);

    cal_.valid = true;

    prefs.end();
    Serial.println("Calibration loaded from NVS");
    print_calibration();
    return true;
}

void MagCalibrationReceiver::print_calibration() {
    Serial.println("Hard Iron Offsets (uT):");
    Serial.print("  X: "); Serial.println(cal_.hard_iron[0], 2);
    Serial.print("  Y: "); Serial.println(cal_.hard_iron[1], 2);
    Serial.print("  Z: "); Serial.println(cal_.hard_iron[2], 2);

    Serial.print("Field Strength: "); Serial.print(cal_.field_strength, 2); Serial.println(" uT");

    Serial.println("Soft Iron Matrix:");
    for (int i = 0; i < 3; i++) {
        Serial.print("  ");
        for (int j = 0; j < 3; j++) {
            Serial.print(cal_.soft_iron[i][j], 4);
            Serial.print(" ");
        }
        Serial.println();
    }
}
