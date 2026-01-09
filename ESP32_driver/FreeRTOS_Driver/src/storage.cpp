#include "protocol.h"
#include "storage.h"

// =============================================================================
// Preferences
// =============================================================================

Preferences preferences;
bool NVS_CALIBRATION;
bool STREAM_ON_BOOT;

void preferences_init() {
    preferences.begin(CONFIGKEY, false);
    // CALIBRATION
    if (!preferences.isKey(CALIBKEY)) {
        out_system("NVS", "NVS does not contain USECALILB flag");
        NVS_CALIBRATION = false;
        preferences.putBool(CALIBKEY, NVS_CALIBRATION); 
    } else {
        NVS_CALIBRATION = preferences.getBool(CALIBKEY);
        out_system("NVS", "NVS contains CALIB flag");
        out_system("NVS", NVS_CALIBRATION);
    }
    // STREAM ON BOOT
    if (!preferences.isKey(STREAMONBOOT)) {
        out_system("NVS", "NVS does not contain STREAMONBOOT flag");
        STREAM_ON_BOOT = false;
        preferences.putBool(STREAMONBOOT, STREAM_ON_BOOT); 
    } else {
        STREAM_ON_BOOT = preferences.getBool(STREAMONBOOT);
        out_system("NVS", "NVS contains STREAMONBOOT flag");
        out_system("NVS", STREAM_ON_BOOT);
        protocol_set_streaming(STREAM_ON_BOOT);
  }
  preferences.end();
}

void streamboot_set_flag(bool enabled){
    preferences.begin(CONFIGKEY, false);
    preferences.putBool(STREAMONBOOT, enabled);
    preferences.end();
}

bool streamboot_get_flag(){
    preferences.begin(CONFIGKEY, true);
    bool retval;
    retval = preferences.getBool(STREAMONBOOT);
    preferences.end();
    return retval;
}

void calib_set_flag(bool enabled){
    preferences.begin(CONFIGKEY, false);
    preferences.putBool(CALIBKEY, enabled);
    preferences.end();
}

bool calib_get_flag(){
    preferences.begin(CONFIGKEY, true);
    bool retval;
    retval = preferences.getBool(CALIBKEY);
    preferences.end();
    return retval;
}
