#include"IMU.h"

// define GPIOs for IIC.
EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;
float temp;


void imu_init() {
	imuInit();
}


void updateIMUData() {
	imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
	//Serial.printf("MAG_RAW: %d, %d, %d\n", 
  //                (int)stMagnRawData.s16X, (int)stMagnRawData.s16Y, (int)stMagnRawData.s16Z);

	ax = stAccelRawData.X;
	ay = stAccelRawData.Y;
	az = stAccelRawData.Z;

	mx = stMagnRawData.s16X;
	my = stMagnRawData.s16Y;
	mz = stMagnRawData.s16Z;

	gx = stGyroRawData.X;
	gy = stGyroRawData.Y;
	gz = stGyroRawData.Z;

	icm_roll = stAngles.roll;
	icm_pitch = stAngles.pitch;
	icm_yaw = stAngles.yaw;

  temp = temperatureRead();
}


void imuCalibration() {

}


void getIMUData() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

	jsonInfoHttp["r"] = icm_roll;
	jsonInfoHttp["p"] = icm_pitch;
	jsonInfoHttp["y"] = icm_yaw;

	jsonInfoHttp["ax"] = ax;
	jsonInfoHttp["ay"] = ay;
	jsonInfoHttp["az"] = az;

	jsonInfoHttp["gx"] = gx;
	jsonInfoHttp["gy"] = gy;
	jsonInfoHttp["gz"] = gz;

	jsonInfoHttp["mx"] = mx;
	jsonInfoHttp["my"] = my;
	jsonInfoHttp["mz"] = mz;

	jsonInfoHttp["temp"] = temp;
  // Debug: include raw magnetometer data when IMU_DEBUG_RAW is enabled
  if (IMU_DEBUG_RAW) {
    jsonInfoHttp["mx_raw"] = mx_raw;
    jsonInfoHttp["my_raw"] = my_raw;
    jsonInfoHttp["mz_raw"] = mz_raw;
    // Also include computed magnitude for quick health check
    jsonInfoHttp["mag_raw"] = (int)sqrt(mx_raw*mx_raw + my_raw*my_raw + mz_raw*mz_raw);
    jsonInfoHttp["mag_cal"] = (int)sqrt(mx*mx + my*my + mz*mz);
		// Auto-calibration status
    jsonInfoHttp["mcal_valid"] = mag_cal_valid ? 1 : 0;
    jsonInfoHttp["mcal_n"] = mag_cal_samples;
    jsonInfoHttp["moff_x"] = mag_offset_auto[0];
    jsonInfoHttp["moff_y"] = mag_offset_auto[1];
    jsonInfoHttp["moff_z"] = mag_offset_auto[2];
  }
	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}

void getIMUOffset() {

}

void setIMUOffset(int16_t inputX, int16_t inputY, int16_t inputZ) {

}