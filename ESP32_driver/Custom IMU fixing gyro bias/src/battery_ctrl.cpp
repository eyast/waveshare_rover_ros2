#include "battery_ctrl.h"

INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0; 
bool ina219_overflow = false;

void ina219_init(){
  if(!ina219.init()){
    Serial.println("INA219 not connected!");
  }
  ina219.setADCMode(BIT_MODE_9);
  ina219.setPGain(PG_320);
  ina219.setBusRange(BRNG_16);
  ina219.setShuntSizeInOhms(0.01); // used in INA219.
}

void inaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}

// baseInfoFeedback.
void baseInfoFeedback() {
	static unsigned long last_feedback_time;
	if (millis() - last_feedback_time < feedbackFlowExtraDelay) {
		return;
	}
	
	last_feedback_time = millis();

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_BASE_INFO;
	//jsonInfoHttp["temp"] = temp;
	jsonInfoHttp["mW"] = power_mW;
	jsonInfoHttp["curr_mA"] = current_mA;
	jsonInfoHttp["busv"] = busVoltage_V;
	jsonInfoHttp["shuntv"] = shuntVoltage_mV;
	jsonInfoHttp["v"] = loadVoltage_V;

	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}

