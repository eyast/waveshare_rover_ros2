#include "config.h"
#include <INA219_WE.h>

#define INA219_ADDRESS 0x42

extern INA219_WE ina219;
extern float shuntVoltage_mV;
extern float loadVoltage_V;
extern float busVoltage_V;
extern float current_mA;
extern float power_mW; 
extern bool ina219_overflow;

void ina219_init();
void inaDataUpdate();
void baseInfoFeedback();