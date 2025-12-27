#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include "config.h"
#include <PID_v2.h>

extern int switch_pwm_A;
extern int switch_pwm_B;
extern bool usePIDCompute;
extern float spd_rate_A;
extern float spd_rate_B ;
extern bool heartbeatStopFlag;

extern int lastEncoderA;
extern int lastEncoderB;

extern double speedGetA;
extern double speedGetB;

extern double plusesRate;

extern PID_v2 pidA;
extern PID_v2 pidB;

extern double outputA;
extern double outputB;
extern double setpointA;
extern double setpointB;

extern int setpoint_interval;
extern float setpointA_buffer;
extern float setpointB_buffer;
extern float setpointA_last;
extern float setpointB_last;
extern float change_offset;
extern bool new_setpoint_flag ;

void motionPinInit();
void switchEmergencyStop();
void switchPortCtrlA(float);
void switchPortCtrlB(float);
void switchCtrl(int, int);
void lightCtrl(int);
void setSpdRate(float, float);
void getSpdRate();

void pidControllerInit();
void leftCtrl(float);
void rightCtrl(float);
void setGoalSpeed(float, float);
void pidControllerCompute();
void LeftPidControllerCompute();
void RightPidControllerCompute();
void setPID(float inputP, float inputI, float inputD, float inputLimits);
void heartBeatCtrl();
void changeHeartBeatDelay(int inputCmd);
void mm_settings(byte inputMain, byte inputModule);


#endif // CONFIG_H