void jsonCmdReceiveHandler(){
	int cmdType = jsonCmdReceive["T"].as<int>();
	switch(cmdType){
	case CMD_SPEED_CTRL:		if (jsonCmdReceive.containsKey("T") &&
										jsonCmdReceive.containsKey("L") &&
										jsonCmdReceive.containsKey("R")){
									if (jsonCmdReceive["L"].is<float>() &&
											jsonCmdReceive["R"].is<float>()){
										heartbeatStopFlag = false;
										lastCmdRecvTime = millis();
										setGoalSpeed(
										jsonCmdReceive["L"],
										jsonCmdReceive["R"]);
									}
								} break;
	case CMD_PWM_INPUT:			usePIDCompute = false;
									heartbeatStopFlag = false;
									lastCmdRecvTime = millis();
									leftCtrl(jsonCmdReceive["L"]);
									rightCtrl(jsonCmdReceive["R"]);
									break;

	case CMD_SET_MOTOR_PID:		setPID(
									jsonCmdReceive["P"],
									jsonCmdReceive["I"],
									jsonCmdReceive["D"],
									jsonCmdReceive["L"]);break;

	case CMD_OLED_CTRL:			oledCtrl(jsonCmdReceive["lineNum"],
									jsonCmdReceive["Text"]);break;

	case CMD_OLED_DEFAULT: 		setOledDefault();break;
	// case CMD_GET_IMU_DATA:						getIMUData();break;
	// case CMD_CALI_IMU_STEP:
	// 											imuCalibration();break;
	// case CMD_GET_IMU_OFFSET:
	// 											getIMUOffset();
	// 											break;
	// case CMD_SET_IMU_OFFSET:
	// 											setIMUOffset(
	// 											jsonCmdReceive["x"],
	// 											jsonCmdReceive["y"],
	// 											jsonCmdReceive["z"]);break;
	// case CMD_BASE_FEEDBACK:
	// 											baseInfoFeedback();break;
	// case CMD_BASE_FEEDBACK_FLOW:
	// 											setBaseInfoFeedbackMode(
	// 											jsonCmdReceive["cmd"]);break;
	// case CMD_FEEDBACK_FLOW_INTERVAL:
	// 											setFeedbackFlowInterval(
	// 											jsonCmdReceive["cmd"]);break;
	// case CMD_UART_ECHO_MODE:
	// 											setCmdEcho(
	// 											jsonCmdReceive["cmd"]);break;
	// case CMD_SET_SPD_RATE:
	// 											setSpdRate(
	// 											jsonCmdReceive["L"],
	// 											jsonCmdReceive["R"]);break;
	// case CMD_GET_SPD_RATE:
	// 											getSpdRate();break;
	// case CMD_SAVE_SPD_RATE:
	// 											saveSpdRate();break;

	// esp-32 dev ctrl.
	case CMD_REBOOT: 			esp_restart();break;


	// mainType & moduleType settings.
	case CMD_MM_TYPE_SET: mm_settings(
												jsonCmdReceive["main"],
												jsonCmdReceive["module"]
												);
												break;
	}
}


void serialCtrl() {
  static String receivedData;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;

    // Detect the end of the JSON string based on a specific termination character
    if (receivedChar == '\n') {
      // Now we have received the complete JSON string
      DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
      if (err == DeserializationError::Ok) {
  			if (InfoPrint == 1 && uartCmdEcho) {
  				Serial.print(receivedData);
  			}
        jsonCmdReceiveHandler();
      } else {
        // Handle JSON parsing error here
      }
      // Reset the receivedData for the next JSON string
      receivedData = "";
    }
  }
}