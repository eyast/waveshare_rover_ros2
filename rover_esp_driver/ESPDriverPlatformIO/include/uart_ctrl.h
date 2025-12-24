// =============================================================================
// UART Control - Serial Command Processing
// =============================================================================

#ifndef UART_CTRL_H
#define UART_CTRL_H

// -----------------------------------------------------------------------------
// State Variables
// -----------------------------------------------------------------------------

bool imu_stream_enabled = false;

// -----------------------------------------------------------------------------
// JSON Command Handler
// -----------------------------------------------------------------------------

void jsonCmdReceiveHandler() {
    int cmdType = jsonCmdReceive["T"].as<int>();

    switch (cmdType) {

    // --- Motor Control ---

    case CMD_SPEED_CTRL:
        if (jsonCmdReceive.containsKey("L") && jsonCmdReceive.containsKey("R")) {
            if (jsonCmdReceive["L"].is<float>() && jsonCmdReceive["R"].is<float>()) {
                heartbeatStopFlag = false;
                lastCmdRecvTime = millis();
                setGoalSpeed(jsonCmdReceive["L"], jsonCmdReceive["R"]);
            }
        }
        break;

    case CMD_PWM_INPUT:
        usePIDCompute = false;
        heartbeatStopFlag = false;
        lastCmdRecvTime = millis();
        leftCtrl(jsonCmdReceive["L"]);
        rightCtrl(jsonCmdReceive["R"]);
        break;

    case CMD_SET_MOTOR_PID:
        setPID(
            jsonCmdReceive["P"],
            jsonCmdReceive["I"],
            jsonCmdReceive["D"],
            jsonCmdReceive["L"]
        );
        break;

    // --- OLED Display ---

    case CMD_OLED_CTRL:
        oledCtrl(jsonCmdReceive["lineNum"], jsonCmdReceive["Text"]);
        break;

    case CMD_OLED_DEFAULT:
        setOledDefault();
        break;

    // --- IMU Streaming ---

    case CMD_IMU_STREAM_CTRL:
        if (jsonCmdReceive.containsKey("cmd")) {
            imu_stream_enabled = (jsonCmdReceive["cmd"].as<int>() == 1);
        }
        break;

    // --- System Control ---

    case CMD_REBOOT:
        esp_restart();
        break;

    // --- Platform Configuration ---

    case CMD_MM_TYPE_SET:
        mm_settings(jsonCmdReceive["main"], jsonCmdReceive["module"]);
        break;
    }
}

// -----------------------------------------------------------------------------
// Serial Control Loop
// -----------------------------------------------------------------------------

void serialCtrl() {
    static String receivedData;

    while (Serial.available() > 0) {
        char receivedChar = Serial.read();
        receivedData += receivedChar;

        if (receivedChar == '\n') {
            DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);

            if (err == DeserializationError::Ok) {
                if (InfoPrint == 1 && uartCmdEcho) {
                    Serial.print(receivedData);
                }
                jsonCmdReceiveHandler();
            }

            receivedData = "";
        }
    }
}

#endif // UART_CTRL_H
