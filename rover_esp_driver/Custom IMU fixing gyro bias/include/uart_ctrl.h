// =============================================================================
// UART Control - Serial Command Processing
// =============================================================================

#ifndef UART_CTRL_H
#define UART_CTRL_H

// -----------------------------------------------------------------------------
// State Variables
// -----------------------------------------------------------------------------

bool imu_stream_enabled = true;
bool stream_as_json = false;

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
        // {"T":325,"cmd":1} to enable, {"T":325,"cmd":0} to disable
        if (jsonCmdReceive.containsKey("cmd")) {
            imu_stream_enabled = (jsonCmdReceive["cmd"].as<int>() == 1);
            jsonInfoSend.clear();
            jsonInfoSend["T"] = CMD_IMU_STREAM_CTRL;
            jsonInfoSend["enabled"] = imu_stream_enabled;
            serializeJson(jsonInfoSend, Serial);
            Serial.println();
        }
        break;

    case CMD_STREAM_FORMAT:
        // {"T":400,"cmd":1} for JSON format, {"T":400,"cmd":0} for MotionCal format
        if (jsonCmdReceive.containsKey("cmd")) {
            stream_as_json = (jsonCmdReceive["cmd"].as<int>() == 1);
            jsonInfoSend.clear();
            jsonInfoSend["T"] = CMD_STREAM_FORMAT;
            jsonInfoSend["json_format"] = stream_as_json;
            serializeJson(jsonInfoSend, Serial);
            Serial.println();
        }
        break;

    // --- IMU Calibration (330-332) ---

    case CMD_IMU_CALIBRATE_GYRO:
        // {"T":330}
        Serial.println("{\"status\":\"calibrating_gyro\"}");
        imu.calibrate_gyro(500);
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_CALIBRATE_GYRO;
        jsonInfoSend["status"] = "complete";
        jsonInfoSend["bias_x"] = imu.get_data().gyro_bias[0];
        jsonInfoSend["bias_y"] = imu.get_data().gyro_bias[1];
        jsonInfoSend["bias_z"] = imu.get_data().gyro_bias[2];
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;

    case CMD_IMU_CALIBRATE_ACCEL:
        // {"T":331}
        Serial.println("{\"status\":\"calibrating_accel\"}");
        imu.calibrate_accel(500);
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_CALIBRATE_ACCEL;
        jsonInfoSend["status"] = "complete";
        jsonInfoSend["bias_x"] = imu.get_data().accel_bias[0];
        jsonInfoSend["bias_y"] = imu.get_data().accel_bias[1];
        jsonInfoSend["bias_z"] = imu.get_data().accel_bias[2];
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;

    case CMD_IMU_CALIBRATE_ALL:
        // {"T":332}
        Serial.println("{\"status\":\"calibrating_all\"}");
        imu.calibrate_gyro(500);
        imu.calibrate_accel(500);
        filter.reset();
        // After calibrating, initialize from sensors for instant alignment
        if (mag_ok) {
            imu.read();
            mag.read();
            const QMI8658C_Data& imu_data = imu.get_data();
            const AK09918C_Data& mag_data = mag.get_data();
            filter.initialize_from_sensors(
                imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]
            );
        }
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_CALIBRATE_ALL;
        jsonInfoSend["status"] = "complete";
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;

    // --- IMU Filter Control (335-336) ---

    case CMD_IMU_FILTER_RESET:
        // {"T":335}
        filter.reset();
        // After reset, optionally start fast convergence
        filter.start_fast_convergence(FAST_CONVERGENCE_DURATION_MS, FAST_CONVERGENCE_BETA);
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_FILTER_RESET;
        jsonInfoSend["status"] = "reset_with_fast_convergence";
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;

    case CMD_IMU_SET_BETA:
        // {"T":336,"beta":0.1}
        if (jsonCmdReceive.containsKey("beta")) {
            float new_beta = jsonCmdReceive["beta"].as<float>();
            if (new_beta > 0 && new_beta < 10.0f) {
                filter.set_beta(new_beta);
                jsonInfoSend.clear();
                jsonInfoSend["T"] = CMD_IMU_SET_BETA;
                jsonInfoSend["beta"] = filter.get_base_beta();
                serializeJson(jsonInfoSend, Serial);
                Serial.println();
            }
        }
        break;

    // --- Magnetometer Axis Correction (340) ---

    case CMD_IMU_SET_MAG_SIGNS:
        // {"T":340,"x":1,"y":1,"z":-1}
        if (jsonCmdReceive.containsKey("x") && 
            jsonCmdReceive.containsKey("y") && 
            jsonCmdReceive.containsKey("z")) {
            int8_t sx = jsonCmdReceive["x"].as<int>();
            int8_t sy = jsonCmdReceive["y"].as<int>();
            int8_t sz = jsonCmdReceive["z"].as<int>();
            mag.set_axis_signs(sx, sy, sz);
            jsonInfoSend.clear();
            jsonInfoSend["T"] = CMD_IMU_SET_MAG_SIGNS;
            jsonInfoSend["x"] = (sx >= 0) ? 1 : -1;
            jsonInfoSend["y"] = (sy >= 0) ? 1 : -1;
            jsonInfoSend["z"] = (sz >= 0) ? 1 : -1;
            serializeJson(jsonInfoSend, Serial);
            Serial.println();
        }
        break;

    // --- IMU Debug/Status (345-346) ---

    case CMD_IMU_DEBUG: {
        // {"T":345} - prints human-readable debug info
        const QMI8658C_Data& imu_data = imu.get_data();
        const AK09918C_Data& mag_data = mag.get_data();
        
        Serial.println("\n=== IMU Debug ===");
        Serial.print("Accel (g): ");
        Serial.print(imu_data.accel[0], 4); Serial.print(", ");
        Serial.print(imu_data.accel[1], 4); Serial.print(", ");
        Serial.println(imu_data.accel[2], 4);
        
        Serial.print("Gyro (dps): ");
        Serial.print(imu_data.gyro_dps[0], 4); Serial.print(", ");
        Serial.print(imu_data.gyro_dps[1], 4); Serial.print(", ");
        Serial.println(imu_data.gyro_dps[2], 4);
        
        Serial.print("Mag (uT): ");
        Serial.print(mag_data.mag[0], 2); Serial.print(", ");
        Serial.print(mag_data.mag[1], 2); Serial.print(", ");
        Serial.println(mag_data.mag[2], 2);
        
        float mag_h = sqrtf(mag_data.mag[0]*mag_data.mag[0] + mag_data.mag[1]*mag_data.mag[1]);
        float mag_t = sqrtf(mag_h*mag_h + mag_data.mag[2]*mag_data.mag[2]);
        float dip = atan2f(mag_data.mag[2], mag_h) * 180.0f / PI;
        
        Serial.print("Mag: horiz="); Serial.print(mag_h, 1);
        Serial.print(" total="); Serial.print(mag_t, 1);
        Serial.print(" dip="); Serial.print(dip, 1); Serial.println("°");
        Serial.println("(Melbourne should have dip ≈ -65°)");
        
        Serial.print("Orientation: Y="); Serial.print(filter.get_yaw(), 1);
        Serial.print(" P="); Serial.print(filter.get_pitch(), 1);
        Serial.print(" R="); Serial.println(filter.get_roll(), 1);
        Serial.print("Base Beta="); Serial.print(filter.get_base_beta(), 3);
        Serial.print(" Active Beta="); Serial.println(filter.get_active_beta(), 3);
        Serial.print("Convergence Rate="); Serial.print(filter.get_convergence_rate(), 4);
        Serial.print(" Converged="); Serial.println(filter.is_converged() ? "YES" : "NO");
        Serial.print("Adaptive Beta="); Serial.print(filter.get_adaptive_beta() ? "ON" : "OFF");
        Serial.print(" Fast Mode="); Serial.println(filter.in_fast_convergence() ? "ACTIVE" : "OFF");
        break;
    }

    case CMD_IMU_GET_STATUS: {
        // {"T":346} -> returns full status as JSON
        const QMI8658C_Data& imu_data = imu.get_data();
        const AK09918C_Data& mag_data = mag.get_data();
        
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_GET_STATUS;
        
        JsonArray accel = jsonInfoSend.createNestedArray("accel");
        accel.add(imu_data.accel[0]);
        accel.add(imu_data.accel[1]);
        accel.add(imu_data.accel[2]);
        
        JsonArray gyro = jsonInfoSend.createNestedArray("gyro_dps");
        gyro.add(imu_data.gyro_dps[0]);
        gyro.add(imu_data.gyro_dps[1]);
        gyro.add(imu_data.gyro_dps[2]);
        
        JsonArray mag_arr = jsonInfoSend.createNestedArray("mag");
        mag_arr.add(mag_data.mag[0]);
        mag_arr.add(mag_data.mag[1]);
        mag_arr.add(mag_data.mag[2]);
        
        float mag_h = sqrtf(mag_data.mag[0]*mag_data.mag[0] + mag_data.mag[1]*mag_data.mag[1]);
        float dip = atan2f(mag_data.mag[2], mag_h) * 180.0f / PI;
        jsonInfoSend["mag_dip"] = dip;
        jsonInfoSend["beta"] = filter.get_base_beta();
        jsonInfoSend["active_beta"] = filter.get_active_beta();
        
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;
    }

    // --- Orientation Query (350) ---

    case CMD_IMU_GET_ORIENTATION:
        // {"T":350} -> returns {"T":350,"yaw":x,"pitch":y,"roll":z}
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_GET_ORIENTATION;
        jsonInfoSend["yaw"] = filter.get_yaw();
        jsonInfoSend["pitch"] = filter.get_pitch();
        jsonInfoSend["roll"] = filter.get_roll();
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;

    // =========================================================================
    // NEW: Fast Initialization & Adaptive Beta Commands (351-354)
    // =========================================================================

    case CMD_IMU_INIT_FROM_SENSORS: {
        // {"T":351} - Initialize filter from current sensor readings
        // This gives instant correct orientation!
        if (!imu_ok) {
            jsonInfoSend.clear();
            jsonInfoSend["T"] = CMD_IMU_INIT_FROM_SENSORS;
            jsonInfoSend["status"] = "error";
            jsonInfoSend["message"] = "IMU not available";
            serializeJson(jsonInfoSend, Serial);
            Serial.println();
            break;
        }
        
        // Get fresh readings
        imu.read();
        const QMI8658C_Data& imu_data = imu.get_data();
        
        if (mag_ok) {
            mag.read();
            const AK09918C_Data& mag_data = mag.get_data();
            filter.initialize_from_sensors(
                imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                mag_data.mag[0], mag_data.mag[1], mag_data.mag[2]
            );
        } else {
            // Fall back to accel-only (yaw will be 0)
            filter.initialize_from_accel(
                imu_data.accel[0], imu_data.accel[1], imu_data.accel[2]
            );
        }
        
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_INIT_FROM_SENSORS;
        jsonInfoSend["status"] = "initialized";
        jsonInfoSend["yaw"] = filter.get_yaw();
        jsonInfoSend["pitch"] = filter.get_pitch();
        jsonInfoSend["roll"] = filter.get_roll();
        jsonInfoSend["with_mag"] = mag_ok;
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;
    }

    case CMD_IMU_FAST_CONVERGENCE: {
        // {"T":352,"duration":2000,"beta":2.5}
        uint32_t duration = FAST_CONVERGENCE_DURATION_MS;
        float fast_beta = FAST_CONVERGENCE_BETA;
        
        if (jsonCmdReceive.containsKey("duration")) {
            duration = jsonCmdReceive["duration"].as<uint32_t>();
        }
        if (jsonCmdReceive.containsKey("beta")) {
            fast_beta = jsonCmdReceive["beta"].as<float>();
        }
        
        filter.start_fast_convergence(duration, fast_beta);
        
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_FAST_CONVERGENCE;
        jsonInfoSend["status"] = "started";
        jsonInfoSend["duration"] = duration;
        jsonInfoSend["beta"] = fast_beta;
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;
    }

    case CMD_IMU_ADAPTIVE_BETA: {
        // {"T":353,"enabled":1,"stationary":0.5,"motion":0.05,"threshold":0.05}
        if (jsonCmdReceive.containsKey("enabled")) {
            bool enabled = (jsonCmdReceive["enabled"].as<int>() == 1);
            filter.set_adaptive_beta(enabled);
        }
        
        if (jsonCmdReceive.containsKey("stationary") && jsonCmdReceive.containsKey("motion")) {
            float stationary = jsonCmdReceive["stationary"].as<float>();
            float motion = jsonCmdReceive["motion"].as<float>();
            filter.set_beta_range(stationary, motion);
        }
        
        if (jsonCmdReceive.containsKey("threshold")) {
            float threshold = jsonCmdReceive["threshold"].as<float>();
            filter.set_motion_threshold(threshold);
        }
        
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_ADAPTIVE_BETA;
        jsonInfoSend["enabled"] = filter.get_adaptive_beta();
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
        break;
    }

    case CMD_IMU_CONVERGENCE_STATUS:
        // {"T":354} -> {"T":354,"rate":0.01,"converged":true,"active_beta":0.1}
        jsonInfoSend.clear();
        jsonInfoSend["T"] = CMD_IMU_CONVERGENCE_STATUS;
        jsonInfoSend["rate"] = filter.get_convergence_rate();
        jsonInfoSend["converged"] = filter.is_converged();
        jsonInfoSend["active_beta"] = filter.get_active_beta();
        jsonInfoSend["base_beta"] = filter.get_base_beta();
        jsonInfoSend["fast_mode"] = filter.in_fast_convergence();
        jsonInfoSend["adaptive"] = filter.get_adaptive_beta();
        serializeJson(jsonInfoSend, Serial);
        Serial.println();
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
