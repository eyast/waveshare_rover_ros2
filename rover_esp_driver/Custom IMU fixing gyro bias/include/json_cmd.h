#define FEEDBACK_BASE_INFO  1001
#define FEEDBACK_IMU_DATA   1002

// SPEED_INPUT
// {"T":1,"L":0.5,"R":0.5}
#define CMD_SPEED_CTRL	1

// {"T":11,"L":164,"R":164} (input PWM +-255)
#define CMD_PWM_INPUT	11

#define CMD_SET_MOTOR_PID	2

// OLED INFO SET
// {"T":3,"lineNum":0,"Text":"putYourTextHere"}
#define CMD_OLED_CTRL	3

// OLED DEFAULT
// {"T":-3}
#define CMD_OLED_DEFAULT	-3

// MODULE TYPE
// 0: nothing
// 1: RoArm-M2-S
// 2: Gimbal
// {"T":4,"cmd":0}
#define CMD_MODULE_TYPE	4

// CHANGE HEART BEAT DELAY
// {"T":136,"cmd":3000}
#define CMD_HEART_BEAT_SET	136

// SET SPEED RATE
// {"T":138,"L":1,"R":1}
#define CMD_SET_SPD_RATE	138

// GET SPEED RATE
// {"T":139}
#define CMD_GET_SPD_RATE	139

// SAVE SPEED RATE
// {"T":140}
#define CMD_SAVE_SPD_RATE	140

// reboot device.
// {"T":600}
#define CMD_REBOOT 	600

// === === === mainType & moduleType settings. === === ===
// {"T":900,"main":1,"module":0}
// main_type: 1-WAVE ROVER, 2-UGV02, 3-UGV01
#define CMD_MM_TYPE_SET 900

// Toggle IMU data streaming
// off: {"T":325,"cmd":0} [default]
//  on: {"T":325,"cmd":1}
#define CMD_IMU_STREAM_CTRL    325

// IMU stream data response type
// {"T":326,"ax":...,"ay":...,...}
#define FEEDBACK_IMU_STREAM    326

// duplicate of 325, I believe
#define CMD_STREAM_FORMAT      400

// WIFI AP settings (note: lower case 'ssid' and 'pass')
// {"T": 500, "ssid": "abc", "pass": "password"}
#define CMD_WIFI                500

// WebSockets IP. Port is hardcoded at 8080
// {"T": 501, "server": "10.0.0.1"}
#define CMD_SERVER              501

// GET STATUS OF WEBSOCKETS
// {"T": 502}
#define WS_STATUS               502

// START STREAMING WEBSOCKETS
// {"T": 503}
#define WS_START                503

// STOP STREAMING WEBSOCKETS
// {"T": 504}
#define WS_STOP                 504
