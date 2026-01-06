1. Removed WebSocket
2. Removed Calibration commands (commands start with CAL)
3. Removed Beta convergeance (commands start with BETA, FAST)
4. Changed Axis orientation of MAG back to +Z
5. Changed frequency of Power monitoring to 2 Hz
6. Reduced Splash screen to 250 ms
7. Increased max battery setting to 12.65


sensors.cpp
- Removed references to Ctrl 8 (it's unused)
- Moved Gyro calibration to the beginning (post who ami) and added system confirmation
- Added a call to Status 1 after gyro calib
- Change Ctrl 1 initialization code to 0x60 instead of (0x60 | 0x18)
- Change filter pass settings from 0x11 (0b0001 0001) to 0x0111 0111 (0x77)


TODO:
[x] Add Gyroscope and Accelerometer Self-Test
[ ] Check the time for Gyro sensitivity calibration (sensors.cpp line 45) is enough (set at 3200 ms)
[ ] Check INA configuration
[ ] Inspect the board (microscope) to check that Z axis of both AK and QMI are the same. Reflect this on the hardcoded_calibration.h

reordered bias terms
added big endian
fixed accelerator code
fixed soft iron
fixed call to is_overflow (out of index)
