# ESP32 Driver V3

The General Driver v3 is optimized for continuous operation. It features:
- Removal of all references to any bot other than Waveshare rover
- A new display library
- Storage of calibration information in NVS. Calibration information includes Hard/Soft iron, Accelerometer, Magnetometer
- A built-in Magdwick filter that can be disabled.
- Beta convergeance
- Stream to Websockets server for remote calibration