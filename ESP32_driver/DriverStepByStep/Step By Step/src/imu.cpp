#include "imu.h"
#include "i2c_helpers.h"
#include "config.h"
#include "syshelpers.h"


// =============================================================================
// QMI8658C REGISTER ADDRESSES
// =============================================================================
// Register map on page 26 of the datasheet (Revision 0x7C)
#define POSTBOOT_TIME   1750  //timeout period after booting (ms)

// Control registers
#define QMI_WHO_AM_I    0x00  // Device ID register (should read 0x05)
#define QMI_REVISION_ID 0x01  // Revision ID (should read 0x79)
#define QMI_CTRL1       0x02  // SPI interface and Sensor Enable
#define QMI_CTRL2       0x03  // Accelerometer config
#define QMI_CTRL3       0x04  // Gyroscope config  
#define QMI_CTRL5       0x06  // Low pass filter setting
#define QMI_CTRL6       0x07  // AttitudeEngine Settings
#define QMI_CTRL7       0x08  // Enable/disable sensors
#define QMI_CTRL9       0x0A  // Calibration commands
// Each data point below is 16 bits long and stored
// in 2x8 bits registers, hence why there's a L(Low)
// and (H)High for each datapoint
// Calibration Registers
#define QMI_CAL1_L      0x0B  
#define QMI_CAL1_H      0x0C
#define QMI_CAL2_L      0x0D
#define QMI_CAL2_H      0x0E
#define QMI_CAL3_L      0x0F
#define QMI_CAL3_H      0x10
#define QMI_CAL4_L      0x11
#define QMI_CAL4_H      0x12
// FIFO registers
#define QMI_FIFO_WTM_TH 0x13  // FIFO watermark level, in ODRs
#define QMI_FIFO_CTRL   0x14  // FIFO Setup
#define QMI_SMPL_CNT    0x15  // FIFO sample count LSBs
#define QMI_FIFO_STATUS 0x16  // FIFO Status
#define QMI_FIFO_DATA   0x17  // FIFO Data
// Status
#define QMI_STATUSINT   0x2D  // Sensor Data Availability with the Locking Mechanism
#define QMI_STATUS0     0x2E  // Data ready status
#define QMI_STATUS1     0x2F  // Misc Status (Wake on Motion, CmdDone, Ctrl9 protocol bit)
// Timestamp
#define QMI_TS_L        0x30  // Lower 8 bits
#define QMI_TS_M        0x31  // Middle 8 bits
#define QMI_TS_H        0x32  // Upper 8 bits
// Data Output: Temperature
#define QMI_TEMP_L      0x33  // Temperature (low byte)
#define QMI_TEMP_H      0x34  // Temperature (upper 8 bits)
// Data Output: Accelerometer
#define QMI_AX_L        0x35  // Accelerometer X (low byte) - data starts here
#define QMI_AX_H        0x36  // Accel, X, upper
#define QMI_AY_L        0x37  //
#define QMI_AY_H        0x38
#define QMI_AZ_L        0x39
#define QMI_AZ_H        0x3A
// Data Output: Gyroscope
#define QMI_GX_L        0x3B  // Gyroscope, X, Lower
#define QMI_GX_H        0x3C
#define QMI_GY_L        0x3D
#define QMI_GY_H        0x3E
#define QMI_GZ_L        0x3F
#define QMI_GZ_H        0x40
// Data Output: Quaternion Increment
#define QMI_DQW_L       0x49
#define QMI_DQW_H       0x4A
#define QMI_DQX_L       0x4B
#define QMI_DQX_H       0x4C
#define QMI_DQY_L       0x4D
#define QMI_DQY_H       0x4E
#define QMI_DQZ_L       0x4F
#define QMI_DQZ_H       0x50
// Data Output: Velocity Increment
#define QMI_DVX_L       0x51
#define QMI_DVX_H       0x52
#define QMI_DVY_L       0x53
#define QMI_DVY_H       0x54
#define QMI_DVZ_L       0x55
#define QMI_DVZ_H       0x56
// Attitude Engine
#define QMI_AE_REG1     0x57
#define AMI_AE_REG2     0x58
// System commands
#define QMI_RESET       0x60  // Reset command register

// Expected WHO_AM_I value - confirms we're talking to right chip
#define QMI_WHO_AM_I_VALUE  0x05
#define QMI_REVISION_ID_VAL 0x7C

IMU imu;

IMU::IMU(){
    sysecho("[QMI]:Constructing IMU");
    memset(&data, 0, sizeof(data));
    imu_ok = false;
    mag_ok = false;
}

bool IMU::begin(){
    sysecho("[QMI]:Initializing Accel/Gyro");
    bool ae_enabled = true;
    imu_ok = init_qmi8658c(ae_enabled);
    return true;
}

bool IMU::qmi_soft_reset(){
    // Soft resets the QMI chip
    bool softreset = false;
    sysecho("[QMI]:Soft resetting, please wait 1.75s");
    softreset = i2c_write_byte(QMI8658C_ADDR, QMI_RESET, 0xB0);
    delay(POSTBOOT_TIME);
    if (!softreset){
        sysecho("[QMI]:Failed to soft reset");
        return false;
    } else sysecho("[QMI]:Soft Reset successful");
    return true;
}

bool IMU::qmi_who_am_i() const{
    // Typical usage defined on page 29 / datasheet
    // Step 1: Check the device is there
    // Initialize the Gyroscope / Accelerometer
    // By reading the "WHO_AM_ register" which should return
    // a specific value
    uint8_t who_am_i = i2c_read_byte(QMI8658C_ADDR, QMI_WHO_AM_I);
    sysecho("[QMI]:who am i:", who_am_i);
    uint8_t revision_id = i2c_read_byte(QMI8658C_ADDR, QMI_REVISION_ID);
    sysecho("[QMI]:revision id:", revision_id);
    if (who_am_i != QMI_WHO_AM_I_VALUE){
        sysecho("[QMI]:Wrong QMI Who Am I value returned");
        return false;
    }
    return true;
}

bool IMU::init_qmi8658c(bool ae_enabled){
    bool qmi_who_ami_result = qmi_who_am_i();
    bool reset_status = qmi_soft_reset();
    
    // STEP 3: Configure CTRL1
    // WHAT: Sets up how data is formatted, and disable the sensors
    uint8_t ctrl1_val = 0x00;
    ctrl1_val |= (0 << 7);  // SIM: 4-wire SPI (doesn't matter for I2C)
    ctrl1_val |= (1 << 6);  // ADDR_AI: Enable auto-increment
    ctrl1_val |= (1 << 5);  // BE: Big endian
    ctrl1_val |= (0 << 0);  // SensorDisable true (Sensors are enabled)
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL1, ctrl1_val);
    //i2c_write_byte(QMI8658C_ADDR, QMI_CTRL1, 0xB0);
    delay(20);

    //
    //i2c_write_byte(QMI8658C_ADDR, QMI_CTRL1, 0x60 | 0x18);
    
    // STEP 4: Disable sensors while configuring (safe practice)
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL7, 0x00);
    
    // STEP 5: Configure accelerometer
    // WHAT: Sets measurement range and update rate
    // Format: (range << 4) | output_data_rate
    // WHY shift by 4: Upper 4 bits = range, lower 4 bits = ODR
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL2, (ACCEL_RANGE << 4) | SENSOR_ODR);
    
    // STEP 6: Configure gyroscope (same format)
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL3, (GYRO_RANGE << 4) | SENSOR_ODR);
    
    // STEP 7: Enable low-pass filters
    // WHY: Reduces noise, makes data smoother
    // 0x11 = enable filters for both accel and gyro
    // 0b 0 0 0 0 0 0 0 0
    // 0b 0 1 1 1 0 1 1 1
    // 0b01110111
    // 0b00010001)
    //i2c_write_byte(QMI8658C_ADDR, QMI_CTRL5, 0x11);
    i2c_write_byte(QMI8658C_ADDR, QMI_CTRL5, 0b00010001);
    
    // STEP 8: Enable both sensors
    if (ae_enabled){
        sysecho("[QMI]:AttitudeEngine Enabled");
        // 0b 0 0 0 0 1 0 1 1 -> 0x11;
        i2c_write_byte(QMI8658C_ADDR, QMI_CTRL7, 0x11);
        //i2c_write_byte(QMI8658C_ADDR, QMI_CTRL7, 0x75); //High speed clock
    } else {
        i2c_write_byte(QMI8658C_ADDR, QMI_CTRL7, 0x03);
    }
    
    // STEP 9: Calculate conversion scales
    // WHY: Raw data is in arbitrary units - we need real world units
    
    // Accelerometer: depends on range setting
    // Datasheet - page 18
    const float accel_ranges[] = {2.0f, 4.0f, 8.0f, 16.0f};  // in g
    // WHY divide by 32768: 16-bit signed = -32768 to +32767
    // So ±range maps to ±32768
    accel_scale = accel_ranges[ACCEL_RANGE] / 32768.0f;
    
    // Gyroscope: depends on range setting
    // Datasheet - page 19
    const float gyro_ranges[] = {16.0f, 32.0f, 64.0f, 128.0f, 
                                 256.0f, 512.0f, 1024.0f, 2048.0f};  // in °/s
    gyro_scale = gyro_ranges[GYRO_RANGE] / 32768.0f;
    Serial.println(accel_scale, 7);
    Serial.println(gyro_scale, 7);
    sysecho("[QMI]:Initialized successfully");
    
    // STEP 10: Let sensor stabilize
    delay(100);
    
    return true;
}

bool IMU::update(){
    bool imu_success = false;
    if (imu_ok){
        imu_success = read_qmi8658c();
    }
    return imu_success;
}

bool IMU::read_qmi8658c() {
    // There are 28 addresses to read data from 
    // The first 14 start from TEMP_L (0x33) and include
    // Temperature(2), Accelerometer(6), Gyroscope(6)
    // The 2nd 14 start from dQw_l (0x49) and include
    // Quaternion updates (8), Velocity Updates (6)
    uint8_t status = i2c_read_byte(QMI8658C_ADDR, QMI_STATUS0);
    if (!(status & 0x03)) {
        return false;  // Data not ready yet
    }
    uint8_t buffer1[14], buffer2[14];
    if (!i2c_read_bytes(QMI8658C_ADDR,
                        QMI_TEMP_L,
                        buffer1,
                        14)
    ){
        return false;
    };
    if (!i2c_read_bytes(QMI8658C_ADDR,
                        QMI_DQW_L,
                        buffer2,
                        14)
    ){
        return false;
    };
    // Temperature 
    int16_t temp_raw = (buffer1[1] << 8) | buffer1[0];
    data.temperature = temp_raw / 256.0f;
    // Accel and Gyro
    data.accel_raw[0] = (buffer1[3] << 8) | buffer1[2];
    data.accel_raw[1] = (buffer1[5] << 8) | buffer1[4];
    data.accel_raw[2] = (buffer1[7] << 8) | buffer1[6];
    data.gyro_raw[0] = (buffer1[9] << 8) | buffer1[8];
    data.gyro_raw[1] = (buffer1[11] << 8) | buffer1[10];
    data.gyro_raw[2] = (buffer1[13] << 8) | buffer1[12];
    // Quaternion and Velocity
    data.dq[0] = (buffer2[1] << 8) | buffer2[0];
    data.dq[1] = (buffer2[3] << 8) | buffer2[2];
    data.dq[2] = (buffer2[5] << 8) | buffer2[4];
    data.dq[3] = (buffer2[7] << 8) | buffer2[6];
    data.dv[0] = (buffer2[9] << 8) | buffer2[8];
    data.dv[1] = (buffer2[11] << 8) | buffer2[10];
    data.dv[2] = (buffer2[13] << 8) | buffer2[12];
    //Convert to physical units
    for (int i=0; i < 3; i++){
        data.accel[i] = data.accel_raw[i] * accel_scale;
        data.gyro[i] = data.gyro_raw[i] * gyro_scale;
    }
    print_imu_data(data);
    return true;
}

void print_imu_data(IMU_DATA data){
        // Print Accel and Gyro
    Serial.printf("PHYS - T:%.1f°C A:[%.4f,%.4f,%.4f]g G:[%.2f,%.2f,%.2f]°/s\n",
              data.temperature,
              data.accel[0], data.accel[1], data.accel[2],
              data.gyro[0], data.gyro[1], data.gyro[2]);
    // Print Quaternion and Velocity 
    Serial.printf("AE - Q:[%d,%d,%d,%d] dv:[%d,%d,%d]m/s\n",
              data.dq[0], data.dq[1], data.dq[2], data.dq[3],
              data.dv[0], data.dv[1], data.dv[2]);    
}