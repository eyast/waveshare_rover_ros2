/*
 * Sensors Module Implementation
 */

#include "sensors.h"
#include "i2c_helpers.h"
#include "protocol.h"
#include "hardcoded_calibration.h"

// =============================================================================
// Global Sensor Instances
// =============================================================================

QMI8658C imu(ADDR_QMI8658C);
AK09918C mag(ADDR_AK09918C);
INA219 pwr(ADDR_INA219);

// =============================================================================
// QMI8658C Implementation
// =============================================================================

QMI8658C::QMI8658C(uint8_t addr) : addr_(addr), accel_scale_(0), gyro_scale_(0), ok_(false) {
    memset(&data_, 0, sizeof(data_));
    
    // Initialize Acceleration Matrix to identity
    data_.accel_calib_matrix[0][0] = 1.0f;
    data_.accel_calib_matrix[1][1] = 1.0f;
    data_.accel_calib_matrix[2][2] = 1.0f;
}

bool QMI8658C::begin(uint8_t accel_fs, uint8_t gyro_fs, uint8_t odr) {
    // Check device ID
    uint8_t whoami = i2c_read_register(addr_, QMI_WHO_AM_I);
    if (whoami != QMI_WHO_AM_I_VALUE) {
        out_error("QMI8658C", "WHO_AM_I failed");
        ok_ = false;
        return false;
    }


    // On-demand calibration (hardware gyro bias)
    out_system("QMI8658C", "calibrating");
    i2c_write_register(addr_, QMI_RESET, 0xB0);
    delay(10);

    // Disable sensors before config
    i2c_write_register(addr_, QMI_CTRL7, 0x00);

    // Calibrate Gyroscope sensitivity
    i2c_write_register(addr_, QMI_CTRL9, QMI_CTRL9_CMD_CALI);
    vTaskDelay(pdMS_TO_TICKS(1000));
    //delay(3200);
    i2c_write_register(addr_, QMI_CTRL9, QMI_CMD_NOP);
    delay(100);
    i2c_write_register(addr_, QMI_STATUS1, QMI_CMD_NOP);
    delay(100);

    
    // CTRL1: Address auto-increment, Big-endian
    // i2c_write_register(addr_, QMI_CTRL1, 0x60 | 0x18);
    //i2c_write_register(addr_, QMI_CTRL1, 0x60 | 0x10);
    i2c_write_register(addr_, QMI_CTRL1, 0x60);
    
    // Accelerometer config
    i2c_write_register(addr_, QMI_CTRL2, (accel_fs << 4) | odr);
    
    // Gyroscope config
    i2c_write_register(addr_, QMI_CTRL3, (gyro_fs << 4) | odr);
    
    // Enable low-pass filters
    //i2c_write_register(addr_, QMI_CTRL5, 0x11);
    i2c_write_register(addr_, QMI_CTRL5, 0x77);
    
    // Enable both sensors
    i2c_write_register(addr_, QMI_CTRL7, 0x03);
    
    // Calculate scale factors
    const float accel_fs_values[] = {2.0f, 4.0f, 8.0f, 16.0f};
    accel_scale_ = accel_fs_values[accel_fs & 0x03] / 32768.0f;
    
    const float gyro_fs_values[] = {16.0f, 32.0f, 64.0f, 128.0f, 256.0f, 512.0f, 1024.0f, 2048.0f};
    gyro_scale_ = gyro_fs_values[gyro_fs & 0x07] / 32768.0f;
    
    delay(100);
    
    // Flush initial readings
    for (int i = 0; i < 20; i++) {
        read();
        delay(5);
    }
    
    ok_ = true;
    out_system("QMI8658C", "OK");
    return true;
}

void QMI8658C::set_accel_calib(float bx,
                               float by,
                               float bz,
                               float acc_calib_matrix[3][3]){
    data_.accel_bias[0] = bx;
    data_.accel_bias[1] = by;
    data_.accel_bias[2] = bz;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            data_.accel_calib_matrix[i][j] = acc_calib_matrix[i][j];
        }
    }
}

void QMI8658C::set_gyro_calib(float bx,
                              float by,
                              float bz){
    data_.gyro_bias[0] = bx;
    data_.gyro_bias[1] = by;
    data_.gyro_bias[2] = bz;
}

bool QMI8658C::read() {
    uint8_t status = i2c_read_register(addr_, QMI_STATUS0);
    if (!(status & QMI_STATUS0_AVAIL)) {
        return false;
    }
    
    uint8_t buffer[14];
    if (!i2c_read_registers(addr_, QMI_TEMP_L, buffer, 14)) {
        return false;
    }
    
    // Parse temperature
    data_.temp_raw = (int16_t)((buffer[1] << 8) | buffer[0]);
    data_.temperature = data_.temp_raw / 256.0f;
    
    // Parse accelerometer (little-endian in registers, but we configured big-endian)
    data_.accel_raw[0] = (int16_t)((buffer[3] << 8) | buffer[2]);
    data_.accel_raw[1] = (int16_t)((buffer[5] << 8) | buffer[4]);
    data_.accel_raw[2] = (int16_t)((buffer[7] << 8) | buffer[6]);
    
    // Parse gyroscope
    data_.gyro_raw[0] = (int16_t)((buffer[9] << 8) | buffer[8]);
    data_.gyro_raw[1] = (int16_t)((buffer[11] << 8) | buffer[10]);
    data_.gyro_raw[2] = (int16_t)((buffer[13] << 8) | buffer[12]);
    
    float accel_unbiased[3], gyro_unbiased[3];
    for (int i = 0; i < 3; i++) {
        data_.accel[i]= (data_.accel_raw[i] * accel_scale_) - data_.accel_bias[i];
        gyro_unbiased[i] = data_.gyro_raw[i] - data_.gyro_bias[i];  // Bias in LSB!
        data_.gyro_dps[i] = gyro_unbiased[i] * gyro_scale_;
        data_.gyro[i] = data_.gyro_dps[i] * (PI / 180.0f);
    }
    // for (int i = 0; i < 3; i++) {
    //     data_.accel[i] = (data_.accel_raw[i] * accel_scale_) - data_.accel_bias[i];
    //     //data_.gyro_dps[i] = (data_.gyro_raw[i] * gyro_scale_) - data_.gyro_bias[i];
    //     data_.gyro_dps[i] = ((data_.gyro_raw[i] - data_.gyro_bias[i]  ) * gyro_scale_);
    //     data_.gyro[i] = data_.gyro_dps[i] * (PI / 180.0f);
    // }
    
    float ax_cal = data_.accel_calib_matrix[0][0] * data_.accel[0] +
                data_.accel_calib_matrix[0][1] * data_.accel[1] +
                data_.accel_calib_matrix[0][2] * data_.accel[2];

    float ay_cal = data_.accel_calib_matrix[1][0] * data_.accel[0] +
                data_.accel_calib_matrix[1][1] * data_.accel[1] +
                data_.accel_calib_matrix[1][2] * data_.accel[2];

    float az_cal = data_.accel_calib_matrix[2][0] * data_.accel[0] +
                data_.accel_calib_matrix[2][1] * data_.accel[1] +
                data_.accel_calib_matrix[2][2] * data_.accel[2];

    data_.accel[0] = ax_cal;
    data_.accel[1] = ay_cal;
    data_.accel[2] = az_cal;

    return true;
}

// =============================================================================
// AK09918C Implementation
// =============================================================================

AK09918C::AK09918C(uint8_t addr) : addr_(addr), scale_(MAG_SCALE), ok_(false) {
    memset(&data_, 0, sizeof(data_));
    
    // Initialize soft iron matrix to identity
    data_.soft_iron[0][0] = 1.0f;
    data_.soft_iron[1][1] = 1.0f;
    data_.soft_iron[2][2] = 1.0f;
    overflow_ = false;
    
}

bool AK09918C::begin(uint8_t mode) {
    uint8_t wia1 = i2c_read_register(addr_, AK_WIA1);
    uint8_t wia2 = i2c_read_register(addr_, AK_WIA2);
    
    if (wia1 != AK_WIA1_VALUE || wia2 != AK_WIA2_VALUE) {
        out_error("AK09918C", "ID check failed");
        ok_ = false;
        return false;
    }
    
    // Soft reset
    i2c_write_register(addr_, AK_CNTL3, 0x01);
    delay(100);
    
    // Set operation mode
    i2c_write_register(addr_, AK_CNTL2, mode);
    delay(10);
    
    ok_ = true;
    out_system("AK09918C", "OK");
    return true;
}

void AK09918C::is_overflow(uint8_t hofl_register ){
    bool holf_status = (hofl_register >> 3 ) & 1;
    if (holf_status == 1){
        this->overflow_ = true;
    }
}

bool AK09918C::data_ready() {
    uint8_t st1 = i2c_read_register(addr_, AK_ST1);
    return (st1 & AK_ST1_DRDY) != 0;
}

bool AK09918C::read() {
    if (!data_ready()) {
        return false;
    }
    
    uint8_t buffer[8];
    if (!i2c_read_registers(addr_, AK_HXL, buffer, 8)) {
        return false;
    }
    
    is_overflow(buffer[7]);

    // Parse raw values (little-endian)
    data_.mag_raw[0] = (int16_t)((buffer[1] << 8) | buffer[0]);
    data_.mag_raw[1] = (int16_t)((buffer[3] << 8) | buffer[2]);
    data_.mag_raw[2] = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Apply hard iron correction
    float mx = (data_.mag_raw[0] * scale_) - data_.hard_iron[0];
    float my = (data_.mag_raw[1] * scale_) - data_.hard_iron[1];
    float mz = (data_.mag_raw[2] * scale_) - data_.hard_iron[2];
    
    // Apply soft iron correction
    float mx_si = data_.soft_iron[0][0] * mx + data_.soft_iron[0][1] * my + data_.soft_iron[0][2] * mz;
    float my_si = data_.soft_iron[1][0] * mx + data_.soft_iron[1][1] * my + data_.soft_iron[1][2] * mz;
    float mz_si = data_.soft_iron[2][0] * mx + data_.soft_iron[2][1] * my + data_.soft_iron[2][2] * mz;
    
    data_.mag[0] = mx_si;
    data_.mag[1] = my_si;
    data_.mag[2] = mz_si;

    // data_.magnitude = sqrtf(mx_si*mx_si + my_si*my_si + mz_si*mz_si);
    // data_.magnitude_error = data_.magnitude - MAG_FIELD_STRENGTH;

    // // Warn if magnitude is significantly off
    // if (fabsf(data_.magnitude_error) > MAG_MAGNITUDE_WARNING_THRESHOLD) {
    //     static uint32_t last_warning = 0;
    //     uint32_t now = millis();
    //     // Rate-limit warnings to once per 5 seconds
    //     if (now - last_warning > 5000) {
    //         out_error("MAG", "field magnitude deviation");
    //         last_warning = now;
    //     }
    // }

    return true;
}

void AK09918C::set_hard_iron(float bx, float by, float bz) {
    data_.hard_iron[0] = bx;
    data_.hard_iron[1] = by;
    data_.hard_iron[2] = bz;
}

void AK09918C::set_soft_iron(const float matrix[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            data_.soft_iron[i][j] = matrix[i][j];
        }
    }
}

// =============================================================================
// INA219 Implementation
// =============================================================================

// INA219 Registers
#define INA219_REG_CONFIG       0x00
#define INA219_REG_SHUNTVOLTAGE 0x01
#define INA219_REG_BUSVOLTAGE   0x02
#define INA219_REG_POWER        0x03
#define INA219_REG_CURRENT      0x04
#define INA219_REG_CALIBRATION  0x05

// Config bits
#define INA219_CONFIG_DEFAULT   0x399F  // 32V, 320mV, 12-bit, continuous

INA219::INA219(uint8_t addr) : addr_(addr), ok_(false) {
    memset(&data_, 0, sizeof(data_));
}

bool INA219::write_register(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    return (Wire.endTransmission() == 0);
}

int16_t INA219::read_register(uint8_t reg) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return 0;
    }
    
    Wire.requestFrom(addr_, (uint8_t)2);
    if (Wire.available() < 2) {
        return 0;
    }
    
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return (int16_t)((msb << 8) | lsb);
}

bool INA219::begin() {
    // Reset device
    if (!write_register(INA219_REG_CONFIG, 0x8000)) {
        out_error("INA219", "reset failed");
        ok_ = false;
        return false;
    }
    delay(10);
    
    // Configure: 16V bus, 320mV shunt, 9-bit, continuous
    // For 0.01 ohm shunt resistor
    uint16_t config = 0x019F;  // BRNG=0(16V), PG=3(320mV), BADC=0011(12bit), SADC=0011(12bit), MODE=111(cont)
    if (!write_register(INA219_REG_CONFIG, config)) {
        out_error("INA219", "config failed");
        ok_ = false;
        return false;
    }
    
    // Set calibration for 0.01 ohm shunt
    // Cal = 0.04096 / (Current_LSB * Rshunt)
    // For 1mA LSB and 0.01 ohm: Cal = 0.04096 / (0.001 * 0.01) = 4096
    if (!write_register(INA219_REG_CALIBRATION, 4096)) {
        out_error("INA219", "calibration failed");
        ok_ = false;
        return false;
    }
    
    ok_ = true;
    out_system("INA219", "OK");
    return true;
}

bool INA219::read() {
    if (!ok_) return false;
    
    // Read shunt voltage (LSB = 10µV)
    int16_t shunt_raw = read_register(INA219_REG_SHUNTVOLTAGE);
    data_.shunt_voltage_mV = shunt_raw * 0.01f;
    
    // Read bus voltage (LSB = 4mV, but bits 0-2 are status)
    int16_t bus_raw = read_register(INA219_REG_BUSVOLTAGE);
    data_.overflow = (bus_raw & 0x01) != 0;
    data_.bus_voltage_V = ((bus_raw >> 3) * 4) / 1000.0f;
    
    // Read current (LSB depends on calibration, we set 1mA)
    int16_t current_raw = read_register(INA219_REG_CURRENT);
    data_.current_mA = current_raw;
    
    // Read power (LSB = 20 * Current_LSB = 20mW with our cal)
    int16_t power_raw = read_register(INA219_REG_POWER);
    data_.power_mW = power_raw * 20.0f;
    
    // Calculate load voltage
    data_.load_voltage_V = data_.bus_voltage_V + (data_.shunt_voltage_mV / 1000.0f);
    
    return true;
}

// =============================================================================
// High-Level Sensor Functions
// =============================================================================

void sensors_init() {
    // Initialize I2C
    i2c_init(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ);
    delay(100);
    
    // Initialize IMU
    if (imu.begin()){
        apply_accelerometer_calibration(imu);
        apply_gyroscope_calibration(imu);

    }
    
    // Initialize Magnetometer
    if (mag.begin()) {
        // Apply hardcoded calibration
        apply_hardcoded_calibration(mag);
    }
    
    // Initialize Power Monitor
    pwr.begin();
}

void sensors_read_imu() {
    imu.read();
    mag.read();
}

void sensors_read_power() {
    pwr.read();
}

bool sensors_imu_ok() {
    return imu.is_ok();
}

bool sensors_mag_ok() {
    return mag.is_ok();
}

bool sensors_pwr_ok() {
    return pwr.is_ok();
}
