#include "ICM20948.h"
#include <stdio.h>
#include <iostream>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using std::cout;
using std::endl;
using std::runtime_error;


AK09916::AK09916(I2CMaster &i2c) : I2CDevice(I2C_ADDR, i2c) {
    uint8_t value;
    readRegister(WIA2, &value); if (value != CHIP_ID) throw(runtime_error("AK09916 not found"));
    writeRegister(CNTL3, 0x01); vTaskDelay(5/portTICK_RATE_MS);  // reset and delay 100us (see sec 9.3)
    writeRegister(CNTL2, MDR_100HZ); vTaskDelay(10/portTICK_RATE_MS); // set sample rate
}


ICM20948::ICM20948(I2CMaster &i2c) : I2CDevice(I2C_ADDR_ALT, i2c) { 

    uint8_t value;
    setBank(0);
    readRegister(WHO_AM_I, &value); if (value != CHIP_ID) throw(runtime_error("ICM20948 not found"));
    writeRegister(PWR_MGMT_1, 0x80); vTaskDelay(10 / portTICK_RATE_MS); // reset
    writeRegister(PWR_MGMT_1, 0x01); // autoselect best clock source

    // configure gyro
    setGyroRange(FSR_250DPS);
    setGyroLowpass(true, 5);
    setGyroDataRate(GDR_1250HZ);
    
    // configure accelerometer
    setAccelRange(FSR_2G);
    setAccelLowpass(true, 5);
    setAccelDataRate(ADR_1250HZ);

    writeRegister(PWR_MGMT_2, 0x00); // enable accelerometer and gyro


    // configure magnetometer
    // Ref: https://github.com/kriswiner/MPU9250/issues/86

    // Method 1: Pass-through mode - use primary I2C bus to communicate with magnetometer
    configI2CAuxilary(PASSTHROUGH_MODE);
    try {AK09916 mag(i2c);} catch(const runtime_error &e) {std::cerr << e.what() << endl;}

    configI2CAuxilary(MASTER_MODE);

    // Method 2: I2CMaster Mode - use auxilary I2C bus to communicate with magnetometer
    // readMagRegister(AK09916::WIA2, &value); if (value != AK09916::CHIP_ID) throw(runtime_error("AK09916 not found\n"));
    // writeMagRegister(AK09916::CNTL3, 0x01); vTaskDelay(5/portTICK_RATE_MS); // reset and delay 100us (see sec 9.3)
    // writeMagRegister(AK09916::CNTL2, 0x08); vTaskDelay(10/portTICK_RATE_MS); // set sample rate: CNTL2[4:0] = 0x0:PWR DOWN, 0x02:10Hz, 0x04:20Hz, 0x06:50Hz, 0x08:100Hz

    // configure slave0 to write to EXT_SLV_SENS_DATA_00
    setBank(3);
    writeRegister(I2C_SLV0_ADDR, AK09916::I2C_ADDR | 0x80);    
	writeRegister(I2C_SLV0_REG, AK09916::ST1); // start reading at ST1
	writeRegister(I2C_SLV0_CTRL, 0x80 | 10); // read in 10 bytes continuously at sample rate of gyro
}


void ICM20948::configI2CAuxilary(AUXI2C mode) {
    if(mode==MASTER_MODE) {
        setBank(0);
        writeRegister(INT_PIN_CFG, 0x00); // disable bypass mode
        setBank(3);
        writeRegister(I2C_MST_CTRL, 0x17); // Full-stop and 400KHz
        setBank(0);
        writeRegister(USER_CTRL, 0x20);   // Enable I2C Master I/F Module
    } else if(mode==PASSTHROUGH_MODE){
        setBank(0);
        writeRegister(USER_CTRL, 0x00);   // disable I2C Master I/F Module
        writeRegister(INT_PIN_CFG, 0x02); // enable bypass mode
        writeRegister(USER_CTRL, 0x02);   // reset I2C Master Module (not sure why this is needed)
    }
}


void ICM20948::readSensors(float *data) {
    uint8_t d[20], idx;
    int16_t ai, aj, ak, gi, gj, gk, Ti, mi, mj, mk;
    float s, ax, ay, az, gx, gy, gz, T, mx, my, mz;

    setBank(0);
    read(ACCEL_XOUT_H, d, 20);
    ai = d[ 0]<<8 | d[ 1];  aj = d[ 2]<<8 | d[ 3];  ak = d[ 4]<<8 | d[ 5];
    gi = d[ 6]<<8 | d[ 7];  gj = d[ 8]<<8 | d[ 9];  gk = d[10]<<8 | d[11];
    mi = d[16]<<8 | d[15];  mj = d[18]<<8 | d[17];  mk = d[20]<<8 | d[19];
    Ti = d[12]<<8 | d[13];

    setBank(2);
    readRegister(ACCEL_CONFIG, &idx);
    s = accel_sensitvity[(idx & 0x06) >> 1];
    ax = float(ai)/s; ay = float(aj)/s; az = float(ak)/s; 

    readRegister(GYRO_CONFIG_1, &idx);
    s = gyro_sensitvity[(idx & 0x06) >> 1];
    gx = float(gi)/s; gy = float(gj)/s; gz = float(gk)/s; 

    T = (Ti-21.0f)/333.87f + 21.0f;

    mx = mi*mag_sensitivity; my = mj*mag_sensitivity; mz = mk*mag_sensitivity; 

    data[0] = ax; data[1] = ay, data[2] = az;
    data[3] = gx; data[4] = gy, data[5] = gz;
    data[6] = mx; data[7] = my, data[8] = mz;
    data[9] = T;
}


void ICM20948::readMagRegister(uint8_t register_address, uint8_t* value) {
    setBank(3);
    writeRegister(I2C_SLV0_ADDR, AK09916::I2C_ADDR | 0x80);
    writeRegister(I2C_SLV0_REG, register_address);
    writeRegister(I2C_SLV0_CTRL, 0x80 | 1);  // Read 1 byte   
    triggerMagIO();
    setBank(0);
    readRegister(EXT_SLV_SENS_DATA_00, value);
}


void ICM20948::readMagRegisters(uint8_t register_address, uint8_t* data, uint length) {
    setBank(3);
    writeRegister(I2C_SLV0_ADDR, AK09916::I2C_ADDR | 0x80);    
	writeRegister(I2C_SLV0_REG, register_address);
	writeRegister(I2C_SLV0_CTRL, 0x80 | length);
    triggerMagIO();
    setBank(0);
	read(EXT_SLV_SENS_DATA_00, data, length);     
}


void ICM20948::writeMagRegister(uint8_t register_address, uint8_t value) {
    setBank(3);
    writeRegister(I2C_SLV0_ADDR, AK09916::I2C_ADDR);
	writeRegister(I2C_SLV0_REG, register_address);
	writeRegister(I2C_SLV0_DO, value);
    triggerMagIO();
}


void ICM20948::triggerMagIO() {
    uint8_t user;
    setBank(0);
    readRegister(USER_CTRL, &user);
    writeRegister(USER_CTRL, user | 0x20);
    vTaskDelay(3 / portTICK_RATE_MS);
    writeRegister(USER_CTRL, user);
}


void ICM20948::setBank(uint8_t value) {
    writeRegister(BANKSEL, value << 4);
}


void ICM20948::setGyroRange(GyroRange range) {
    /* Set the gyro full range range to +- supplied value. */
    setBank(2);
    uint8_t value;
    readRegister(GYRO_CONFIG_1, &value); 
    value &= 0b11111001;
    value |= (range & 0x03) << 1;
    writeRegister(GYRO_CONFIG_1, value);
}


void ICM20948::setGyroDataRate(GyroDataRate rate) {
    /* Set the gyro sample rate dividier */
    // REF[1]  Table 1. Gyroscope Specifications - OUTPUT DATA RATE
    // REF[2]  Section 10.1 GYRO_SMPLRT_DIV
    // REF[3]  Table 16. Gyroscope Configuration 1
    // If GYRO_FCHOICE = 0 then ODR = 9kHz
    // If GYRO_FCHOICE = 1 then ODR = 1.125 kHz/(1+GYRO_SMPLRT_DIV[7:0])
    // Note there is a discrepancy in the datasheet to calculate ODR 
    // e.g. contrast REF[1,3] with REF[2], it is believed the formula in REF[2] is incorrect, however this has not been verified.
    setBank(2);
    writeRegister(GYRO_SMPLRT_DIV, rate);    
}


void ICM20948::setGyroLowpass(bool enabled, uint8_t mode) {
    /* Configure the gyro low pass filter */
    setBank(2);
    uint8_t value;
    readRegister(GYRO_CONFIG_1, &value);
    value &= 0b11000110;
    if (enabled) value |= 0b1;
    value |= (mode & 0x07) << 3;
    writeRegister(GYRO_CONFIG_1, value);
}


void ICM20948::setAccelRange(AccelRange range) {
    /* Set the accel full range range to +- supplied value. */
    setBank(2);
    uint8_t value;
    readRegister(ACCEL_CONFIG, &value); 
    value &= 0b11111001;
    value |= (range & 03) << 1;
    writeRegister(ACCEL_CONFIG, value);
}


void ICM20948::setAccelDataRate(AccelDataRate rate) {
    /* Set the accel sample rate in Hz. */
    // ODR = 1.125 kHz/(1+GYRO_SMPLRT_DIV[11:0])
    setBank(2);
    uint8_t data[2];
    data[0] = uint8_t(rate >> 8) & 0x0f;
    data[1] = uint8_t(rate);
    write(ACCEL_SMPLRT_DIV_1, data, 2); 
}


void ICM20948::setAccelLowpass(bool enabled, uint8_t mode) {
    /* Configure the accel low pass filter */
    setBank(2);
    uint8_t value;
    readRegister(ACCEL_CONFIG, &value);
    value &= 0b11000110;
    if (enabled) value |= 0b1;
    value |= (mode & 0x07) << 3;
    writeRegister(ACCEL_CONFIG, value);
}


float ICM20948::readTemperature() {
    /* read the current IMU temperature */
    // Offset and sensitivity - defined in electrical characteristics, and TEMP_OUT_H/L of datasheet
    setBank(0);
    uint8_t data[2];
    read(TEMP_OUT_H, data, 2);
    int16_t temp = data[1] | (uint16_t)data[0] << 8;
    return (temp-21.0f)/333.87f + 21.0f;
}
