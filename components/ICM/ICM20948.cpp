#include "ICM20948.h"
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <iostream>

using std::cout;
using std::endl;
using std::runtime_error;


AK09916::AK09916(I2CMaster &i2c) : I2CDevice(I2C_ADDR, i2c) {
    uint8_t value;
    readRegister(WIA2, &value);
    if (value != CHIP_ID) throw(runtime_error("AK09916 not found"));
    writeRegister(CNTL3, 0x01);  // reset
    writeRegister(CNTL2, 0x08);  // mode 4 - 100 Hz sample frequency
    vTaskDelay(10/ portTICK_RATE_MS);
}



ICM20948::ICM20948(I2CMaster &i2c) : I2CDevice(I2C_ADDR_ALT, i2c), _bank(0) { 

    uint8_t value;
    setBank(0);
    readRegister(WHO_AM_I, &value); 
    if (value != CHIP_ID) throw(runtime_error("ICM20948 not found"));
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

    // put I2C Master in bypass-mode
    setBank(0);
    // writeRegister(USER_CTRL, 0x00);  // not needed since this is the default
    writeRegister(INT_PIN_CFG, 0x02); // set to bypass mode
    writeRegister(USER_CTRL, 0x02);  // reset I2C Master Module

    // init magnetometer
    try {AK09916 mag(i2c);} catch(const runtime_error &e) {cout << e.what() << endl;}

    // setup and use I2C master to read sensors - ref: https://github.com/kriswiner/MPU9250/issues/86
    // enableI2CMaster mode 
    setBank(0);
    writeRegister(INT_PIN_CFG, 0x00); // disable bypass mode

    // configure I2CMaster
    // I2C_MST_CLK[7] = 0; Enables multi-master capability (If there are other masters on the bus switch this bit high)
    // I2C_MST_CLK[5:6] Reserved
    // I2C_MST_CLK[4] = 1; 1=Full stop between reads, 0=restart
    // I2C_MST_CLK[3:0] = 7; recommended in data sheet sec 14.5
    setBank(3);
    writeRegister(I2C_MST_CTRL, 0x17);
    setBank(0);
    writeRegister(USER_CTRL, 0x20);   // Enable I2C Master IF Module

    readMagRegister(MAG_WIA, &value);  if (value != MAG_CHIP_ID) printf("MAG not found\n"); else printf("MAG found\n");

    // configure I2CMaster to read from slave0
    setBank(3);
    // writeRegister(I2C_MST_DELAY_CTRL, 0x01); // not sure what this does (delay on how oftern it is written?)
    writeRegister(I2C_SLV0_ADDR, MAG_I2C_ADDR); // MAG_I2C_ADDR[7]=0 defines a write operation 
    writeRegister(I2C_SLV0_REG, MAG_ST1);  // where to begin data transfer
    // I2C_SLV0_CTRL[7] = 1; enable reading data from slave 0
    // I2C_SLV0_CTRL[6] = 1; swap bytes when reading a word
    // I2C_SLV0_CTRL[5] = 0; do not write register values, only read or write data
    // I2C_SLV0_CTRL[4] = 1; group 1 and 2 (odd) bytes to gether
    // I2C_SLV0_CTRL[3:0] = 1010; Number of bytes to read from I2C slave 0
    writeRegister(I2C_SLV0_CTRL, 0xda);


    // readMagRegister(MAG_WIA, &value);  if (value != MAG_CHIP_ID) printf("MAG not found\n"); else printf("MAG found\n");
    // readMagRegisters(MAG_HXL, data, 6); 
    // printf("%02x %02x %02x %02x %02x %02x\n", data[0], data[1], data[2], data[3], data[4], data[5]);
    // read(EXT_SLV_SENS_DATA_00, data, 6);
    // printf("%02x %02x %02x %02x %02x %02x\n", data[0], data[1], data[2], data[3], data[4], data[5]);

    // reset the magnetometer
    // writeMagRegister(MAG_CNTL3, 0x01);
    // readMagRegister(MAG_CNTL3, &value);
    // while (value==0x01) {
    //     vTaskDelay(1 / portTICK_RATE_MS);
    //     readMagRegister(MAG_CNTL3, &value);
    // }

   
}


void ICM20948::readMagRegister(uint8_t register_address, uint8_t* value) {
    setBank(3);
    writeRegister(I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80);
    writeRegister(I2C_SLV0_REG, register_address);
    writeRegister(I2C_SLV0_CTRL, 0x80 | 1);  // Read 1 byte   
    triggerMagIO();
    readRegister(EXT_SLV_SENS_DATA_00, value);
}


void ICM20948::readMagRegisters(uint8_t register_address, uint8_t* data, uint length) {
    setBank(3);
    writeRegister(I2C_SLV0_ADDR, MAG_I2C_ADDR | 0x80);    
	writeRegister(I2C_SLV0_REG, register_address);
	writeRegister(I2C_SLV0_CTRL, 0x80 | length);
    triggerMagIO();
	read(EXT_SLV_SENS_DATA_00, data, length);     
}


void ICM20948::writeMagRegister(uint8_t register_address, uint8_t value) {
    setBank(3);
    writeRegister(I2C_SLV0_ADDR, MAG_I2C_ADDR);
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


// void ICM20948::readMagData(float *magdata) {
//     uint8_t data[6];
//     int16_t mi, mj, mk;
//     writeMagRegister(MAG_CNTL2, 0x01);  // Trigger single measurement
//     while (!isMagReady())
//         vTaskDelay(1 / portTICK_RATE_MS);
//     readMagRegisters(MAG_HXL, data, 6);
//     // setBank(0);
//     // read(EXT_SLV_SENS_DATA_00, data, 6);

//     mi = data[1]<<8 | data[0]; mj = data[3]<<8 | data[2]; mk = data[5]<<8 | data[4]; 
//     // mi = (((int16_t)data[1]) << 8) | data[0];
//     // mj = (((int16_t)data[3]) << 8) | data[2];
//     // mk = (((int16_t)data[5]) << 8) | data[4];
//     // printf("%d, %d, %d\n", mi, mj, mk);
//     magdata[0] = mi;// * mag_sensitivity;
//     magdata[1] = mj;// * mag_sensitivity;
//     magdata[2] = mk;// * mag_sensitivity;
// }


// bool ICM20948::isMagReady() {
//     uint8_t value;
//     readMagRegister(MAG_ST1, &value);
//     return((value & 0x01) > 0);
// }


void ICM20948::readAHRS(uint8_t *data) {
    setBank(0);
    read(ACCEL_XOUT_H, data, 24);
}


void ICM20948::readIMU(float *imudata) {

    uint8_t data[12], idx;
    int16_t ai, aj, ak, gi, gj, gk;
    float s, ax, ay, az, gx, gy, gz;

    setBank(0);
    read(ACCEL_XOUT_H, data, 12);
    ai = data[0]<<8 | data[1]; aj = data[2]<<8 | data[3]; ak = data[4]<<8 | data[5];
    gi = data[6]<<8 | data[7]; gj = data[8]<<8 | data[9]; gk = data[10]<<8 | data[11];

    setBank(2);
    readRegister(ACCEL_CONFIG, &idx);
    s = accel_sensitvity[(idx & 0x06) >> 1];
    ax = float(ai)/s; ay = float(aj)/s; az = float(ak)/s; 

    readRegister(GYRO_CONFIG_1, &idx);
    s = gyro_sensitvity[(idx & 0x06) >> 1];
    gx = float(gi)/s; gy = float(gj)/s; gz = float(gk)/s; 

    imudata[0] = gx; imudata[1] = gy, imudata[2] = gz;
    imudata[3] = ax; imudata[4] = ay, imudata[5] = az;
}


void ICM20948::setBank(uint8_t value) {
    if (_bank != value) {
        writeRegister(setBank_SEL, value << 4);
        _bank = value;
    }
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
