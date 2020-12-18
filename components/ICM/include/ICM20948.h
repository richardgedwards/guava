#ifndef __ICM20948_H__
#define __ICM20948_H__

#include "i2cw.h"

// ICM20948MCM (multi-chip-module)
//     ICM20948Module/Die
//         Interface I2C
//         Interface SPI
//         Accelerometer
//         Gyroscope
//         Digital Motion Processor/DMP
//         Temperature Sensor
//         FIFO
//         FSYNC
//         Interrupts
//     AK09916Module/Die
//         Interface I2c 
//         Magnetometer
// https://github.com/kriswiner/MPU9250/issues/86 - reading magnetometer


class AK09916 : public I2CDevice {
    public:
        AK09916(I2CMaster &i2c);

        // void readMagRegister(uint8_t register_address, uint8_t* data);
        // void writeMagRegister(uint8_t register_address, uint8_t data);

    protected:
        // protected data-members

    public:
        const float sensitivity = 0.15f;

        static const uint8_t I2C_ADDR = 0x0c;
        static const uint8_t CHIP_ID = 0x09;
        static const uint8_t WIA1 = 0x00;
        static const uint8_t WIA2 = 0x01;
        static const uint8_t ST1 = 0x10;
        static const uint8_t ST1_DOR = 0b00000010;   // Data overflow bit
        static const uint8_t ST1_DRDY = 0b00000001;  // Data .ready bit
        static const uint8_t HXL = 0x11;
        static const uint8_t ST2 = 0x18;
        static const uint8_t ST2_HOFL = 0b00001000;  // Magnetic sensor overflow bit
        static const uint8_t CNTL2 = 0x31;
        static const uint8_t CNTL2_MODE = 0b00001111;
        static const uint8_t CNTL2_MODE_OFF = 0;
        static const uint8_t CNTL2_MODE_SINGLE = 0b00001;
        static const uint8_t CNTL2_MODE_CONT1 = 0b00010;
        static const uint8_t CNTL2_MODE_CONT2 = 0b00100;
        static const uint8_t CNTL2_MODE_CONT3 = 0b00110;
        static const uint8_t CNTL2_MODE_CONT4 = 0b01000;
        static const uint8_t CNTL2_MODE_TEST = 0b10000;
        static const uint8_t CNTL3 = 0x32;
};





class ICM20948 : public I2CDevice {


    public:
    enum GyroRange { FSR_250DPS, FSR_500DPS, FSR_1000DPS, FSR_2000DPS };
    enum AccelRange { FSR_2G, FSR_4G, FSR_8G, FSR_16G };
    enum GyroDataRate{ GDR_1250HZ=0, GDR_550HZ=1, GDR_275HZ=3, GDR_220HZ=4, GDR_110HZ=9, GDR_100HZ=10, GDR_50HZ=50, GDR_44HZ=24, GDR_20HZ=54, GDR_10HZ=109, GDR_5HZ=219 };
    enum AccelDataRate{ ADR_1250HZ=0, ADR_625HZ=1, ADR_250HZ=4, ADR_125HZ=9, ADR_50HZ=24, ADR_25HZ=49, ADR_10HZ=124, ADR_5HZ=249, ADR_1HZ=1249 };


    public:
    ICM20948(I2CMaster &i2c);

    void setBank(uint8_t value);

    void readIMU(float *imudata);
    void readAHRS(uint8_t *data);

    void setGyroRange(GyroRange range=FSR_250DPS);
    void setGyroLowpass(bool enabled=true, uint8_t mode=5);
    void setGyroDataRate(GyroDataRate rate=GDR_1250HZ);
    float getGyroSamplerate();

    void setAccelRange(AccelRange range=FSR_2G);
    void setAccelLowpass(bool enabled=true, uint8_t mode=5);
    void setAccelDataRate(AccelDataRate rate=ADR_1250HZ);

    void writeMagRegister(uint8_t register_address, uint8_t data);
    void readMagRegister(uint8_t register_address, uint8_t* value);
    void readMagRegisters(uint8_t register_address, uint8_t* data, uint length);
    void triggerMagIO();
    // void readMagData(float *magdata);
    // // bool isMagReady();

    float readTemperature();

    // def read_magnetometer_data(timeout=1.0);
    // def read_accelerometer_gyro_data();

    protected:
    uint8_t _bank;


    public:
    const float accel_sensitvity[4] = {16384.f, 8192.f, 4096.f, 2048.f};
    const float gyro_sensitvity[4] = {131.f, 65.5f, 32.8f, 16.4f};
    // const float mag_sensitivity = 0.15;

    static const uint8_t CHIP_ID = 0xEA;
    static const uint8_t I2C_ADDR = 0x68;
    static const uint8_t I2C_ADDR_ALT = 0x69;
    static const uint8_t setBank_SEL = 0x7f;

    static const uint8_t I2C_MST_ODR_CONFIG = 0x00;
    static const uint8_t I2C_MST_CTRL = 0x01;
    static const uint8_t I2C_MST_DELAY_CTRL = 0x02;
    static const uint8_t I2C_SLV0_ADDR = 0x03;
    static const uint8_t I2C_SLV0_REG = 0x04;
    static const uint8_t I2C_SLV0_CTRL = 0x05;
    static const uint8_t I2C_SLV0_DO = 0x06;
    static const uint8_t EXT_SLV_SENS_DATA_00 = 0x3B;

    static const uint8_t GYRO_SMPLRT_DIV = 0x00;
    static const uint8_t GYRO_CONFIG_1 = 0x01;
    static const uint8_t GYRO_CONFIG_2 = 0x02;

    // setBank 0
    static const uint8_t WHO_AM_I = 0x00;
    static const uint8_t USER_CTRL = 0x03;
    static const uint8_t PWR_MGMT_1 = 0x06;
    static const uint8_t PWR_MGMT_2 = 0x07;
    static const uint8_t INT_PIN_CFG = 0x0F;

    // setBank 2
    static const uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
    static const uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
    static const uint8_t ACCEL_INTEL_CTRL = 0x12;
    static const uint8_t ACCEL_WOM_THR = 0x13;
    static const uint8_t ACCEL_CONFIG = 0x14;
    static const uint8_t ACCEL_XOUT_H = 0x2D;
    static const uint8_t GRYO_XOUT_H = 0x33;

    static const uint8_t TEMP_OUT_H = 0x39;
    static const uint8_t TEMP_OUT_L = 0x3A;

    static const uint8_t MAG_I2C_ADDR = 0x0c;
    static const uint8_t MAG_CHIP_ID = 0x09;
    static const uint8_t MAG_WIA = 0x01;
    static const uint8_t MAG_ST1 = 0x10;
    static const uint8_t MAG_ST1_DOR = 0b00000010;   // Data overflow bit
    static const uint8_t MAG_ST1_DRDY = 0b00000001;  // Data .ready bit
    static const uint8_t MAG_HXL = 0x11;
    static const uint8_t MAG_ST2 = 0x18;
    static const uint8_t MAG_ST2_HOFL = 0b00001000;  // Magnetic sensor overflow bit
    static const uint8_t MAG_CNTL2 = 0x31;
    static const uint8_t MAG_CNTL2_MODE = 0b00001111;
    static const uint8_t MAG_CNTL2_MODE_OFF = 0;
    static const uint8_t MAG_CNTL2_MODE_SINGLE = 1;
    static const uint8_t MAG_CNTL2_MODE_CONT1 = 2;
    static const uint8_t MAG_CNTL2_MODE_CONT2 = 4;
    static const uint8_t MAG_CNTL2_MODE_CONT3 = 6;
    static const uint8_t MAG_CNTL2_MODE_CONT4 = 8;
    static const uint8_t MAG_CNTL2_MODE_TEST = 16;
    static const uint8_t MAG_CNTL3 = 0x32;
};

#endif // __ICM20948_H__