#ifndef __I2CW_H__
#define __I2CW_H__

#include "driver/i2c.h"
#include <stdio.h>

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */


class I2CBase {
    protected:
    i2c_port_t _port;
    i2c_config_t _config;

    public:
    I2CBase(i2c_port_t port, int sda_io_num, int scl_io_num)  : _port(port) {
        _config.sda_io_num = sda_io_num; 
        _config.scl_io_num = scl_io_num; 
    }
    inline void setPort(i2c_port_t port) { _port = port; }
    inline i2c_port_t getPort() { return _port; }

    void printb(uint8_t c) {
        uint8_t i1 = (1 << (sizeof(c)*8-1));
        for(; i1; i1 >>= 1) printf("%d",(c&i1)!=0);
        printf("\n");
    }
};


class I2CMaster : public I2CBase {
    public:
    I2CMaster(i2c_port_t port=0, int sda_io_num=5, int scl_io_num=4, uint32_t clk_speed=400000);
};


class I2CDevice {
    protected:
    uint8_t _device_address;
    I2CMaster &_i2c;

    public:
    I2CDevice(uint8_t device_address, I2CMaster &i2c) : _device_address(device_address), _i2c(i2c) { }
    virtual void readRegister(uint8_t register_address, uint8_t *data);
    virtual void writeRegister(uint8_t register_address, uint8_t data);
    virtual void read(uint8_t register_address, uint8_t *data, size_t data_len);
    virtual void write(uint8_t register_address, uint8_t *data, size_t data_len);
};


class I2CSlave : public I2CBase {
    public:
    I2CSlave(i2c_port_t port=1, int sda_io_num=4, int scl_io_num=4, uint16_t i2c_addr=0x0, size_t rx_buf_length=1024, 
                size_t tx_buf_length=1024, bool addr_10bit_en=0);
};

#endif // __I2CW_H__