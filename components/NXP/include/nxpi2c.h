#ifndef __NXPI2C_H__
#define __NXPI2C_H__

#include "driver/i2c.h"
#include "i2cw.h"


class I2CNXP 
{
    protected:
        uint8_t _device_address;
        I2CMaster &_i2c;

    public:
        I2CNXP(uint8_t device_address, I2CMaster &i2c) : _device_address(device_address), _i2c(i2c) { }

    esp_err_t read_register(uint8_t register_address, uint8_t *data) {
        // see datasheet section 3.1.3.3 Figure 7. I2C data sequence diagram <Single Byte Read>
        esp_err_t e;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        e = i2c_master_cmd_begin(_i2c.get_port(), cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return e;
    }

    esp_err_t write_register(uint8_t register_address, uint8_t data) {
        // see datasheet section 3.1.3.3 Figure 7. I2C data sequence diagram  <Single Byte Write>
        esp_err_t e;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);
        i2c_master_stop(cmd);
        e = i2c_master_cmd_begin(_i2c.get_port(), cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return e;
    }

    esp_err_t read(uint8_t register_address, uint8_t *data, size_t data_len) {
        // see datasheet section 3.1.3.3 Figure 7. I2C data sequence diagram <Multiple Byte Read>
        esp_err_t e;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
        i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        e = i2c_master_cmd_begin(_i2c.get_port(), cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return e;
    }

    void printb(uint8_t c) {
        uint8_t i1 = (1 << (sizeof(c)*8-1));
        for(; i1; i1 >>= 1) printf("%d",(c&i1)!=0);
        printf("\n");
    }
};

#endif // __NXPI2C_H__