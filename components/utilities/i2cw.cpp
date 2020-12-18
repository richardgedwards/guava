#include "i2cw.h"
#include "esp_exception.h"  // define THROW macro
using namespace idf;



// I2CMaster
I2CMaster::I2CMaster(i2c_port_t port, int sda_io_num, int scl_io_num, uint32_t clk_speed)
    : I2CBase(port, sda_io_num, scl_io_num)
{
    _config.sda_pullup_en = GPIO_PULLUP_ENABLE; 
    _config.scl_pullup_en = GPIO_PULLUP_ENABLE; 
    _config.mode = I2C_MODE_MASTER;
    _config.master.clk_speed = clk_speed;

    THROW(i2c_param_config(_port, &_config));
    THROW(i2c_driver_install(_port, _config.mode, 0, 0, 0));
}


void I2CDevice::readRegister(uint8_t register_address, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    THROW(i2c_master_cmd_begin(_i2c.getPort(), cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
}


void I2CDevice::writeRegister(uint8_t register_address, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    THROW(i2c_master_cmd_begin(_i2c.getPort(), cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
}


void I2CDevice::read(uint8_t register_address, uint8_t *data, size_t data_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    THROW(i2c_master_cmd_begin(_i2c.getPort(), cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
}


void I2CDevice::write(uint8_t register_address, uint8_t *data, size_t data_len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_device_address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, register_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, true);
    i2c_master_stop(cmd);
    THROW(i2c_master_cmd_begin(_i2c.getPort(), cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
}


// I2CSlave
I2CSlave::I2CSlave(
    i2c_port_t port, 
    int sda_io_num, 
    int scl_io_num,
    uint16_t i2c_addr,
    size_t rx_buf_length,
    size_t tx_buf_length,
    bool addr_10bit_en
    ) : I2CBase(port, sda_io_num, scl_io_num)
{
    _config.sda_pullup_en = GPIO_PULLUP_DISABLE; 
    _config.scl_pullup_en = GPIO_PULLUP_DISABLE; 
    _config.mode = I2C_MODE_SLAVE;
    _config.slave.slave_addr = i2c_addr;
    _config.slave.addr_10bit_en = addr_10bit_en;
    i2c_param_config(_port, &_config);
    i2c_driver_install(_port, _config.mode, rx_buf_length, tx_buf_length, 0);
}