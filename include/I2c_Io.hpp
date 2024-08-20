#ifndef _I2C_IO_HPP
#define _I2C_IO_HPP

#include <cstdint>

class I2c_Io {
protected:
    uint8_t i2c_write(int fd, uint8_t cmd, uint8_t* buf, uint16_t length);
    uint8_t i2c_read(int fd, uint8_t cmd, uint8_t* buf, uint8_t length);
};


#endif