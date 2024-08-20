#include "I2c_Io.hpp"
#include <cstdio>
#include <cstring>
#include <unistd.h>


uint8_t I2c_Io::i2c_write(int fd, uint8_t cmd, uint8_t* buf, uint16_t length) {
    uint8_t ucTemp[512];
    uint8_t rc;

    if (length > 511 || length < 1 || buf == NULL)
        return -1; // invalid write
    ucTemp[0] = cmd; // send the register number first
    memcpy(&ucTemp[1], buf, length); // followed by the data
    rc = write(fd, ucTemp, length + 1);
    return rc - 1;
}

uint8_t I2c_Io::i2c_read(int fd, uint8_t cmd, uint8_t* buf, uint8_t length) {
    uint8_t rc;
    rc = write(fd, &cmd, 1);
    if (rc == 1) {
        rc = read(fd, buf, length);
    }
    return rc;
};