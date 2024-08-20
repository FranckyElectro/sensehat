#ifndef _HTS221_HPP
#define _HTS221_HPP

#include "I2c_Io.hpp"

#define HTS221_CTRL_REG1 0x20
#define HTS221_CTRL_REG2 0x21
#define HTS221_CTRL_REG3 0x22
#define HTS221_T0_OUT_L 0x3c
#define HTS221_T0_OUT_H 0x3d
#define HTS221_T1_OUT_L 0x3e
#define HTS221_T1_OUT_H 0x3f
#define HTS221_T0_DEGC_X8 0x32
#define HTS221_T1_DEGC_X8 0x33
#define HTS221_T1T0_MSB 0x35
#define HTS221_TEMP_OUT_L 0x2A
#define HTS221_TEMP_OUT_H 0x2B

class Hts221 : public I2c_Io {
private:
    int _fd;
    static uint8_t H0_rH_x2, H1_rH_x2 ;
    static uint16_t T0_degC_x8, T1_degC_x8;
    static int16_t H0_T0_OUT, H1_T0_OUT, T0_OUT, T1_OUT;
    uint8_t ucTemp[32];

public:
    Hts221(int fd);
    void init();
    int getTemp(float* temp);
    int getTemp(float* temp, float* hum);
};

#endif