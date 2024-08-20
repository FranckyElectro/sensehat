#include "Hts221.hpp"
#include <cstdio>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <i2c/smbus.h>
#ifdef __cplusplus
}
#endif

uint8_t Hts221::H0_rH_x2 = 0;
uint8_t Hts221::H1_rH_x2 = 0;
uint16_t Hts221::T0_degC_x8 = 0;
uint16_t Hts221::T1_degC_x8 = 0;
int16_t Hts221::H0_T0_OUT = 0;
int16_t Hts221::H1_T0_OUT = 0;
int16_t Hts221::T0_OUT = 0;
int16_t Hts221::T1_OUT = 0;

Hts221::Hts221(int fd) {
    _fd = fd;
}

void Hts221::init() {
    // Configuring humidity sensor

    // Setting humidity and temperature average from AV_CONF register
    i2c_read(_fd, 0x10, ucTemp, 1);
    // ------------- AV_CONF -----------
    // b7 b6        b5 b4 b3    b2 b1 b0
    // Reserved     AVGT2:0     AVGH2:0
    // ---------------------------------
    ucTemp[0] &= 0xc0; // keep reserved bits
    ucTemp[0] |= 0x1b; // AVGT=16, AVGH=32
    i2c_write(_fd, 0x10, ucTemp, 1);

    // Setting CTRL_REG1:3
    i2c_read(_fd, 0x20 | 0x80, ucTemp, 3);  // get CTRL_REG 1-3
    ucTemp[0] &= 0x78;                      // keep reserved bits
    ucTemp[0] |= 0x81;                      // turn on device + 1Hz sample rate
    ucTemp[1] &= 0x7c;                      // turn off (heater + boot + one shot)
    i2c_write(_fd, 0x20 | 0x80, ucTemp, 3); // turn on + set sample rate

    // Get the H/T calibration values
    i2c_read(_fd, 0x30 | 0x80, ucTemp, 16);
    H0_rH_x2 = ucTemp[0];
    H1_rH_x2 = ucTemp[1];
    T0_degC_x8 = ucTemp[2];
    T1_degC_x8 = ucTemp[3];

    T0_degC_x8 |= ((ucTemp[5] & 0x3) << 8); // 2 msb
    T1_degC_x8 |= ((ucTemp[5] & 0xc) << 6); // 2 msb

    H0_T0_OUT = ucTemp[6] | (ucTemp[7] << 8);
    H1_T0_OUT = ucTemp[10] | (ucTemp[11] << 8);
    T0_OUT = ucTemp[12] | (ucTemp[13] << 8);
    T1_OUT = ucTemp[14] | (ucTemp[15] << 8);

    // printf("Humidity sensor initialized\n\r");
}

int Hts221::getTemp(float* temp) {
    uint8_t status;

    // Power down the device (clean start)
    i2c_smbus_write_byte_data(_fd, HTS221_CTRL_REG1, 0x00);

    // Turn on the humidity sensor analog front end in single shot mode
    i2c_smbus_write_byte_data(_fd, HTS221_CTRL_REG1, 0x84);

    // Run one-shot measurement (temperature and humidity).
    // The set bit will be reset by the sensor itself after execution
    // (self-clearing bit)
    i2c_smbus_write_byte_data(_fd, HTS221_CTRL_REG2, 0x01);
    do {
        usleep(25 * 1000); // 25 milliseconds
        status = i2c_smbus_read_byte_data(_fd, HTS221_CTRL_REG2);
    } while (status != 0);


    // Read calibration temperature LSB (ADC) data
    // (temperature calibration x-data for two points)
    uint8_t t0_out_l = i2c_smbus_read_byte_data(_fd, HTS221_T0_OUT_L);
    uint8_t t0_out_h = i2c_smbus_read_byte_data(_fd, HTS221_T0_OUT_H);
    uint8_t t1_out_l = i2c_smbus_read_byte_data(_fd, HTS221_T1_OUT_L);
    uint8_t t1_out_h = i2c_smbus_read_byte_data(_fd, HTS221_T1_OUT_H);


    // Read calibration temperature (°C) data
    // (temperature calibration y-data for two points)
    uint8_t t0_degC_x8 = i2c_smbus_read_byte_data(_fd, HTS221_T0_DEGC_X8);
    uint8_t t1_degC_x8 = i2c_smbus_read_byte_data(_fd, HTS221_T1_DEGC_X8);
    uint8_t t1_t0_msb = i2c_smbus_read_byte_data(_fd, HTS221_T1T0_MSB);

    // make 16 bit values (bit shift)
    // (temperature calibration x-values)
    int16_t T0_OUT = t0_out_h << 8 | t0_out_l;
    int16_t T1_OUT = t1_out_h << 8 | t1_out_l;

    // make 16 and 10 bit values (bit mask and bit shift)
    uint16_t T0_DegC_x8 = (t1_t0_msb & 3) << 8 | t0_degC_x8;
    uint16_t T1_DegC_x8 = ((t1_t0_msb & 12) >> 2) << 8 | t1_degC_x8;

    // Calculate calibration values
    // (temperature calibration y-values)
    double T0_DegC = T0_DegC_x8 / 8.0;
    double T1_DegC = T1_DegC_x8 / 8.0;

    // Solve the linear equasions 'y = mx + c' to give the
    // calibration straight line graphs for temperature and humidity
    double t_gradient_m = (T1_DegC - T0_DegC) / (T1_OUT - T0_OUT);
    double t_intercept_c = T1_DegC - (t_gradient_m * T1_OUT);

    // Read the ambient temperature measurement (2 bytes to read)
    uint8_t t_out_l = i2c_smbus_read_byte_data(_fd, HTS221_TEMP_OUT_L);
    uint8_t t_out_h = i2c_smbus_read_byte_data(_fd, HTS221_TEMP_OUT_H);

    // make 16 bit value
    int16_t T_OUT = t_out_h << 8 | t_out_l;

    // Calculate ambient temperature
    *temp = (t_gradient_m * T_OUT) + t_intercept_c;

    return 0;
}

int Hts221::getTemp(float* temp, float* hum) {
    unsigned char ucTemp[4];
    int8_t rc;
    int16_t H_T_out, T_out;
    uint8_t T0_degC, T1_degC, H0_rh, H1_rh;
    float tmp;

    rc = i2c_read(_fd, 0x28 | 0x80, ucTemp, 4);
    if (rc == 4) {
        H_T_out = ucTemp[0] + (ucTemp[1] << 8);
        T_out = ucTemp[2] + (ucTemp[3] << 8);
        T0_degC = T0_degC_x8 / 8;
        T1_degC = T1_degC_x8 / 8;
        H0_rh = H0_rH_x2 / 2;
        H1_rh = H1_rH_x2 / 2;
        tmp = (H_T_out - H0_T0_OUT) * (H1_rh - H0_rh);
        *hum = tmp / (H1_T0_OUT - H0_T0_OUT) + H0_rh;
        tmp = (T_out - T0_OUT) * (T1_degC - T0_degC);
        *temp = tmp / (T1_OUT - T0_OUT) + T0_degC;
        return 1;
    }
    return 0; // not ready
}