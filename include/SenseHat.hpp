#ifndef _SENSEHAT_HPP
#define _SENSEHAT_HPP

#include <cstdint>
#include <cstdio>
#include "SenseHatDefs.hpp"
#include "I2c_Io.hpp"
#include "Hts221.hpp"
#include "Lps25h.hpp"

class Sensehat : public I2c_Io
{
public:
    static int delay_ms(int ms);
    int8_t init(uint8_t hat_addr);
    uint8_t read_joystick();
    uint8_t wait_for_joystick_change();
    int add_joystick_event(eventHandler func);
    Sensehat();
    ~Sensehat();

private:
    int file_led;
    int file_acc;
    int file_mag;
    int file_hum;
    int file_pres;
    static uint8_t LEDArray[192];
    intVec interruptVector;    
    /* uint8_t i2c_write(int fd, uint8_t cmd, uint8_t* buf, uint16_t length);
    uint8_t i2c_read(int fd, uint8_t cmd, uint8_t* buf, uint8_t length); */
    void check_i2c_bus();
    FILE *do_command(const char *cmd);
};

#endif