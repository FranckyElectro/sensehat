#ifndef _LPS25H_HPP
#define _LPS25H_HPP

#include "I2c_Io.hpp"

class Lps25h : public I2c_Io {
private:
    int _fd;    

public:
    Lps25h(int fd);     
    int getTempAndPressure(float* temp, float* hum);
};

#endif