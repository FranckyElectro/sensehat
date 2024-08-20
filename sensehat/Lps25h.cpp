#include "Lps25h.hpp"
#include <cstdio>

Lps25h::Lps25h(int fd) {
    _fd = fd;
}

int Lps25h::getTempAndPressure(float* temp, float* pressure) {
    unsigned char ucTemp[8];
    int rc, P, T;
    // prepare pressure sensor
    ucTemp[0] = 0x90; // turn on and set 1Hz update
    i2c_write(_fd, 0x20, ucTemp, 1);

    rc = i2c_read(_fd, 0x28 + 0x80, ucTemp, 5);
    if (rc == 5) {        
        // std::printf("ucTemp[0]: %d\tucTemp[1]: %d\tucTemp[2]: %d \tucTemp[3]: %d\tucTemp[4]: %d\n\r", ucTemp[0], ucTemp[1], ucTemp[2], ucTemp[3], ucTemp[4]);
        P = ucTemp[0] + (ucTemp[1] << 8) + (ucTemp[2] << 16);
        *pressure = P / 4096; //hPa
        T = ucTemp[3] + (ucTemp[4] << 8);
        if (T > 32767)
            T -= 65536; // two's complement
        T = 425 + (T / 48); // 42.5 + T value/480
        *temp = T;
        return 1;
    } else
        return 0;
}
