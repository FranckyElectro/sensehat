#include "SenseHat.hpp"
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <pthread.h>

uint8_t Sensehat::LEDArray[192] = {
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
    63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0, 63, 0, 0,
};

Sensehat::Sensehat() {
    check_i2c_bus();
    file_acc = -1;
    file_hum = -1;
    file_led = -1;
    file_mag = -1;
    file_pres = -1;
}

Sensehat::~Sensehat() {
    close(file_led);
}

int Sensehat::delay_ms(int ms) {
    struct timespec delay = {0, ms * 1000 * 1000};
    return nanosleep(&delay, NULL);
}

/// @brief
/// @param hat_addr number of the BSC (Broadcom Serial Controller) - in general hat_addr=1 will be useds
/// @return status giving the state of the different sensehate's i2c devices
int8_t Sensehat::init(uint8_t hat_addr) {
    char file_name[32];
    uint8_t status = 0;
    uint8_t buf[1] = {0};
    sprintf(file_name, "/dev/i2c-%d", hat_addr);
    if ((file_led = open(file_name, O_RDWR)) < 0) {
        // perror("Failure to open i2c bus!");
        return -1;
    }
    ioctl(file_led, I2C_SLAVE, 0x46);
    if (write(file_led, buf, 1) == -1) {
        perror("Failed to detect leds matrix");
        file_led = -1;
    } else
        status |= 1 << SENSE_HAT_DISP;

    file_acc = open(file_name, O_RDWR);
    ioctl(file_acc, I2C_SLAVE, 0x6a);
    if (write(file_acc, buf, 1) == -1) {
        perror("Failed to detect accelerometer");
        file_acc = -1;
    } else
        status |= 1 << SENSE_HAT_ACC;

    file_mag = open(file_name, O_RDWR);
    ioctl(file_mag, I2C_SLAVE, 0x1c);
    if (write(file_mag, buf, 1) == -1) {
        perror("Failed to detect magnetometer");
        file_mag = -1;
    } else
        status |= 1 << SENSE_HAT_MAG;

    file_hum = open(file_name, O_RDWR);
    ioctl(file_hum, I2C_SLAVE, 0x5f);
    if (write(file_hum, buf, 1) == -1) {
        perror("Failed to detect humidity");
        file_hum = -1;
    } else
        status |= 1 << SENSE_HAT_HUM;

    file_pres = open(file_name, O_RDWR);
    ioctl(file_pres, I2C_SLAVE, 0x5c);
    if (write(file_pres, buf, 1) == -1) {
        perror("Failed to detect pressure");
        file_pres = -1;
    } else
        status |= 1 << SENSE_HAT_PRES;

    if (status & (1 << SENSE_HAT_DISP)) {
        i2c_write(file_led, 0, LEDArray, sizeof(LEDArray));
        sleep(1);
        // Fill the LED with black
        memset(LEDArray, 0, 192);
        i2c_write(file_led, 0, LEDArray, sizeof(LEDArray));
    }
    printf("Status: %d\n", status);
    Hts221 hts221(file_hum);
    Lps25h lps25h(file_pres);
    hts221.init();
    return status;
    /* float temp = 0;
    float hum = 0;
    float pressure = 0;
    while (hts221.getTemp(&temp, &hum))
    {
        printf("Temperature: %.2f°C - humidity: %.2f%\n\r", temp, hum);
        delay_ms(500);
    }
    while (1)
    {
        temp = 0;
        hts221.getTemp(&temp);
        printf("Another calculation for temperature: %.2f°C\n\r", temp);
        delay_ms(500);
    }
    while (lps25h.getTempAndPressure(&temp, &hum))
    {
        printf("Temperature: %.2f°C - pressure: %.2f hPa\n\r", temp / 10, hum);
        delay_ms(500);
    } */
}

/* uint8_t Sensehat::i2c_write(int fd, uint8_t cmd, uint8_t* buf, uint16_t length) {
    uint8_t ucTemp[512];
    uint8_t rc;

    if (length > 511 || length < 1 || buf == NULL)
        return -1; // invalid write
    ucTemp[0] = cmd; // send the register number first
    memcpy(&ucTemp[1], buf, length); // followed by the data
    rc = write(fd, ucTemp, length + 1);
    return rc - 1;
} */

/* uint8_t Sensehat::i2c_read(int fd, uint8_t cmd, uint8_t* buf, uint8_t length) {
    uint8_t rc;
    rc = write(fd, &cmd, 1);
    if (rc == 1) {
        rc = read(fd, buf, length);
    }
    return rc;
} */

void Sensehat::check_i2c_bus() {
    FILE* fd = do_command("sudo dtparam -l");
    char output[1024];
    int txfound = 0;
    while (fgets(output, sizeof(output), fd) != NULL) {
        printf("%s", output);
        fflush(stdout);
        if (strstr(output, "i2c_arm=on") != NULL)
            txfound = 1;
        if (strstr(output, "i2c_arm=off") != NULL)
            txfound = 0;
    }
    pclose(fd);
    if (txfound == 0) {
        fd = do_command("sudo dtparam i2c_arm=on");
        printf("i2c_arm set to on\n\r");
        pclose(fd);
    }
}

FILE* Sensehat::do_command(const char* cmd) {
    FILE* fp = popen(cmd, "r");
    if (fp == NULL) {
        printf("Failed to run command %s\n\r", cmd);
        exit(1);
    }
    return fp;
}

void* poll_joystick_change(void* arg) {
    intVec* intData = (intVec*)arg;
    Sensehat* caller = (Sensehat*)intData->caller;
    for (;;) {
        uint8_t value = caller->wait_for_joystick_change();
        intData->func(value);
    }
    return arg;
}

/// @brief Read the i2c µC to get the state of the joystick.
/// @return state of the joystick i.e.: idle -> 0 - down -> 1 - rigth -> 2 - up -> 4 - pushed -> 8 - left -> 16
uint8_t Sensehat::read_joystick() {
    uint8_t ucBuf[1];
    int rc;
    if (file_led != -1) {
        rc = i2c_read(file_led, 0xf2, ucBuf, 1);
        if (rc == 1)
            return ucBuf[0];
    }
    return 0;
}

/// @brief blocking function waiting for a joystick change
/// @return the value associated to the joystick position
uint8_t Sensehat::wait_for_joystick_change() {
    static uint8_t last_value = 0;
    for (;;) {
        uint8_t current_value = read_joystick();
        if (last_value != current_value) {
            last_value = current_value;
            return current_value;
        }
        delay_ms(10);
    }
}

/// @brief
/// @param func
/// @return
int Sensehat::add_joystick_event(eventHandler func) {
    pthread_t intThread;
    interruptVector.caller = this;
    interruptVector.func = func;
    if (pthread_create(&intThread, NULL, poll_joystick_change, &interruptVector)) {
        perror("Error creating thread");
        return 1;
    }
    return 0;
}