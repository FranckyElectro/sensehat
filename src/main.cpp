#include "SenseHat.hpp"
#include <unistd.h>
#include <gnu/libc-version.h>

void interruptFunction(uint8_t value) {
    printf("joystick has changed, new value is %d\n", value);
    fflush(stdout);
}

int main() {
    printf("sizeof int is: %d bytes\n", sizeof(int));
    uint8_t button_state;
    Sensehat sense_hat;
    sense_hat.init(1);
    printf("glibc version is: %s\n", gnu_get_libc_version());
    /*  while (1) {
         button_state = sense_hat.read_joystick();
         printf("Button value: %d\n", button_state);
         sleep(1);
     } */
    /* for (;;) {
        button_state = sense_hat.wait_for_joystick_change();
        printf("Button value: %d\n", button_state);
    } */
    sense_hat.add_joystick_event(interruptFunction);
    for (;;) {
        printf(".");
        fflush(stdout);
        Sensehat::delay_ms(20);
    }
    return 0;
}