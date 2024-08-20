#ifndef _SENSEHATDEFS_HPP
#define _SENSEHATDEFS_HPP


#define SENSE_HAT_MAG 0
#define SENSE_HAT_DISP 1
#define SENSE_HAT_PRES 2
#define SENSE_HAT_HUM 3
#define SENSE_HAT_ACC 4


typedef void (*eventHandler)(uint8_t);
typedef struct {
    void* caller;
    eventHandler func;
} intVec;

#endif