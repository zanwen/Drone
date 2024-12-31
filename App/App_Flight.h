#ifndef __APP_FLIGHT_H__
#define __APP_FLIGHT_H__

#include "stm32f1xx_hal.h"

typedef struct {
    uint16_t durationTime;
    enum {
        AlwaysOn,
        AlwaysOff,
        AllFlashLight,
        AlternateFlash,
        WARNING,
        DANGEROURS,
        GET_OFFSET
    } state;
} PilotLEDState;

typedef struct {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
} MPU6050Data;

extern PilotLEDState ledState;

void App_Flight_DetectBatVoltage(void);

void App_Flight_MPUCalibrate(void);

void App_Flight_FetchMPUData(void);

void App_Flight_PilotLED(void);

#endif /* __APP_FLIGHT_H__ */
