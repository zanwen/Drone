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

typedef struct {
    float pitch;
    float roll;
    float yaw;
} FlightAngle;

typedef struct {
    int16_t THR;
    int16_t YAW;
    int16_t ROL;
    int16_t PIT;
    int16_t AUX1;
    int16_t AUX2;
    int16_t AUX3;
    int16_t AUX4;
    int16_t AUX5;
    int16_t AUX6;
} RemoteControlData_t;

typedef struct { 
    uint16_t header;
    uint8_t function;
    uint8_t dataLen;
    RemoteControlData_t data;
    uint32_t checkSum;
} RemoteControlPacket_t;



extern uint16_t batteryAdcValue;
extern PilotLEDState ledState;
extern MPU6050Data mpu6050;
extern FlightAngle flightAngle;



void App_Flight_DetectBatVoltage(void);

void App_Flight_MPUCalibrate(void);

void App_Flight_FetchMPUData(void);

void App_Flight_PilotLED(void);

#endif /* __APP_FLIGHT_H__ */
