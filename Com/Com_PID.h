#ifndef __COM_PID_H__
#define __COM_PID_H__

#include "stm32f1xx_hal.h"

typedef volatile struct {
    float desired;   // 期望值
    float prevErr; // 上次偏差
    float integralErr;     // 误差积分累加值
    float kp;        // p参数
    float ki;        // i参数
    float kd;        // d参数
    float measured;  // 实际测量值
    float out;       // pid输出
} PidUnit_t;

void Com_PID_Control(PidUnit_t *pid, float dt);

void Com_PID_CascadeControl(PidUnit_t *main, PidUnit_t *secondary, float dt);

void Com_PID_Reset(PidUnit_t *pids[], uint8_t size);

#endif /* __COM_PID_H__ */
