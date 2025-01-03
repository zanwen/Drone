#include "Com_PID.h"

void Com_PID_Control(PidUnit_t *pid, float dt) {
    float err = pid->measured - pid->desired;
    float derivativeErr = (err - pid->prevErr) / dt; // 误差变化率
    pid->integralErr += err * dt;                    // 误差积分
    pid->out = pid->kp * err + pid->ki * pid->integralErr + pid->kd * derivativeErr;
    pid->prevErr = err;
}

void Com_PID_CascadeControl(PidUnit_t *main, PidUnit_t *secondary, float dt) {
    Com_PID_Control(secondary, dt);
    main->desired = secondary->out;
    Com_PID_Control(main, dt);
}

void Com_PID_Reset(PidUnit_t *pids[], uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        pids[i]->desired = pids[i]->integralErr = pids[i]->prevErr = pids[i]->out = 0.0f;
    }
}
