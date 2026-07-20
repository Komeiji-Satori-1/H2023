#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integrator;
    float last_error;
    float out_min;
    float out_max;
} PidController_t;

void Pid_Init(PidController_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max);
void Pid_Reset(PidController_t *pid);
float Pid_Calc(PidController_t *pid, float error);

#endif /* __PID_H__ */
