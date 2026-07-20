#include "pid.h"

static float Pid_Limit(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

void Pid_Init(PidController_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

void Pid_Reset(PidController_t *pid)
{
    if (pid == 0)
    {
        return;
    }

    pid->integrator = 0.0f;
    pid->last_error = 0.0f;
}

float Pid_Calc(PidController_t *pid, float error)
{
    float derivative;
    float output;

    if (pid == 0)
    {
        return 0.0f;
    }

    pid->integrator += error;
    derivative = error - pid->last_error;
    output = (pid->kp * error) +
             (pid->ki * pid->integrator) +
             (pid->kd * derivative);

    output = Pid_Limit(output, pid->out_min, pid->out_max);

    if ((output >= pid->out_max) || (output <= pid->out_min))
    {
        pid->integrator -= error;
    }

    pid->last_error = error;

    return output;
}
