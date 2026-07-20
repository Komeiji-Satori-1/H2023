#ifndef _PID_H
#define _PID_H

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  float k_deadband;

  int deadband_zero_output;
  float err[2];

  float p_out;
  float i_out;
  float d_out;
} pid_struct_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max,
              float deadband);
void pid_reset(pid_struct_t * pid);
float pid_calc(pid_struct_t *pid, float ref, float cur);
float pid_calc_deadband(pid_struct_t *pid, float ref, float cur);

#endif
