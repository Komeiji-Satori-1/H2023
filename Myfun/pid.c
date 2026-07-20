#include "pid.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max,
              float deadband)
{
  if(pid == ((void*)0)) return;
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
  pid->k_deadband = deadband;
  pid->deadband_zero_output = 0;
  pid->err[1] = pid->err[0] = 0.0f;
  pid->i_out = 0.0f;
  pid->p_out = 0.0f;
  pid->d_out = 0.0f;
}

void pid_reset(pid_struct_t * pid)
{
	pid->err[1] = pid->err[0] = 0.0f;
	pid->i_out = 0.0f;
	pid->p_out = 0.0f;
	pid->d_out = 0.0f;
}

float pid_calc(pid_struct_t *pid, float ref, float cur)
{
  float output;
  pid->err[1] = pid->err[0];
  pid->err[0] = ref - cur;

  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

  output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(output, -pid->out_max, pid->out_max);
  return output;
}

float pid_calc_deadband(pid_struct_t *pid, float ref, float cur)
{
  float output;
  pid->err[1] = pid->err[0];

  pid->err[0] = ref - cur;
  if(pid->err[0] > pid->k_deadband) {
    pid->err[0] -= pid->k_deadband;
  }
  else if(pid->err[0] < -pid->k_deadband) {
    pid->err[0] += pid->k_deadband;
  }else{
    if(ref < pid->k_deadband && ref > -pid->k_deadband
		&& (pid->err[1] < -pid->k_deadband || pid->err[1] > pid->k_deadband)){
			pid->i_out = 0.0f;
	}
	if(pid->deadband_zero_output) return 0.0f;
  }

  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);

  output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(output, -pid->out_max, pid->out_max);
  return output;
}
