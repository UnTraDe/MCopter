#include "PID.h"

float PID_Compute(PID* pid, float target, float value, float dt_ms)
{
	pid->error = (target - value);
	pid->error_sum += pid->error * dt_ms;
	pid->error_rate = (pid->error - pid->last_error) / dt_ms;
	pid->last_error = pid->error;

	float err_i = pid->error_sum * pid->ki;

	if (err_i > pid->i_limit)
		err_i = pid->i_limit;
	else if (err_i < -pid->i_limit)
		err_i = -pid->i_limit;

	return pid->error * pid->kp + err_i + pid->error_rate * pid->kd;
}