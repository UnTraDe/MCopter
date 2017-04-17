#ifndef PID_H_
#define PID_H_

typedef struct
{
	float kp;
	float ki;
	float kd;
	float i_limit;
	float error_sum;
	float last_error;
	float error;
	float error_rate;
} PID;

float PID_Compute(PID* pid, float target, float value, float dt_ms);

#endif