#include "Quaternion.h"
#include <math.h>

void Quaternion_Multiply(float* q, float* r, float* result)
{
	result[0] = (r[0] * q[0]) - (r[1] * q[1]) - (r[2] * q[2]) - (r[3] * q[3]);
	result[1] = (r[0] * q[1]) + (r[1] * q[0]) - (r[2] * q[3]) + (r[3] * q[2]);
	result[2] = (r[0] * q[2]) + (r[1] * q[3]) + (r[2] * q[0]) - (r[3] * q[1]);
	result[3] = (r[0] * q[3]) - (r[1] * q[2]) + (r[2] * q[1]) + (r[3] * q[0]);
}

void Quaternion_ToEuler(float* quat, float* pitch, float* roll, float* yaw)
{
	float a12 = 2.0f * (quat[1] * quat[2] + quat[0] * quat[3]);
	float a22 = quat[0] * quat[0] + quat[1] * quat[1] - quat[2] * quat[2] - quat[3] * quat[3];
	float a31 = 2.0f * (quat[0] * quat[1] + quat[2] * quat[3]);
	float a32 = 2.0f * (quat[1] * quat[3] - quat[0] * quat[2]);
	float a33 = quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3];
	*pitch = -atan2f(a31, a33);
	*roll = -asinf(a32);
	*yaw = atan2f(a12, a22);
}

void Quaternion_Inverse(float* quat, float* result)
{
	result[0] = quat[0];
	result[1] = -quat[1];
	result[2] = -quat[2];
	result[3] = -quat[3];
}