#ifndef QUATERNION_H_
#define QUATERNION_H_

void Quaternion_Multiply(float* q, float* r, float* result);
void Quaternion_ToEuler(float* quat, float* pitch, float* roll, float* yaw);
void Quaternion_Inverse(float* quat, float* result);

#endif