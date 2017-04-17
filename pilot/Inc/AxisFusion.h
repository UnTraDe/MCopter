#ifndef AXIS_FUSION_H_
#define AXIS_FUSION_H_

void AxisFusion_MahonyAHRSupdateIMU(float* q, float gx, float gy, float gz, float ax, float ay, float az);
void AxisFusion_MahonyAHRSupdate(float* q, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

#endif