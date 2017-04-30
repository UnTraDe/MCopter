#ifndef ICM20689_H_
#define ICM20689_H_

#include "stm32f4xx_hal.h"

typedef enum
{
	GFS_250DPS = 0,
	GFS_500DPS = 1,
	GFS_1000DPS = 2,
	GFS_2000DPS = 3
} GyroFullScaleRange;

typedef enum
{
	AFS_2G = 0,
	AFS_4G = 1,
	AFS_8G = 2,
	AFS_16G = 3
} AccelFullScaleRange;

extern float GyroScale[4];
extern float AccelScale[4];

uint8_t ICM20689_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* slave_select_port, uint16_t slave_select_pin);
void ICM20689_ReadGyro(float* gyro);
void ICM20689_ReadGyroRaw(int16_t* gyro);
void ICM20689_ReadAccel(float* accel);
void ICM20689_ReadAccelRaw(int16_t* accel);
uint8_t ICM20689_DataReady();
void ICM20689_SetGyroFullScaleRange(GyroFullScaleRange fsr);
void ICM20689_SetAccelFullScaleRange(AccelFullScaleRange fsr);
void ICM20689_SetLocalGyroBias(float* bias);
void ICM20689_SetLocalAccelBias(float* bias);

#endif