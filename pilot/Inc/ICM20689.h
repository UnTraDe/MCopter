#ifndef ICM20689_H_
#define ICM20689_H_

#include "stm32f4xx_hal.h"

uint8_t ICM20689_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* slave_select_port, uint16_t slave_select_pin);
void ICM20689_ReadGyro(float* gyro);
void ICM20689_ReadAccel(float* accel);
uint8_t ICM20689_DataReady();

#endif