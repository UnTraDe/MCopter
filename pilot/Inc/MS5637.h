#ifndef MS5637_H_
#define MS5637_H_

#include "stm32f4xx_hal.h"


uint8_t MS5637_Init(I2C_HandleTypeDef* hi2c, uint8_t address);


#endif