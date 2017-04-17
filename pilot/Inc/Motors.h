#ifndef MOTORS_H_
#define MOTORS_H_

#include "stm32f4xx_hal.h"

void Motors_Init(TIM_HandleTypeDef* htim);
void Motors_Set(uint16_t* values);

#endif