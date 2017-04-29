#include "Motors.h"

static TIM_HandleTypeDef* _htim;

#define PERIOD_MIN			50000
#define PERIOD_MAX			100000
#define PERIOD_RESOLUTION	(PERIOD_MAX - PERIOD_MIN)
#define MAX_VALUE			1000

// TODO currently using the PWM mode there's a delay from calling start to when the actual signal starts, depends on  ARR - CCRx

void Motors_Init(TIM_HandleTypeDef* htim)
{
	_htim = htim;
	
	_htim->Instance->CCR1 = PERIOD_MIN;
	_htim->Instance->CCR2 = PERIOD_MIN;
	_htim->Instance->CCR3 = PERIOD_MIN;
	_htim->Instance->CCR4 = PERIOD_MIN;
	
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4);
}

void Motors_Set(float* values)
{
	for (int i = 0; i < 4; i++)
	{
		float value = values[i];
		
		if (value > MAX_VALUE)
			value = MAX_VALUE;
		
		int output = PERIOD_MIN + ((PERIOD_RESOLUTION / MAX_VALUE) * value);
		
		switch (i)
		{
		case 0:
			_htim->Instance->CCR1 = output;
			break;
		case 1:
			_htim->Instance->CCR2 = output;
			break;
		case 2:
			_htim->Instance->CCR3 = output;
			break;
		case 3:
			_htim->Instance->CCR4 = output;
			break;
		}
	}
}