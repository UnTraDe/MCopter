#include "Motors.h"

static TIM_HandleTypeDef* _htim;
static TIM_OC_InitTypeDef _template_config;

#define PERIOD_MIN			50000
#define PERIOD_MAX			100000
#define PERIOD_RESOLUTION	(PERIOD_MAX - PERIOD_MIN)
#define MAX_VALUE			1000

void Motors_Init(TIM_HandleTypeDef* htim)
{
	_htim = htim;
	
	_template_config.OCMode = TIM_OCMODE_PWM1;
	_template_config.Pulse = PERIOD_MIN;
	_template_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	_template_config.OCFastMode = TIM_OCFAST_DISABLE;
	
	HAL_TIM_PWM_ConfigChannel(_htim, &_template_config, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(_htim, &_template_config, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(_htim, &_template_config, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(_htim, &_template_config, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(_htim, TIM_CHANNEL_4);
}

void Motors_Set(float* values)
{
	for (int i = 0; i < 4; i++)
	{
		uint32_t channel;
		
		switch (i)
		{
		case 0:
			channel = TIM_CHANNEL_1;
			break;
		case 1:
			channel = TIM_CHANNEL_2;
			break;
		case 2:
			channel = TIM_CHANNEL_3;
			break;
		case 3:
			channel = TIM_CHANNEL_4;
			break;
		}
		
		float value = values[i];
		
		if (value > MAX_VALUE)
			value = MAX_VALUE;
		
		_template_config.Pulse = PERIOD_MIN + ((PERIOD_RESOLUTION / MAX_VALUE) * value);
		HAL_TIM_PWM_ConfigChannel(_htim, &_template_config, channel);
		HAL_TIM_PWM_Start(_htim, channel);
	}
}