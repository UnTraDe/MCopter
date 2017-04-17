#include "Motors.h"

static TIM_HandleTypeDef* _htim;
static TIM_OC_InitTypeDef _template_config;

#define PERIOD_MIN		6250
#define PERIOD_MAX		12500
#define PERIOD_RANGE	(PERIOD_MAX - PERIOD_MIN)
#define MAX_VALUE		1000

void Motors_Init(TIM_HandleTypeDef* htim)
{
	_htim = htim;
	
	_template_config.OCMode = TIM_OCMODE_PWM1;
	_template_config.Pulse = 6250;
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

void Motors_Set(uint16_t* values)
{
	for (int i = 0; i < 4; i++)
	{
		if (values[i] > MAX_VALUE)
			values[i] = MAX_VALUE;
		
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
		
		_template_config.Pulse = PERIOD_MIN + ((PERIOD_RANGE / MAX_VALUE) * values[i]);
		HAL_TIM_PWM_ConfigChannel(_htim, &_template_config, channel);
		HAL_TIM_PWM_Start(_htim, channel);
	}
}