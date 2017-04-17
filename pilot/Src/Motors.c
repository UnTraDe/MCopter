#include "Motors.h"

static TIM_HandleTypeDef* _htim;

void Motors_Init(TIM_HandleTypeDef* htim)
{
	_htim = htim;
	
	TIM_OC_InitTypeDef sConfigOC;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 6250;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
}