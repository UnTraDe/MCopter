/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <string.h>

#include "ICM20689.h"
#include "MS5637.h"
#include "AxisFusion.h"
#include "Common.h"
#include "Quaternion.h"
#include "Motors.h"
#include "PID.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static const float Kp = 3.0f;
static const float Ki = 0.0f; //0.08f;
static const float Kd = 10.0f;
static const float i_limit = 40.0f;

static const float Kp_yaw = 8.0f;
static const float Ki_yaw = 0.0f; //0.4f;
static const float Kd_yaw = 0.0f;
static const float i_limit_yaw = 40.0f;

static const float _control_angle = 30.0f;
static const float _yaw_max_rate = 60.0f;

static volatile uint8_t _imu_data_ready = 0;
static volatile uint16_t _missed_imu_data = 0;

static float _orientation[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; // Quaternion
static float _reference_orientation[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

static uint16_t _throttle = 0;
static float _target_yaw_rate = 0.0f;
static float _target_pitch = 0.0f;
static float _target_roll = 0.0f;

static PID _pid_pitch;
static PID _pid_roll;
static PID _pid_yaw;

static volatile uint8_t _receiver_data_ready = 0;
static volatile struct
{
	uint16_t channels[16];
	uint8_t fail_safe;
} _receiver_data;

#define BATTERY_VOLTAGE_PERIOD		1000
#define BATTERY_LOW_THRESHOLD		10.8f
#define BATTERY_VOLTAGE_MULTIPLIER	3.794f
#define THROTTLE_THRESHOLD			150

#define SBUS_CHANNEL_MIN 				172
#define SBUS_CHANNEL_MAX 				1811
#define SBUS_CHANENL_RANGE				(SBUS_CHANNEL_MAX - SBUS_CHANNEL_MIN)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_4)
	{
		// TODO need to disable interrupts before doing SPI stuff?
		
		if (!_imu_data_ready)
		{
			if (ICM20689_DataReady())
				_imu_data_ready = 1;
		}
		else
		{
			_missed_imu_data++;
		}
		
		// TODO handle cases where the interrupt was generated because of another reason... 
	}	
}

static uint8_t _uart_rx_data_buffer[32];
static uint8_t _uart_tx_data_buffer[32];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if (!_receiver_data_ready)
	{
		if (_uart_rx_data_buffer[0] == 0x0F && _uart_rx_data_buffer[24] == 0x00)
		{
			uint8_t* payload = &_uart_rx_data_buffer[1];
		
			_receiver_data.channels[0]  = (uint16_t)((payload[0]    | payload[1] << 8) & 0x07FF);
			_receiver_data.channels[1]  = (uint16_t)((payload[1] >> 3 | payload[2] << 5) & 0x07FF);
			_receiver_data.channels[2]  = (uint16_t)((payload[2] >> 6 | payload[3] << 2 | payload[4] << 10) & 0x07FF);
			_receiver_data.channels[3]  = (uint16_t)((payload[4] >> 1 | payload[5] << 7) & 0x07FF);
			_receiver_data.channels[4]  = (uint16_t)((payload[5] >> 4 | payload[6] << 4) & 0x07FF);
			_receiver_data.channels[5]  = (uint16_t)((payload[6] >> 7 | payload[7] << 1 | payload[8] << 9) & 0x07FF);
			_receiver_data.channels[6]  = (uint16_t)((payload[8] >> 2 | payload[9] << 6) & 0x07FF);
			_receiver_data.channels[7]  = (uint16_t)((payload[9] >> 5 | payload[10] << 3) & 0x07FF);
			_receiver_data.channels[8]  = (uint16_t)((payload[11]   | payload[12] << 8) & 0x07FF);
			_receiver_data.channels[9]  = (uint16_t)((payload[12] >> 3 | payload[13] << 5) & 0x07FF);
			_receiver_data.channels[10] = (uint16_t)((payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF);
			_receiver_data.channels[11] = (uint16_t)((payload[15] >> 1 | payload[16] << 7) & 0x07FF);
			_receiver_data.channels[12] = (uint16_t)((payload[16] >> 4 | payload[17] << 4) & 0x07FF);
			_receiver_data.channels[13] = (uint16_t)((payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF);
			_receiver_data.channels[14] = (uint16_t)((payload[19] >> 2 | payload[20] << 6) & 0x07FF);
			_receiver_data.channels[15] = (uint16_t)((payload[20] >> 5 | payload[21] << 3) & 0x07FF);
			
			_receiver_data.fail_safe = (payload[22] & 0x08);
		
			_receiver_data_ready = 1;
		}
	}
	else
	{
		// TODO increment some dropped counter
	}
	
	// TODO can this be replaced by configuring the DMA as circular?
	HAL_UART_Receive_DMA(&huart1, _uart_rx_data_buffer, 25);
}

static void CalculateBias(float* gyro_bias, float* accel_bias);

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	  /* Configure the system clock */
	SystemClock_Config();

	  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_TIM1_Init();
	MX_TIM5_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	MX_I2C2_Init();
	MX_ADC1_Init();

	  /* USER CODE BEGIN 2 */
	Motors_Init(&htim5);
	
	uint8_t error = ICM20689_Init(&hspi1, GPIOA, GPIO_PIN_4);
	
	if (error)
	{
		while (1)
		{
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
			HAL_Delay(500);
		}
	}
	
	_pid_pitch.kp = Kp;
	_pid_pitch.ki = Ki;
	_pid_pitch.kp = Kp;
	_pid_pitch.i_limit = i_limit;
	_pid_pitch.error_sum = 0.0f;
	_pid_pitch.last_error = 0.0f;
	_pid_pitch.error = 0.0f;
	_pid_pitch.error_rate = 0.0f;
	
	_pid_roll.kp = Kp;
	_pid_roll.ki = Ki;
	_pid_roll.kp = Kp;
	_pid_roll.i_limit = i_limit;
	_pid_roll.error_sum = 0.0f;
	_pid_roll.last_error = 0.0f;
	_pid_roll.error = 0.0f;
	_pid_roll.error_rate = 0.0f;
	
	_pid_yaw.kp = Kp_yaw;
	_pid_yaw.ki = Ki_yaw;
	_pid_yaw.kp = Kp_yaw;
	_pid_yaw.i_limit = i_limit_yaw;
	_pid_yaw.error_sum = 0.0f;
	_pid_yaw.last_error = 0.0f;
	_pid_yaw.error = 0.0f;
	_pid_yaw.error_rate = 0.0f;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	
	
	float gyro_bias[3] = { -1.228333, 2.250671, 0.083923 };
	float accel_bias[3] = { 0.005554, -0.005920, 0.026794 };
	//CalculateBias(gyro_bias, accel_bias);
	
//	uint8_t output[128] = { 0 };
//	sprintf((char*)output, "%f, %f, %f\r\n%f, %f, %f\r\n", gyro_bias[0], gyro_bias[1], gyro_bias[2], accel_bias[0], accel_bias[1], accel_bias[2]);
//	HAL_UART_Transmit(&huart6, output, strlen((char*)output), 100);
	
	gyro_bias[0] *= -1;
	gyro_bias[1] *= -1;
	gyro_bias[2] *= -1;
	ICM20689_SetLocalGyroBias(gyro_bias);
	
	accel_bias[0] *= -1;
	accel_bias[1] *= -1;
	accel_bias[2] *= -1;
	ICM20689_SetLocalAccelBias(accel_bias);
	
	// SBUS is sending every 4 ms, so we wait to synchronize
	
	while (1)
	{
		if (HAL_TIMEOUT == HAL_UART_Receive(&huart1, _uart_rx_data_buffer, 1, 2))
		{
			HAL_UART_Receive_DMA(&huart1, _uart_rx_data_buffer, 25);
			break;
		}
	}
	
	uint8_t fail_safe = 1;
	
	uint32_t now = HAL_GetTick();
	uint32_t last = now;
	
	uint32_t battery_voltage_timer = 0;
	uint32_t test_timer = 0;
	uint32_t hz_timer = 0;
	
	uint32_t hz_counter = 0;
	
	uint8_t cmd;
	
	// Barometer test
	
	//MS5637_Init(&hi2c2, 0x76);
	
//	cmd = 0x1E;
//	if (HAL_I2C_Master_Transmit(&hi2c2, 0b11101100, &cmd, 1, 100) != HAL_OK)
//	{
//		while (1)
//		{
//			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
//			HAL_Delay(500);
//		}
//	}
//	
//	cmd = 0xA0;
//	HAL_I2C_Master_Transmit(&hi2c2, 0b11101100, &cmd, 1, 100);
//	uint16_t data = 0;
//	HAL_I2C_Master_Receive(&hi2c2, 0b11101101, (uint8_t*)&data, 2, 100);
	
	// Magnetometer test
	
//	cmd = 0x07;
//	HAL_I2C_Master_Transmit(&hi2c2, 0x0E << 1, &cmd, 1, 100);
//	uint8_t id = 0;
//	HAL_I2C_Master_Receive(&hi2c2, (0x0E << 1) | 0x01, (uint8_t*)&id, 1, 100);
	
	float battery_voltage = 12.6f;
	float battery_voltage_idle = 12.6f;
	
	// TODO intialize those with actual values from the ADC
	
	while (1)
	{
		now = HAL_GetTick(); // TODO handle rollover
		uint32_t dt_ms = now - last;
		last = now;
		
		battery_voltage_timer += dt_ms;
		test_timer += dt_ms;
		hz_timer += dt_ms;
		
		if (_receiver_data_ready)
		{
			uint16_t throttle = ((_receiver_data.channels[0] - SBUS_CHANNEL_MIN) / (float)SBUS_CHANENL_RANGE) * 1000.0f;
			
			if (throttle > 800)
				_throttle = 800;
			else
				_throttle = throttle;
			
//			
//			if (throttle > 500)
//				throttle = 1000;
//			else
//				throttle = 0;
//			
//			float motors[4];
//			
//			motors[0] = throttle; // Back Right
//			motors[1] = throttle; // Back Left
//			motors[2] = throttle; // Front Right
//			motors[3] = throttle; // Front Left
//
//			Motors_Set(motors);
			
			_target_yaw_rate = ((_receiver_data.channels[3] - SBUS_CHANNEL_MIN) * (_yaw_max_rate / SBUS_CHANENL_RANGE)) - (_yaw_max_rate / 2.0f);
			_target_pitch = ((_receiver_data.channels[2] - SBUS_CHANNEL_MIN) * (_control_angle / SBUS_CHANENL_RANGE)) - (_control_angle / 2.0f);
			_target_roll = ((_receiver_data.channels[1] - SBUS_CHANNEL_MIN) * (_control_angle / SBUS_CHANENL_RANGE)) - (_control_angle / 2.0f);

			_target_yaw_rate *= -1;
			
			fail_safe = _receiver_data.fail_safe;
			
//			if (test_timer >= 10)
//			{
//				test_timer = 0;
//				uint8_t output[32] = { 0 };
//				sprintf((char*)output, "%f, %f, %f\r\n", _target_pitch, _target_yaw_rate, _target_roll);
//				HAL_UART_Transmit(&huart6, output, strlen((char*)output), 100);
//			}
			
			_receiver_data_ready = 0;
		}
		
		if (fail_safe)
		{
			float motors[4] = { 0 };
			Motors_Set(motors);
			//continue;
		}
		
		if (battery_voltage_timer >= BATTERY_VOLTAGE_PERIOD)
		{
			battery_voltage_timer = 0;
			
			HAL_StatusTypeDef status = HAL_ADC_Start(&hadc1);
			status = HAL_ADC_PollForConversion(&hadc1, 10);
		
			if (status != HAL_TIMEOUT)
			{
				uint32_t val = HAL_ADC_GetValue(&hadc1);
				float voltage = (3.3f / 4096.0f) * val;
				battery_voltage = voltage * BATTERY_VOLTAGE_MULTIPLIER;
				
				if (_throttle < THROTTLE_THRESHOLD)
					battery_voltage_idle = voltage * BATTERY_VOLTAGE_MULTIPLIER;
				
				// TODO averaging samples?
			}
			else
			{
				battery_voltage = 0.0f;
			}
		
			HAL_ADC_Stop(&hadc1);
		}
		
		if (battery_voltage_idle <= BATTERY_LOW_THRESHOLD)
		{
			float motors[4] = { 0 };
			Motors_Set(motors);
			
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(200);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(1000);
		}
		
		if (_imu_data_ready)
		{
			_imu_data_ready = 0;
			
			float gyro[3];
			float accel[3];
			ICM20689_ReadGyro(gyro);
			ICM20689_ReadAccel(accel);
			
//			hz_counter++;
//			
//			if (hz_timer >= 1000)
//			{
//				hz_timer = 0;
//				uint8_t output[16] = { 0 };
//				sprintf((char*)output, "%d\r\n", counter);
//				counter = 0;
//				HAL_UART_Transmit(&huart6, output, strlen((char*)output), 100);
//			}
//			
			AxisFusion_MahonyAHRSupdateIMU(_orientation, gyro[0]*PI / 180.0f, gyro[1]*PI / 180.0f, gyro[2]*PI / 180.0f, accel[0], accel[1], accel[2]);
		
			float quat[4];
			Quaternion_Multiply(_reference_orientation, _orientation, quat);
		
			float pitch, roll, yaw;
			Quaternion_ToEuler(quat, &pitch, &roll, &yaw);
		
			pitch *= 180.0f / PI;
			yaw *= 180.0f / PI;
			roll *= 180.0f / PI;
			
//			if (test_timer >= 100)
//			{
//				test_timer = 0;
//				uint8_t output[32] = { 0 };
//				sprintf((char*)output, "%f, %f, %f\r\n", pitch, yaw, roll);
//				HAL_UART_Transmit(&huart6, output, strlen((char*)output), 100);
//			}
			
			float dt_ms;
			
			float output_pitch = 0.0f;
			float output_roll = 0.0f;
			float output_yaw = 0.0f;

			if (_throttle >= THROTTLE_THRESHOLD)
			{
				output_pitch = PID_Compute(&_pid_pitch, _target_pitch, pitch, dt_ms);
				output_roll = PID_Compute(&_pid_roll, _target_roll, roll, dt_ms);
				output_yaw = PID_Compute(&_pid_yaw, _target_yaw_rate, gyro[2] * (PI / 180.0f), dt_ms);
			}
			else
			{
				_pid_pitch.error_sum = 0.0f;
				_pid_roll.error_sum = 0.0f;
				_pid_yaw.error_sum = 0.0f;
			}
	
			float motors[4];
			
			motors[0] = _throttle + output_pitch - output_roll - output_yaw; // Back Right
			motors[1] = _throttle + output_pitch + output_roll + output_yaw; // Back Left
			motors[2] = _throttle - output_pitch - output_roll + output_yaw; // Front Right
			motors[3] = _throttle - output_pitch + output_roll - output_yaw; // Front Left
			
			_throttle = 500;
//			
//			if (test_timer >= 10)
//			{
//				test_timer = 0;
//				uint8_t output[64] = { 0 };
//				sprintf((char*)output, "%f, %f, %f\n", output_pitch, output_roll, output_yaw);
//				HAL_UART_Transmit(&huart6, output, strlen((char*)output), 1000);
//			}
			
			if (test_timer >= 10)
			{
				test_timer = 0;
				uint8_t output[64] = { 0 };
				sprintf((char*)output, "%f, %f, %f, %f\n", motors[0], motors[1], motors[2], motors[3]);
				HAL_UART_Transmit(&huart6, output, strlen((char*)output), 1000);
			}

			Motors_Set(motors);
		}
	}
	
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	    /**Configure the main internal regulator output voltage 
	    */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	    /**Initializes the CPU, AHB and APB busses clocks 
	    */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	    /**Initializes the CPU, AHB and APB busses clocks 
	    */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}

	    /**Configure the Systick interrupt time 
	    */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	    /**Configure the Systick 
	    */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
	    */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
	    */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 15;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 15;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 125000;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim5);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 100000;
	huart1.Init.WordLength = UART_WORDLENGTH_9B;
	huart1.Init.StopBits = UART_STOPBITS_2;
	huart1.Init.Parity = UART_PARITY_EVEN;
	huart1.Init.Mode = UART_MODE_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	  /* DMA interrupt init */
	  /* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	  /*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PC4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

static void CalculateBias(float* gyro_bias, float* accel_bias)
{
	ICM20689_SetGyroFullScaleRange(GFS_250DPS);
	ICM20689_SetAccelFullScaleRange(AFS_2G);
	
	int samples = 0;
	int32_t avg_gyro[3] = { 0 };
	int32_t avg_accel[3] = { 0 };
	
	while (samples < 400)
	{
		if (_imu_data_ready)
		{
			_imu_data_ready = 0;
			
			int16_t gyro[3];
			int16_t accel[3];
			ICM20689_ReadGyroRaw(gyro);
			ICM20689_ReadAccelRaw(accel);
			
			for (int i = 0; i < 3; i++)
				avg_gyro[i] += gyro[i];
			
			for (int i = 0; i < 3; i++)
				avg_accel[i] += accel[i];
			
			samples++;
		}
	}
	
	for (int i = 0; i < 3; i++)
	{
		avg_gyro[i] /= samples;
		gyro_bias[i] = (float)avg_gyro[i] * GyroScale[GFS_250DPS];
	}
	
	for (int i = 0; i < 3; i++)
	{
		avg_accel[i] /= samples;
		accel_bias[i] = (float)avg_accel[i] * AccelScale[AFS_2G];
	}
	
	accel_bias[2] -= 1.0f;
	
	ICM20689_SetGyroFullScaleRange(GFS_2000DPS);
	ICM20689_SetAccelFullScaleRange(AFS_16G);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
