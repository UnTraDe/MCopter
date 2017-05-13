#include "MS5637.h"

#include <string.h>

#define CMD_RESET	0x1E

static I2C_HandleTypeDef* _hi2c;
static uint8_t _address;
static uint8_t _prom[14];

static void Write(uint8_t cmd);

uint8_t MS5637_Init(I2C_HandleTypeDef* hi2c, uint8_t address)
{
	_hi2c = hi2c;
	_address = address;
	
	uint8_t cmd = CMD_RESET;
	
	// Reset device
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(_hi2c, 0b11101100, &cmd, 1, 100);
	
	// Read PROM
	
	memset(_prom, 0, 14);
	
	cmd = 0xA0;
	
	for (int i = 0; i < 14; i += 2)
	{
		cmd += 2;
		HAL_I2C_Master_Transmit(_hi2c, _address << 1, &cmd, 1, 100);
		HAL_I2C_Master_Receive(_hi2c, (_address << 1) | 0x01, &_prom[i], 2, 100);	
	}
	
	return 0;
}

void Write(uint8_t cmd)
{
	
	
	// TODO check status
}
