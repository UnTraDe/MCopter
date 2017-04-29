#include "ICM20689.h"
#include <stdint.h>

#define TIMEOUT 100

#define REG_SMPLRT_DIV		0x19
#define REG_CONFIG			0x1A
#define REG_GYRO_CONFIG		0x1B
#define REG_ACCEL_CONFIG	0x1C
#define REG_ACCEL_CONFIG2	0x1D

#define REG_INT_PIN_CFG		0x37
#define REG_INT_ENABLE		0x38
#define REG_INT_STATUS		0x3A

#define REG_GYRO_XOUT_H		0x43
#define REG_GYRO_XOUT_L		0x44
#define REG_GYRO_YOUT_H		0x45
#define REG_GYRO_YOUT_L		0x46
#define REG_GYRO_ZOUT_H		0x47
#define REG_GYRO_ZOUT_L		0x48

#define REG_ACCEL_XOUT_H	0x3B
#define REG_ACCEL_XOUT_L	0x3C
#define REG_ACCEL_YOUT_H	0x3D
#define REG_ACCEL_YOUT_L	0x3E
#define REG_ACCEL_ZOUT_H	0x3F
#define REG_ACCEL_ZOUT_L	0x40

#define REG_USER_CTRL		0x6A
#define REG_PWR_MGMT_1		0x6B
#define REG_PWR_MGMT_2		0x6C
#define REG_WHO_AM_I		0x75

#define REG_FIFO_EN         0x23
#define REG_I2C_MST_CTRL    0x24
#define REG_FIFO_COUNTH     0x72
#define REG_FIFO_R_W        0x74
#define REG_XG_OFFSET_H     0x13
#define REG_XG_OFFSET_L     0x14
#define REG_YG_OFFSET_H     0x15
#define REG_YG_OFFSET_L     0x16
#define REG_ZG_OFFSET_H     0x17
#define REG_ZG_OFFSET_L     0x18
#define REG_XA_OFFSET_H     0x77
#define REG_XA_OFFSET_L     0x78
#define REG_YA_OFFSET_H     0x7A
#define REG_YA_OFFSET_L     0x7B
#define REG_ZA_OFFSET_H     0x7D
#define REG_ZA_OFFSET_L     0x7E

#define ICM20689_ID		0x98

static void WriteRegister(uint8_t reg, uint8_t data);
static uint8_t ReadRegister(uint8_t reg);
static void ReadMultipleRegister(uint8_t reg, uint8_t* data, uint16_t count);

static SPI_HandleTypeDef* _hspi;
static GPIO_TypeDef* _slave_select_port;
static uint16_t _slave_select_pin;

float GyroScale[4] =
{
	(250.0f / 32768.0f),
	(500.0f / 32768.0f),
	(1000.0f / 32768.0f),
	(2000.0f / 32768.0f)
};

float AccelScale[4] =
{
	(2.0f / 32768.0f),
	(4.0f / 32768.0f),
	(8.0f / 32768.0f),
	(16.0f / 32768.0f)
};

static GyroFullScaleRange _gyro_range = GFS_2000DPS;
static AccelFullScaleRange _accel_range = AFS_16G;

uint8_t ICM20689_Init(SPI_HandleTypeDef* hspi, GPIO_TypeDef* slave_select_port, uint16_t slave_select_pin)
{
	_hspi = hspi;
	_slave_select_port = slave_select_port;
	_slave_select_pin = slave_select_pin;
	
	HAL_GPIO_WritePin(_slave_select_port, _slave_select_pin, GPIO_PIN_SET); // idle high
	
	HAL_Delay(100); // Start-up time for register read/write from power up: typ - 11ms, max 100ms
	
	// Initializing ICM20689
	
	WriteRegister(REG_PWR_MGMT_1, 0b10000000); // reset
	HAL_Delay(10);
	
	WriteRegister(REG_USER_CTRL, 0b00010000); // Disable I2C Slave module and put the serial interface in SPI mode only.
	HAL_Delay(10);
	
	WriteRegister(REG_PWR_MGMT_1, 0b00000000); // wake up
	HAL_Delay(10);
	
	WriteRegister(REG_PWR_MGMT_1, 0b00000001); // Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
	HAL_Delay(10);
	
	// Making sure it's the correct model
	
	if (ReadRegister(REG_WHO_AM_I) != ICM20689_ID)
		return 1;
	
	HAL_Delay(10);
	
	// Configuring gyroscope
	
	WriteRegister(REG_CONFIG, 0x03); // DLPF_CFG = 3, gyro filter = 41/59.0, gyro rate = 1KHz, temp filter = 42
	HAL_Delay(10);
	
	uint8_t gyro_cfg = 0;
	gyro_cfg = (gyro_cfg & 0xE7) | (_gyro_range << 3);
	WriteRegister(REG_GYRO_CONFIG, gyro_cfg); // gyro full scale = ±2000dps, FCHOICE_B = 00
	HAL_Delay(10);
	
	// Configuring accelerometer
	
	uint8_t accel_cfg = 0;
	accel_cfg = (accel_cfg & 0xE7) | (_accel_range << 3);
	WriteRegister(REG_ACCEL_CONFIG, accel_cfg); // accel full scale = ±16g
	HAL_Delay(10);
	
	WriteRegister(REG_ACCEL_CONFIG2, 0x03); // ACCEL_FCHOICE_B = 0, A_DLPF_CFG = 3 filter=44.8/61.5 rate=1KHz
	HAL_Delay(10);
	
	// Sample rate divider (effective only if FCHOICE_B is 0b00 and 0 < DLPF_CFG < 7)
	
	WriteRegister(REG_SMPLRT_DIV, 0); // SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV) Where INTERNAL_SAMPLE_RATE = 1kHz
	HAL_Delay(10);
	
	// Enable interrupt
	
	// The logic level for INT/DRDY pin is active high.
	// INT/DRDY pin is configured as push-pull.
	// INT/DRDY pin indicates interrupt pulse’s width is 50us.
	// Interrupt status is cleared only by reading INT_STATUS register
	WriteRegister(REG_INT_PIN_CFG, 0x00);
	HAL_Delay(10);
	
	WriteRegister(REG_INT_ENABLE, 0x01); // Data ready interrupt enable
	HAL_Delay(10);
	
	return 0;
}

void ICM20689_ReadGyro(float* gyro)
{
	uint8_t data[6];
	ReadMultipleRegister(REG_GYRO_XOUT_H, data, 6);

	gyro[0] = (int16_t)((data[0] << 8) | data[1]) * GyroScale[_gyro_range];
	gyro[1] = (int16_t)((data[2] << 8) | data[3]) * GyroScale[_gyro_range];
	gyro[2] = (int16_t)((data[4] << 8) | data[5]) * GyroScale[_gyro_range];
}

void ICM20689_ReadGyroRaw(int16_t* gyro)
{
	uint8_t data[6];
	ReadMultipleRegister(REG_GYRO_XOUT_H, data, 6);

	gyro[0] = (int16_t)((data[0] << 8) | data[1]);
	gyro[1] = (int16_t)((data[2] << 8) | data[3]);
	gyro[2] = (int16_t)((data[4] << 8) | data[5]);
}

void ICM20689_ReadAccel(float* accel)
{
	uint8_t data[6];
	ReadMultipleRegister(REG_ACCEL_XOUT_H, data, 6);

	accel[0] = (int16_t)((data[0] << 8) | data[1]) * AccelScale[_accel_range];
	accel[1] = (int16_t)((data[2] << 8) | data[3]) * AccelScale[_accel_range];
	accel[2] = (int16_t)((data[4] << 8) | data[5]) * AccelScale[_accel_range];
}

void ICM20689_ReadAccelRaw(int16_t* accel)
{
	uint8_t data[6];
	ReadMultipleRegister(REG_ACCEL_XOUT_H, data, 6);

	accel[0] = (int16_t)((data[0] << 8) | data[1]);
	accel[1] = (int16_t)((data[2] << 8) | data[3]);
	accel[2] = (int16_t)((data[4] << 8) | data[5]);
}

uint8_t ICM20689_DataReady()
{
	return ReadRegister(REG_INT_STATUS) & 0x01; // returns DATA_RDY_INT bit
}

void ICM20689_SetGyroFullScaleRange(GyroFullScaleRange fsr)
{	
	uint8_t gyro_cfg = ReadRegister(REG_GYRO_CONFIG);
	gyro_cfg = (gyro_cfg & 0xE7) | (fsr << 3);
	WriteRegister(REG_GYRO_CONFIG, gyro_cfg);
	_gyro_range = fsr;
}

void ICM20689_SetAccelFullScaleRange(AccelFullScaleRange fsr)
{	
	uint8_t accel_cfg = ReadRegister(REG_ACCEL_CONFIG);
	accel_cfg = (accel_cfg & 0xE7) | (fsr << 3);
	WriteRegister(REG_ACCEL_CONFIG, accel_cfg);
	_accel_range = fsr;
}

void WriteRegister(uint8_t reg, uint8_t data)
{
	uint8_t packet[2] = { reg & 0x7F, data };
	HAL_GPIO_WritePin(_slave_select_port, _slave_select_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(_hspi, packet, 2, TIMEOUT);
	HAL_GPIO_WritePin(_slave_select_port, _slave_select_pin, GPIO_PIN_SET);
	
	// TODO check status
}

uint8_t ReadRegister(uint8_t reg)
{
	uint8_t packet[2] = { reg | 0x80, 0x00 };
	uint8_t data[2] = { 0 };
	
	HAL_GPIO_WritePin(_slave_select_port, _slave_select_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(_hspi, packet, data, 2, TIMEOUT);
	HAL_GPIO_WritePin(_slave_select_port, _slave_select_pin, GPIO_PIN_SET);
	
	// TODO check status
	
	return data[1];
}

void ReadMultipleRegister(uint8_t reg, uint8_t* data, uint16_t count)
{
	uint8_t packet = reg | 0x80;
	
	HAL_GPIO_WritePin(_slave_select_port, _slave_select_pin, GPIO_PIN_RESET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(_hspi, &packet, 1, TIMEOUT);
	status = HAL_SPI_Receive(_hspi, data, count, TIMEOUT);
	HAL_GPIO_WritePin(_slave_select_port, _slave_select_pin, GPIO_PIN_SET);	
	
	// TODO check status
}