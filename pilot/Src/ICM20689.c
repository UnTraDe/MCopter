#include "ICM20689.h"
#include <stdint.h>

#define TIMEOUT 100

#define GYRO_SCALE		(2000.0f / 32768.0f)
#define ACCEL_SCALE		(16.0f / 32768.0f)

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
	
	WriteRegister(REG_GYRO_CONFIG, 0b00011000); // gyro full scale = ±2000dps, FCHOICE_B = 00
	HAL_Delay(10);
	
	// Configuring accelerometer
	
	WriteRegister(REG_ACCEL_CONFIG, 0b00011000); // accel full scale = ±16g
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

	gyro[0] = (int16_t)((data[0] << 8) | data[1]) * GYRO_SCALE;
	gyro[1] = (int16_t)((data[2] << 8) | data[3]) * GYRO_SCALE;
	gyro[2] = (int16_t)((data[4] << 8) | data[5]) * GYRO_SCALE;
}

void ICM20689_ReadAccel(float* accel)
{
	uint8_t data[6];
	ReadMultipleRegister(REG_ACCEL_XOUT_H, data, 6);

	accel[0] = (int16_t)((data[0] << 8) | data[1]) * ACCEL_SCALE;
	accel[1] = (int16_t)((data[2] << 8) | data[3]) * ACCEL_SCALE;
	accel[2] = (int16_t)((data[4] << 8) | data[5]) * ACCEL_SCALE;
}

uint8_t ICM20689_DataReady()
{
	return ReadRegister(REG_INT_STATUS) & 0x01; // returns DATA_RDY_INT bit
}

void ICM20689_CalibrateAccelAndGyro(float* gyroscope_bias, float* accelerometer_bias)
{  
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	WriteRegister(REG_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
	// else use the internal oscillator, bits 2:0 = 001
	WriteRegister(REG_PWR_MGMT_1, 0x01);  
	WriteRegister(REG_PWR_MGMT_2, 0x00);
	HAL_Delay(200);                                    

	// Configure device for bias calculation
	WriteRegister(REG_INT_ENABLE, 0x00);   // Disable all interrupts
	WriteRegister(REG_FIFO_EN, 0x00);      // Disable FIFO
	WriteRegister(REG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	WriteRegister(REG_I2C_MST_CTRL, 0x00); // Disable I2C master
	WriteRegister(REG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	WriteRegister(REG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	HAL_Delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	WriteRegister(REG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	WriteRegister(REG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	WriteRegister(REG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	WriteRegister(REG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	WriteRegister(REG_USER_CTRL, 0x40);   // Enable FIFO  
	WriteRegister(REG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	HAL_Delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	WriteRegister(REG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	ReadMultipleRegister(REG_FIFO_COUNTH, data, 2); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		ReadMultipleRegister(REG_FIFO_R_W, data, 12); // read data for averaging
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);    
		gyro_temp[0]  = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1]  = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2]  = (int16_t)(((int16_t)data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	}

	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity; }

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4)       & 0xFF;
	data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4)       & 0xFF;

	// Push gyro biases to hardware registers
//	WriteRegister(REG_XG_OFFSET_H, data[0]);
//	WriteRegister(REG_XG_OFFSET_L, data[1]);
//	WriteRegister(REG_YG_OFFSET_H, data[2]);
//	WriteRegister(REG_YG_OFFSET_L, data[3]);
//	WriteRegister(REG_ZG_OFFSET_H, data[4]);
//	WriteRegister(REG_ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	gyroscope_bias[0] = (float) gyro_bias[0] / (float) gyrosensitivity;  
	gyroscope_bias[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	gyroscope_bias[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	ReadMultipleRegister(REG_XA_OFFSET_H, data, 2); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	ReadMultipleRegister(REG_YA_OFFSET_H, data, 2);
	accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
	ReadMultipleRegister(REG_ZA_OFFSET_H, data, 2);
	accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++)
	{
		if ((accel_bias_reg[ii] & mask))
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  I2C_WriteRegister(XA_OFFSET_H, data[0]);
	I2C_WriteRegister(XA_OFFSET_L, data[1]);
	I2C_WriteRegister(YA_OFFSET_H, data[2]);
	I2C_WriteRegister(YA_OFFSET_L, data[3]);
	I2C_WriteRegister(ZA_OFFSET_H, data[4]);
	I2C_WriteRegister(ZA_OFFSET_L, data[5]);
	*/
	// Output scaled accelerometer biases for display in the main program
	accelerometer_bias[0] = (float)accel_bias[0] / (float)accelsensitivity; 
	accelerometer_bias[1] = (float)accel_bias[1] / (float)accelsensitivity;
	accelerometer_bias[2] = (float)accel_bias[2] / (float)accelsensitivity;
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