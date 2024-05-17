/*
 * mpu6050.c
 *
 *  Created on: Jan 3, 2024
 *      Author: mastamysta
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"

#include "mpu6050.h"

// I2C data size defines
#define DATA_SIZE_ONE_BYTE 1
#define DATA_SIZE_TWO_BYTE 2

#define I2C_TIMEOUT_ONE_SECOND 1000

// MPU accel index defines
#define ACCEL_X_INDEX 0
#define ACCEL_Y_INDEX 1
#define ACCEL_Z_INDEX 2

// MPU gyro index defines
#define GYRO_X_INDEX 0
#define GYRO_Y_INDEX 1
#define GYRO_Z_INDEX 2

// Default address of MPU6050, without AD0 set.
static uint8_t i2cAddr = 0b1101000 << 1;

// PWR_MGMT1 register constants
static const uint8_t pwrMgmtAddr = 0x6B;
static const uint8_t pwrMgmtNoSleep = 0b0;
static const uint8_t pwrMgmtRestart = 0b1 << 7;

// INT_ENABLE register constants
static const uint8_t intEnableAddr = 0x38;
static const uint8_t intEnableEnabled = 0x01;
static const uint8_t intEnableDisabled = 0x00;

// Data block address and size - so we can burst read all data at once
static const uint8_t gyroAccelTempAddr = 0x3B;
static const uint8_t gyroAccelTempBlockSize = 14;

// ACCEL_CONFIG register constants
static const uint8_t accelConfigAddr = 0x1C;
static const uint8_t gyroConfigAddr = 0x1B;

// GYRO_*_OUT register constants
static const uint16_t gyroXHighAddr = 0x43;
static const uint16_t gyroXLowAddr = 0x44;
static const uint16_t gyroYHighAddr = 0x45;
static const uint16_t gyroYLowAddr = 0x46;
static const uint16_t gyroZHighAddr = 0x47;
static const uint16_t gyroZLowAddr = 0x48;

// ACCEL_*_OUT register constants
static const uint8_t accelXHighAddr = 0x3B;
static const uint8_t accelXLowAddr = 0x3C;
static const uint8_t accelYHighAddr = 0x3D;
static const uint8_t accelYLowAddr = 0x3E;
static const uint8_t accelZHighAddr = 0x3F;
static const uint8_t accelZLowAddr = 0x40;

// Gyroscope & accelerometer sensitivity levels
static uint8_t gyro_sensitivity = 0x00;
static uint8_t accel_sensitivity = 0x00;

// HAL_I2C handle for connection to MPU_6050
static I2C_HandleTypeDef *i2cHandle;

static float twos_complement_size_to_float(uint16_t input)
{
	bool neg = false;

	// Negative number need to be taken out of two's complement
	if (input >> 15)
	{
		neg = true;
		input = ~(input) + 1;
	}

	return neg ? -input : input;
}

// Specifically for burst reading the HIGH-LOW register pair scheme on the MPU60X0 gyro/accel.
static uint8_t mpu_read_value_register(uint8_t mpu_high_addr, uint16_t* data)
{
	HAL_StatusTypeDef ret;

	// Burst read most-significant byte, then least significant byte.
	// Burst read is used because a pair of individual reads requires the master to check
	// whether the slave data has been updated since the first read.
	if ((ret = HAL_I2C_Mem_Read(i2cHandle,
								i2cAddr,
								mpu_high_addr,
								I2C_MEMADD_SIZE_8BIT,
								data,
								DATA_SIZE_TWO_BYTE,
								I2C_TIMEOUT_ONE_SECOND)) != HAL_OK)
	{
		printf("ERROR: Failed to read ACCEL_ZOUT_H. HALStatus: %i\n", ret);
		return MPU_RETCODE_ERR;
	}

	// Swap low and high bytes as MSB is read first

	uint8_t tmp = ((uint8_t*)data)[1];
	*data = *data << 8;
	((uint8_t*)data)[0] = tmp;

	return MPU_RETCODE_OK;
}

// Write a single-byte register
static uint8_t mpu_write_reg(uint8_t reg_addr, uint8_t data)
{
	HAL_StatusTypeDef ret;

	if ((ret = HAL_I2C_Mem_Write(i2cHandle,
									i2cAddr,
									reg_addr,
									I2C_MEMADD_SIZE_8BIT,
									&data,
									DATA_SIZE_ONE_BYTE,
									I2C_TIMEOUT_ONE_SECOND)) != HAL_OK)
	{
		printf("ERROR: Failed to set register %i. HALStatus: %i\n", reg_addr, ret);
		return MPU_RETCODE_ERR;
	}

	return MPU_RETCODE_OK;
}

uint8_t MPUSetAccelSensitivity(uint8_t level)
{
	if (level > 3 || level < 0)
		return MPU_RETCODE_ERR;

	if(!mpu_write_reg(accelConfigAddr, level << 3))
		accel_sensitivity = level;
	else
	{
		printf("ERROR: Failed to set accelerometer sensitivity register.");
		return MPU_RETCODE_ERR;
	}


	return MPU_RETCODE_OK;
}

uint8_t MPUSetGyroSensitivity(uint8_t level)
{
	if (level > 3 || level < 0)
		return MPU_RETCODE_ERR;

	if(!mpu_write_reg(gyroConfigAddr, level << 3))
		accel_sensitivity = level;
	else
	{
		printf("ERROR: Failed to set gyroscope sensitivity register.");
		return MPU_RETCODE_ERR;
	}

	return MPU_RETCODE_OK;
}

uint8_t MPUSetDataReadyInterruptEnable(bool value)
{
	if(mpu_write_reg(intEnableAddr, value ? intEnableEnabled : intEnableDisabled))
	{
		printf("ERROR: Failed to enable/disable data ready interrupt.");
		return MPU_RETCODE_ERR;
	}

	return MPU_RETCODE_OK;
}

uint8_t MPUInit(I2C_HandleTypeDef *hi2c1, bool AD0)
{
	HAL_StatusTypeDef ret;

	// Store I2C handle in static variable, so it needn't be passed to all calls.
	i2cHandle = hi2c1;

	// Update LSB of i2cAddr depending on AD0 pin. Shift left for R/W flag.
	i2cAddr |= (AD0 << 1);

	while((ret = HAL_I2C_IsDeviceReady(i2cHandle, i2cAddr, 64, 1000)) != HAL_OK);

	if(mpu_write_reg(pwrMgmtAddr, pwrMgmtNoSleep))
	{
		printf("ERROR: Failed to reset PWR_MGMT1 register. HALStatus: %i\n", ret);
		return MPU_RETCODE_ERR;
	}

	return MPU_RETCODE_OK;
}

uint8_t MPUGetAccel(float *oData)
{
	HAL_StatusTypeDef ret;
	uint16_t data[3] = {0, 0, 0};

	memset(oData, 0, sizeof(float) * 3);

	if (!i2cHandle)
	{
		printf("ERROR: Must initialise MPU6050 before attempting to read.\n");
		return MPU_RETCODE_ERR;
	}

	mpu_read_value_register(accelZHighAddr, &data[ACCEL_Z_INDEX]);
	mpu_read_value_register(accelYHighAddr, &data[ACCEL_Y_INDEX]);
	mpu_read_value_register(accelXHighAddr, &data[ACCEL_X_INDEX]);

	// Normalise the data
	for (int i = 0; i <= 2; i++)
	{
		oData[i] = twos_complement_size_to_float(data[i]);
		oData[i] = oData[i] / 16384; // scale factor for 1g
	}

	return MPU_RETCODE_OK;
}

uint8_t MPUGetGyro(float *oData)
{
	HAL_StatusTypeDef ret;
	uint16_t data[3] = {0, 0, 0};

	memset(oData, 0, sizeof(uint16_t) * 3);

	if (!i2cHandle)
	{
		printf("ERROR: Must initialise MPU6050 before attempting to read.\n");
		return MPU_RETCODE_ERR;
	}

	mpu_read_value_register(gyroZHighAddr, &data[GYRO_Z_INDEX]);
	mpu_read_value_register(gyroYHighAddr, &data[GYRO_Y_INDEX]);
	mpu_read_value_register(gyroXHighAddr, &data[GYRO_X_INDEX]);

	// Get two's complement of data.
	for (int i = 0; i <= 2; i++)
	{
		oData[i] = twos_complement_size_to_float(data[i]);
		oData[i] = oData[i] / 131; // Scale factor for degrees per second.
	}

	return MPU_RETCODE_OK;
}

// ------ FUNCS FOR NON-BLOCKING I2C TRANSFER ----------

static mpu_i2c_complete_callback transfer_complete_callback = NULL;

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (transfer_complete_callback)
		transfer_complete_callback(hi2c);
}

void MPUPostTransferCompleteCallback(mpu_i2c_complete_callback cb)
{
	transfer_complete_callback = cb;
}

void MPURemoveTransferCompleteCallback()
{
	transfer_complete_callback = NULL;
}

uint8_t MPUGetDataInterrupt(MPUData *data)
{
	HAL_StatusTypeDef ret;

	uint16_t bytes[gyroAccelTempBlockSize];

	// Burst entire block of reading registers from MPU6050.
	if ((ret = HAL_I2C_Mem_Read_IT(i2cHandle,
								i2cAddr,
								gyroAccelTempAddr,
								I2C_MEMADD_SIZE_8BIT,
								(uint8_t*)bytes,
								gyroAccelTempBlockSize)) != HAL_OK)
	{
		printf("ERROR: Failed to read data block! HALStatus: %i\n", ret);
		return MPU_RETCODE_ERR;
	}

	return MPU_RETCODE_OK;
}

uint8_t MPUTranslateData(MPUData *data, uint16_t *bytes)
{
	data->accel.x = twos_complement_size_to_float(bytes[0]);
	data->accel.y = twos_complement_size_to_float(bytes[1]);
	data->accel.z = twos_complement_size_to_float(bytes[2]);

	// Skip addresses storing temperature data.

	data->gyro.x = twos_complement_size_to_float(bytes[4]);
	data->gyro.y = twos_complement_size_to_float(bytes[5]);
	data->gyro.z = twos_complement_size_to_float(bytes[6]);

	return MPU_RETCODE_OK;
}
