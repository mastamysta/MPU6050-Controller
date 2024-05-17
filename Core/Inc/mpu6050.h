/*
 * mpu6050.h
 *
 *  Created on: Jan 3, 2024
 *      Author: mastamysta
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

#include <stdint.h>
#include <stdbool.h>

typedef void (*mpu_i2c_complete_callback)(I2C_HandleTypeDef *hi2c);

typedef struct
{
	float x;
	float y;
	float z;
} vec3;

typedef struct
{
	vec3 gyro;
	vec3 accel;
} MPUData;

// MPU6050 retcode defines
#define MPU_RETCODE_OK		0
#define MPU_RETCODE_ERR		1

// Initialise the MPU6050
uint8_t MPUInit(I2C_HandleTypeDef *i2cHandle, bool AD0);

// Populate an array with most recent X, Y, Z readings of acceleration.
// Data - a pointer to an array of uns16_t, to be populated.
// Return - nonzero on error. Data is invalid if nonzero.
uint8_t MPUGetAccel(float *data);


// Populate an array with most recent X, Y, Z readings of gyroscope.
// Data - a pointer to an array of uns16_t, to be populated.
// Return - nonzero on error. Data is invalid if nonzero.
uint8_t MPUGetGyro(float *oData);

uint8_t MPUGetData(MPUData *data);

uint8_t MPUSetAccelSensitivity(uint8_t level);
uint8_t MPUSetGyroSensitivity(uint8_t level);

// Interrupt-based I2C functions

uint8_t MPUGetDataInterrupt(MPUData *data);
void MPUPostTransferCompleteCallback(mpu_i2c_complete_callback cb);
void MPURemoveTransferCompleteCallback();



#endif /* INC_MPU6050_H_ */
