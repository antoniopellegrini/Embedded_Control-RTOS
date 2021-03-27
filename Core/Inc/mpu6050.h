/*
 * mpu6050.h
 *
 *  Created on: Jan 17, 2021
 *      Author: antoniopellegrini
 */



#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


//#include "main.h"
#include "stdint.h"
#include "math.h"
#include "stdio.h"
#include "usart.h"
#include "i2c.h"
//private defines

/*	-- printf definition --*/

int16_t Gyro_X_RAW;
int16_t Gyro_Y_RAW;
int16_t Gyro_Z_RAW;

uint8_t Gyro_FS_Select;
uint8_t Gyro_FS_Mult_Factor;

float yaw_actual, yaw_filtered;
volatile float yaw_array[5];// = {0,0,0,0,0};

float Gz;
float GyroErrorZ;

long elapsedTime, currentTime, previousTime;
float Z_error;

float Gz_variance;
float Gz_stdev;

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

typedef enum
{
	MPU_OK       = 0x00U,
	MPU_ERROR    = 0x01U
} MPU6050_StatusTypeDef;

/* -- private variables --   */


/* -- FUNCTIONS  -- */

MPU6050_StatusTypeDef MPU6050_Init(uint8_t gyro_fs);
void MPU6050_Read_Gyro (void);
void MPU6050_Calculate_IMU_Error(int);
//void filter(float *, float *, volatile float *);



#endif /* INC_MPU6050_H_ */

















