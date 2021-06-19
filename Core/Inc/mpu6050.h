/*
 * mpu6050.h
 *
 *  Created on: Jan 17, 2021
 *      Author: antoniopellegrini
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stdint.h"
#include "math.h"
#include "stdio.h"
#include "usart.h"
#include "i2c.h"


/* -- private defines --   */

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define GYRO_ZOUT_H_REG 0x47
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define FS_SEL_0 131.0f
#define FS_SEL_1 65.5f
#define FS_SEL_2 32.8f
#define FS_SEL_3 16.4f


typedef enum
{
	MPU_OK       = 0x00U,
	MPU_ERROR    = 0x01U
} MPU6050_StatusTypeDef;

/* -- private variables --   */


typedef struct MPU_Data{
	float mean;
	float LSB_sensitivity;
	float last_raw_angle;
	int16_t Gyro_Z_RAW;
	uint8_t Rec_Data[2];

}MPU_Data;


/* -- functions declarations  -- */

MPU6050_StatusTypeDef MPU6050_Init(MPU_Data * mpu_data,uint8_t gyro_fs);
MPU6050_StatusTypeDef MPU6050_Read_Gyro ( MPU_Data * mpu_data);
MPU6050_StatusTypeDef MPU6050_Calculate_IMU_Error( MPU_Data * mpu_data, int seconds);

#endif /* INC_MPU6050_H_ */
