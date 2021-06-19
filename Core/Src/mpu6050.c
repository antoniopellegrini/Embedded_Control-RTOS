/*
 * mpu6050.c
 *
 *  Created on: Jan 17, 2021
 *      Author: antoniopellegrini
 */

#include "mpu6050.h"


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	//HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ch, strlen(ch));
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1,0x0001);
	return ch;
}


MPU6050_StatusTypeDef MPU6050_Init (MPU_Data * mpu_data, uint8_t gyro_fs_select)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check != 104)
	{  // 0x68 will be returned by the sensor if everything goes well
		printf("MPU6050 not present!\n");
		return MPU_ERROR;
	}
	uint8_t status = 0;

	Data = 0;
	status += HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	Data = 0x07;
	status += HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

	// Set accelerometer configuration in ACCEL_CONFIG Register
	// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
	Data = 0x00;
	status += HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	// Set Gyroscopic configuration in GYRO_CONFIG Register
	// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
	// shift << 3 to write into FS_SEL (https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)

	Data = gyro_fs_select<<3;

	//FS=[2000, 1000, 500, 250] ---> 2000/250 = 8 ---> fattori moltiplicativi=[8,4,2,1].

	status += HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);

	switch(gyro_fs_select)
	{
	case 0:
		mpu_data->LSB_sensitivity = FS_SEL_0;
		break;
	case 1:
		mpu_data->LSB_sensitivity = FS_SEL_1;
		break;
	case 2:
		mpu_data->LSB_sensitivity = FS_SEL_2;
		break;
	case 3:
		mpu_data->LSB_sensitivity = FS_SEL_3;
		break;

	}

	printf("Status MPU %d\n", status);

	if(status == 0)
		return MPU_OK;
	else
		return MPU_ERROR;



}


MPU6050_StatusTypeDef MPU6050_Read_Gyro(MPU_Data * mpu_data)//
{

	int16_t Gyro_Z_RAW;

	/* Read 6 BYTES of data starting from GYRO_XOUT_H register*/
	uint8_t status = HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, mpu_data->Rec_Data, 2);


	Gyro_Z_RAW = (int16_t)(mpu_data->Rec_Data[0] << 8 | mpu_data->Rec_Data [1]);


	/* convert the RAW values into dps (°/s)
	 *   we have to divide according to the Full scale value set in FS_SEL
	 *   I have configured FS_SEL = 0. So I am dividing by 131.0
	 *   for more details check GYRO_CONFIG Register */
	mpu_data->last_raw_angle =  Gyro_Z_RAW/mpu_data->LSB_sensitivity;


	if(status == 0)
		return MPU_OK;
	else
		return MPU_ERROR;


}


MPU6050_StatusTypeDef MPU6050_Calculate_IMU_Error(MPU_Data * mpu_data, int seconds){



	int numOfIter = 200 * seconds;
	float sum = 0;			//media
	uint8_t status;
	for(int i = 0; i < numOfIter; i++)
	{
		status = MPU6050_Read_Gyro(mpu_data);
		sum = sum + mpu_data->last_raw_angle;
		HAL_Delay(5);
	}



	mpu_data->mean = sum/numOfIter;

	if(status == 0)
		return MPU_OK;
	else
		return MPU_ERROR;


}
