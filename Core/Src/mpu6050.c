/*
 * mpu6050.c
 *
 *  Created on: Jan 17, 2021
 *      Author: antoniopellegrini
 */
//test

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


void filter(float *actual_value, float *value, volatile float *array){

	//FILTRO 5 Sample Average To Smooth Out The Data
	array[0] = array[1];
	array[1] = array[2];
	array[2] = array[3];
	array[3] = array[4];
	array[4] = *actual_value;
	//la Media degli ultimi 5 valori equivale a...
	*value= (array[0] + array[1] + array[2] + array[3] + array[4]) / 5;


}


MPU6050_StatusTypeDef MPU6050_Init (MPU_Data * mpu_data, uint8_t gyro_fs_select)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{


		Data = 0;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		// shift << 3 to write into FS_SEL (https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)


		//FS=[2000, 1000, 500, 250] ---> 2000/250 = 8 ---> fattori moltiplicativi=[8,4,2,1].

		mpu_data->FS_Mult_Factor = pow(2,gyro_fs_select);

		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);



		return MPU_OK;

	} else{

		return MPU_ERROR;
	}

}


float MPU6050_Read_Gyro (MPU_Data * mpu_data)
{
	uint8_t Rec_Data[2];
	uint16_t temp;

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

//	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	HAL_I2C_Mem_Read (&hi2c2, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 1000);

//	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
//	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
//	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);


	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	//Gx = Gyro_X_RAW/131.0;
	//Gy = Gyro_Y_RAW/131.0;

	return  (temp/131.0)* mpu_data->FS_Mult_Factor;

}


void MPU6050_Calculate_IMU_Error(MPU_Data * mpu_data, int seconds){

	printf("\n	[MPU6050] Start of calibration, calculating biases...\n");


	/*	Calcolo della varianza con l'algoritmo di Welford
	 *  https://stats.stackexchange.com/a/235151
	 */

	int numOfIter = 200 * seconds;

	float delta = 0;
	float msq = 0;
	float reading = 0;
	float mean = 0;			//media
	float variance = 0;		//varianza
	float std_dev = 0;		//deviazione standard

	mean = MPU6050_Read_Gyro(mpu_data);

	for(int i = 0; i < numOfIter; i++){
		reading = MPU6050_Read_Gyro(mpu_data);
		delta = reading - mean;
		mean = mean + delta / (i+1);
		msq = msq + delta  * (reading - mean);
		osDelay(5);
	}

	variance = msq / (numOfIter - 1);
	std_dev = sqrtf(variance);

	printf("CALIBRATION DATA :%f,%f,%f\n",mean,variance,std_dev);

	mpu_data->mean = mean;
	mpu_data->variance = variance;
	mpu_data->stdev = std_dev;


	printf("	[MPU6050] End of calibration.\n\n");

}
