/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "gpio.h"
#include "config.h"
#include "crc.h"
#include "dac.h"
#include "tim.h"
#include "utils.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


//variabili per HAL_GPIO_EXTI_Callback
uint32_t old_time=0;
uint32_t time;
int i=0;


uint32_t samples_per_seconds = 1;
uint32_t multiplier = 1;
uint32_t sample_time;

uint8_t debug_active = 1;
uint8_t is_Master = 1;   // 0 --> slave; 1--> master
//if timer interrupt is enabled leave this in slave mode


//pid
// float kp = 22,5 //ORIGINALE

float kp = 30.0;
float ki = 10.0;
float kd = 22.5;

//float kp = 22.5;
//float ki = 8.18;
//float kd = 15.46;


float debug_f1;
float debug_f2;

int debug_direction_P1;
int debug_direction_P2;

float debug_PID_P1;
float debug_PID_P2;

typedef struct angle_data{
	union{
		uint32_t i[1];
		float f[1];
	}angle;
	uint32_t crc;
} angle_data;

angle_data no_data;





/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ThreadP1Handle;
osThreadId ThreadP2Handle;
osThreadId SensorReadHandle;
osThreadId DebugThreadHandle;
osThreadId InitTaskHandle;
osMessageQId Sensor1QueueHandle;
osMessageQId Sensor2QueueHandle;
osMessageQId testHandle;
osSemaphoreId P1ITSemHandle;
osSemaphoreId P2ITSemHandle;
osSemaphoreId DebugThreadSemHandle;
osSemaphoreId MissionTimerSemHandle;
osSemaphoreId CountingSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//INTERRUPT ESTERNO

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == InputInterrupt_Pin){

		//osThreadResume(InterruptSyncHandle);
		osSemaphoreRelease(P1ITSemHandle);
		osSemaphoreRelease(P2ITSemHandle);

	}
}

//INTERRUPT TIMER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


	if( htim->Instance == TIM2){

		//printf("Timer\r\n");
		//TOGGLA il pin OutputInterrupt



		//osThreadResume(InterruptSyncHandle);
		osSemaphoreRelease(P1ITSemHandle);
		osSemaphoreRelease(P2ITSemHandle);
	}

	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	if (htim->Instance == TIM6) {

	}
	if (htim->Instance == TIM7){
		osSemaphoreRelease(MissionTimerSemHandle);

		//TODO implements 1 second timer

	}
}


angle_data queue_receive(osMessageQId queue_id){

	osEvent event = osMessageGet(queue_id, 100);
	if (event.status == osEventMessage){
		return *((angle_data *)event.value.v);
	}else{
		return no_data;
	}
}

void get_I2C_confermation( uint8_t * pData, uint8_t * receiveBuf){		// uint8_t * pData == uint8_y pData[]  <<-- stessa cosa


	HAL_I2C_Slave_Transmit(&hi2c3, pData, (uint16_t) 32, HAL_MAX_DELAY);
	printf("transmitted!\n");


	printf("wait to receive ");
	HAL_I2C_Slave_Receive(&hi2c3, receiveBuf, (uint16_t) 1, HAL_MAX_DELAY);
	printf("received: %s\n", receiveBuf);

//	HAL_I2C_Slave_Receive_IT(&hi2c3, pData, (uint16_t) 32);



}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){

	if (hi2c->Instance == I2C3){

		//TODO: INVIA NOTIFICA AL CASE 3

	}


}



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void P1EntryFunc(void const * argument);
void P2EntryFunc(void const * argument);
void SensorReadFunc(void const * argument);
void DebugThreadFunc(void const * argument);
void InitTaskFunc(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */
	sample_time = 1000/samples_per_seconds*multiplier;


	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of P1ITSem */
	osSemaphoreDef(P1ITSem);
	P1ITSemHandle = osSemaphoreCreate(osSemaphore(P1ITSem), 1);

	/* definition and creation of P2ITSem */
	osSemaphoreDef(P2ITSem);
	P2ITSemHandle = osSemaphoreCreate(osSemaphore(P2ITSem), 1);

	/* definition and creation of DebugThreadSem */
	osSemaphoreDef(DebugThreadSem);
	DebugThreadSemHandle = osSemaphoreCreate(osSemaphore(DebugThreadSem), 1);

	/* definition and creation of MissionTimerSem */
	osSemaphoreDef(MissionTimerSem);
	MissionTimerSemHandle = osSemaphoreCreate(osSemaphore(MissionTimerSem), 1);

	/* definition and creation of CountingSemaphore */
	osSemaphoreDef(CountingSemaphore);
	CountingSemaphoreHandle = osSemaphoreCreate(osSemaphore(CountingSemaphore), 2);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of Sensor1Queue */
	osMessageQDef(Sensor1Queue, 1, float);
	Sensor1QueueHandle = osMessageCreate(osMessageQ(Sensor1Queue), NULL);

	/* definition and creation of Sensor2Queue */
	osMessageQDef(Sensor2Queue, 1, float);
	Sensor2QueueHandle = osMessageCreate(osMessageQ(Sensor2Queue), NULL);

	/* definition and creation of test */
	osMessageQDef(test, 16, uint16_t);
	testHandle = osMessageCreate(osMessageQ(test), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 256);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of ThreadP1 */
	osThreadDef(ThreadP1, P1EntryFunc, osPriorityNormal, 0, 256);
	ThreadP1Handle = osThreadCreate(osThread(ThreadP1), NULL);

	/* definition and creation of ThreadP2 */
	osThreadDef(ThreadP2, P2EntryFunc, osPriorityNormal, 0, 256);
	ThreadP2Handle = osThreadCreate(osThread(ThreadP2), NULL);

	/* definition and creation of SensorRead */
	osThreadDef(SensorRead, SensorReadFunc, osPriorityNormal, 0, 256);
	SensorReadHandle = osThreadCreate(osThread(SensorRead), NULL);

	/* definition and creation of DebugThread */
	osThreadDef(DebugThread, DebugThreadFunc, osPriorityIdle, 0, 256);
	DebugThreadHandle = osThreadCreate(osThread(DebugThread), NULL);

	/* definition and creation of InitTask */
	osThreadDef(InitTask, InitTaskFunc, osPriorityHigh, 0, 512);
	InitTaskHandle = osThreadCreate(osThread(InitTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */


	//sets semaphores to 0
	osSemaphoreWait(P2ITSemHandle, osWaitForever);
	osSemaphoreWait(P1ITSemHandle, osWaitForever);
	osSemaphoreWait(DebugThreadSemHandle, osWaitForever);
	osSemaphoreWait(CountingSemaphoreHandle, osWaitForever);
	osSemaphoreWait(CountingSemaphoreHandle, osWaitForever);
	//fermo il thread della board slave, in modo da riesumarlo dall'interrupt






	/* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	//	/* Infinite loop */
	//	for(;;)
	//	{
	//		osDelay(1);
	//	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_P1EntryFunc */
/**
 * @brief Function implementing the ThreadP1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_P1EntryFunc */
void P1EntryFunc(void const * argument)
{
	/* USER CODE BEGIN P1EntryFunc */



	angle_data data;
	uint32_t crc;

	float current_angle=0.0;
	float desired_angle = 0.0;
	float first_angle = 0.0;


	hpid PidHandler;



	int pid_DAC = 0;
	int first_loop = 1 ;


	/* Infinite loop */
	for(;;)
	{

		if(osSemaphoreWait(P1ITSemHandle, osWaitForever) == osOK){

			if (osSemaphoreRelease(CountingSemaphoreHandle) == osOK){


				//receive data from P1 with a queue
				data = queue_receive(Sensor1QueueHandle);

				if (first_loop){
					first_angle = data.angle.f[0];
					printf("[!] First angle: %f\r\n", first_angle );
					first_loop = 0;

				}

				current_angle = data.angle.f[0] - first_angle ;

				//calculate crc and check if is equal to the one received
				crc = HAL_CRC_Calculate(&hcrc, data.angle.i,1);

				if (crc == data.crc){





					//calculates pid

					PID_Calculate(&PidHandler, xTaskGetTickCount(), current_angle, desired_angle, kp, ki, kd);


					//output direction on gpio:     0--> left     1 --> rigth

					HAL_GPIO_WritePin(DirectionP1_GPIO_Port, DirectionP1_Pin, PID_Get_Direction(&PidHandler));


					//converts pid for DAC

					pid_DAC = (uint32_t) fabs(PID_Get_Actuation(&PidHandler));
					pid_DAC = UTIL_Constrain(pid_DAC, 0, 4096);

					//output pid on DAC

					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pid_DAC);

				} else {
					printf("P1 - CRC Error\r\n");

					/* TODO: Che succede se il CRC è errato?
					 *
					 * possibili soluzioni:
					 *
					 *  - invio nell'union di vecchio e nuovo angolo, scegliendo quello vecchio?
					 *  - oppure il processo richiede la ri-lettura del sensore? in queston caso devo gestire il counting semaphore
					 *
					 *
					 * */
				}
			}

			//debug
			if(debug_active){
				debug_f1=current_angle;
				debug_direction_P1 = PID_Get_Direction(&PidHandler);
				debug_PID_P1 = PID_Get_Actuation(&PidHandler);
			}
		}
		osDelay(1);
	}
	/* USER CODE END P1EntryFunc */
}

/* USER CODE BEGIN Header_P2EntryFunc */
/**
 * @brief Function implementing the ThreadP2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_P2EntryFunc */
void P2EntryFunc(void const * argument)
{
	/* USER CODE BEGIN P2EntryFunc */

	/*TODO: se non mi arriva l'interrupt bisognerebbe negoziare il ruolo di master/slave
	 * quindi al posto di osWaitForever potrebbe esserci un timeout al cui scadere chiamiamo
	 * la funzione di negoziazione */




	angle_data data;
	uint32_t crc;

	float desired_angle = 0.0;
	float current_angle = 0.0;

	hpid PidHandler;

	int pid_DAC = 0;

	float first_angle = 0.0;
	int first_loop = 1;



	/* Infinite loop */
	for(;;)
	{

		if (osSemaphoreWait(P2ITSemHandle, osWaitForever) == osOK){

			if(osSemaphoreRelease(CountingSemaphoreHandle) == osOK){

				//receive data from P2 with a queue
				data = queue_receive(Sensor2QueueHandle);

				//start from 0
				if (first_loop){
					first_angle = data.angle.f[0];
					printf("[!] First angle: %f\r\n", first_angle );
					first_loop = 0;


				}

				current_angle = data.angle.f[0] - first_angle;


				//calculates crc and check if is equal to the one received
				crc = HAL_CRC_Calculate(&hcrc, data.angle.i,1);

				if (crc == data.crc){


					//pid

					PID_Calculate(&PidHandler, xTaskGetTickCount(), current_angle, desired_angle, kp, ki, kd);



					//output direction on gpio:     0--> left     1 --> rigth

					HAL_GPIO_WritePin(DirectionP2_GPIO_Port, DirectionP2_Pin, PID_Get_Direction(&PidHandler));


					//converts pid for DAC

					pid_DAC = (uint32_t) fabs(PID_Get_Actuation(&PidHandler));
					pid_DAC = UTIL_Constrain(pid_DAC, 0, 4096);


					//writes PID to DAC

					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pid_DAC);

				}else{
					printf("P2 - CRC Error\r\n");
				}
			}

			//debug
			if (debug_active){
				debug_f2 = current_angle;
				debug_direction_P2 = PID_Get_Direction(&PidHandler);
				debug_PID_P2 = PID_Get_Actuation(&PidHandler);
			}

		}
		osDelay(1);

	}
	/* USER CODE END P2EntryFunc */
}

/* USER CODE BEGIN Header_SensorReadFunc */
/**
 * @brief Function implementing the SensorRead thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_SensorReadFunc */
void SensorReadFunc(void const * argument)
{
	/* USER CODE BEGIN SensorReadFunc */
	/* Infinite loop */


	angle_data yaw;
	angle_data yaw_new;

	yaw.angle.f[0] = 0.0;
	yaw_new.angle.f[0] = 0.0;



	//uint32_t crc;

	/* Infinite loop */
	for(;;)
	{


		if(osSemaphoreWait(CountingSemaphoreHandle, osWaitForever) == osOK){
			if(osSemaphoreWait(CountingSemaphoreHandle, osWaitForever)== osOK){


				MPU6050_Read_Gyro();

				currentTime = xTaskGetTickCount();
				elapsedTime = currentTime - previousTime;
				previousTime = currentTime;

				Gz = (Gz - Z_error )*Gyro_FS_Mult_Factor;


				yaw_new.angle.f[0] = yaw.angle.f[0]+ Gz * elapsedTime * 0.001; // elapsedTime/1000 to get seconds


				//ignore small angles:
				if((fabs(yaw.angle.f[0]-yaw_new.angle.f[0])<(Gz_stdev))){
					yaw_new.angle.f[0] = yaw.angle.f[0];
				}

				//normalize the angle between -180° and 180°

				//				if (yaw_new.angle.f[0] >= 180.0)
				//					yaw_new.angle.f[0] -= 360;
				//				if (yaw_new.angle.f[0] <= -180.0)
				//					yaw_new.angle.f[0] += 360;


				yaw_new.crc = HAL_CRC_Calculate(&hcrc, yaw_new.angle.i,1);

				//send angle to queues
				osMessagePut(Sensor1QueueHandle, (uint32_t) &yaw_new, 100);
				osMessagePut(Sensor2QueueHandle, (uint32_t) &yaw_new, 100);


				yaw.angle.f[0]=yaw_new.angle.f[0];
				yaw.crc = HAL_CRC_Calculate(&hcrc,yaw.angle.i,1);

				if(debug_active){
					osSemaphoreRelease(DebugThreadSemHandle);
				}

			}
		}
		osDelay(1);
	}
	/* USER CODE END SensorReadFunc */
}

/* USER CODE BEGIN Header_DebugThreadFunc */
/**
 * @brief Function implementing the DebugThread thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_DebugThreadFunc */
void DebugThreadFunc(void const * argument)
{
	/* USER CODE BEGIN DebugThreadFunc */
	/* Infinite loop */
	for(;;)
	{
		if(osSemaphoreWait(DebugThreadSemHandle, osWaitForever) == osOK){
			//printf("debug-thread: P1_angle=%f, P2_angle=%f\r\n",debug_f1,debug_f2);
			printf("DEBUG  --- P1_pid=%f P1_direction=%d   ---  P2_pid=%f P2_direction=%d\r\n", debug_PID_P1,debug_direction_P1, debug_PID_P2, debug_direction_P2);
		}
		osDelay(1);
	}
	/* USER CODE END DebugThreadFunc */
}

/* USER CODE BEGIN Header_InitTaskFunc */
/**
 * @brief Function implementing the InitTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_InitTaskFunc */
void InitTaskFunc(void const * argument)
{
	/* USER CODE BEGIN InitTaskFunc */
	printf("[OS] - Start configuration thread\r\n");

	int state = 0;
	int timer = -5;
	int in_powered_ascent = 0;
	uint8_t cmd1[32] = "STATE 1 WFC";

	//uint8_t msg[32]="TELEMETRY";   //TODO temporaneo, da capire quale telemetria inviare


	uint8_t receiveBuf[1];

	for(;;){

		switch(state){

		//CASE 0: Inizializzo l'mpu

		case 0:

			//transmit state 0


			//first loop initialization



				if(MPU6050_Init(3) == MPU_OK){



					//transmit mpu init success

					state = 1;
					break;


					//TODO inizializza l'angolo del servo a 0 (invia ai processi quest'informazione)


				} else {

					printf("[MPU6050] init Failed!\r\n");
					state = -1;
					break;
				}




		//CASE 1: aspetto il comando di via, di ricalibrazione o di abort


		case 1:


			get_I2C_confermation( cmd1, receiveBuf);





			if ( receiveBuf[0] == 1){
				printf("[OS] Re-calibration");

				MPU6050_Calculate_IMU_Error(2);
				printf("Gz: %f Gz_error: %f\r\n", Gz, GyroErrorZ);
				printf("[OS] Calibration done\r\n");

			}



			if ( receiveBuf[0] == 2){
				printf("[OS] Starting countdown");

				state = 2;
				break;

			}



			if (receiveBuf[0] == -1){
				printf("[OS] ABORT");

				state = -1;
				break;

			}



		//CASE 2 - Avvio il countdown
		//TODO implementare l'i2c non bloccante oppure un thread a parte per le comunicazioni i2c

		case 2:



			//start MISSION timer

			//se arriva un messaggio i2c -1 devo abortire

			HAL_TIM_Base_Start_IT(&htim7);

			state = 3;
			break;



		case 3:


			//get mission timer semaphore, and increment timer by 1 second

			if(osSemaphoreWait(MissionTimerSemHandle, osWaitForever) == HAL_OK)
				timer ++;

			printf("Mission timer: %d\n",timer);


			if (timer == 0 && !in_powered_ascent){

				if(is_Master){


					printf("[OS] MPU is Master - ");
					osThreadResume(SensorReadHandle);
					osThreadResume(DebugThreadHandle);
					osThreadResume(ThreadP1Handle);
					osThreadResume(ThreadP2Handle);



					if (HAL_TIM_Base_Start_IT(&htim2) == HAL_OK){ // custom: init timer for timer interrupt
						printf(" Stating sampling timer...\r\n\n");
						in_powered_ascent = 1;

					} else {
						printf("[OS] Timer failed to start - aborting.\r\n");
						state = -1;
						break;
					}



				}else{
					in_powered_ascent = 1;
					printf("[OS] MPU is Slave, waiting for external interrupt...\r\n\n");
				}

			}
			if (timer == 30){


				if (is_Master)
					HAL_TIM_Base_Stop_IT(&htim2);

				osThreadSuspend(SensorReadHandle);
				osThreadSuspend(DebugThreadHandle);
				osThreadSuspend(ThreadP1Handle);
				osThreadSuspend(ThreadP2Handle);

				state = 4;

				break;
			}


			//TX telemetry    TODO: invia la telemetria e non un messaggio fisso xD


			uint8_t msg[32];   //TODO temporaneo, da capire quale telemetria inviare


			//scrivi il timer nel buffer dei dati della telemetria

			snprintf((char *) msg, sizeof(msg), "TELEMETRY - t=%d\n, ",timer);


			HAL_I2C_Slave_Transmit(&hi2c3, msg, (uint16_t) 32, HAL_MAX_DELAY);
			printf("transmitted!\n");


		case 4:
			printf("[state = COASTING]\n");
			osDelay(1000);

			// TODO get angles from P1,P2 and if needed command a trajectory adjustment

			break;


		case -1:
			printf("[state = ABORT]\n");
			osThreadSuspendAll();
			break;
		}
	}


	//Gyro_Z_RAW = 0;
	//					HAL_GPIO_WritePin(Alive_backup_GPIO_Port, Alive_backup_Pin, 1);
	//kill this thread
	osThreadTerminate(InitTaskHandle);

	/* USER CODE END InitTaskFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


//	int freq = HAL_RCC_GetPCLK1Freq() * 2;  // APB1 Timer Clock f = HAL_RCC_GetPCLK1Freq() * 2;
//	int ARR = 89999999;
//	float Tc = 0.02;  //tempo di campionamento desiderato [ms]
//
//
//	int Calculated_PSC = ((freq * Tc) / (1 + ARR)) - 1;
//	int Calculated_Tc = ((1+ARR)*(Calculated_PSC+1))/freq;


//	printf("Freq=%d Calculated PSC=%d, Calculated_Tc=%d\r\n",freq, Calculated_PSC, Calculated_Tc);

/*	states:
 * 		abort 				-1
 * 		on gound		    0
 * 		ready for fligth    1
 * 		powered ascend		2
 * 		coasting			3
 *
*/
 /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
