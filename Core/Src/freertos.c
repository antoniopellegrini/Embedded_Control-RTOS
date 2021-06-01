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
#define MOTOR_ON 0
#define MOTOR_OFF 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


//variabili per HAL_GPIO_EXTI_Callback

uint8_t debug_active = 0;
uint8_t telemetry_enabled = 1;
uint8_t is_Master = 1;   // 0 --> slave; 1--> master
//if timer interrupt is enabled leave this in slave mode

int softError = 0;

//pid
// float kp = 22,5 //ORIGINALE

float kp = 30.0;
float ki = 10.0;
float kd = 22.5;


//VARIABILI GLOBALI BRUTTE
float debug_f1;
float debug_f2;

int debug_direction_P1;
int debug_direction_P2;

float debug_PID_P1;
float debug_PID_P2;

float debug_current_angle_P1;



int debug_mission_timer;

typedef struct angle_data{
	union{
		uint32_t i[1];
		float f[1];
	}angle;
	uint32_t crc;
} angle_data;

typedef struct telemetry_data{
	float angle;
	float pid;
	int direction;

} telemetry_data;

typedef struct mission_data{
	int timer;
	int state;
	uint8_t is_master;

} mission_data;

typedef struct new_mission_data{
	float desired_angle;
	float degrees_per_second;
}new_mission_data;



angle_data no_data;

telemetry_data telemetry_no_data;
new_mission_data new_mission_no_data;

osPoolDef (object_pool, 16, MPU_Data);  // Declare memory pool
osPoolId  (object_pool_id);



/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ThreadP1Handle;
osThreadId ThreadP2Handle;
osThreadId SensorReadHandle;
osThreadId DebugThreadHandle;
osThreadId InitTaskHandle;
osThreadId TelemetryThreadHandle;
osMessageQId Sensor1QueueHandle;
osMessageQId Sensor2QueueHandle;
osMessageQId P1TelemetryQueueHandle;
osMessageQId P2TelemetryQueueHandle;
osMessageQId MissionTimerQueueHandle;
osMessageQId MewMissionP1QueueHandle;
osMessageQId MewMissionP2QueueHandle;
osMessageQId MpuDataQueueHandle;
osSemaphoreId P1ITSemHandle;
osSemaphoreId P2ITSemHandle;
osSemaphoreId DebugThreadSemHandle;
osSemaphoreId MissionTimerSemHandle;
osSemaphoreId TelemetryThreadSemHandle;
osSemaphoreId CountingSemaphoreHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

//INTERRUPT ESTERNO




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == InputInterrupt_Pin){

		//osThreadResume(InterruptSyncHandle);
		//printf("External Interrupt\n");

		osSemaphoreRelease(P1ITSemHandle);
		osSemaphoreRelease(P2ITSemHandle);
	}

	if (GPIO_Pin == USER_BUTTON_Pin){ // create Soft error

		if(softError == 0){
			softError = 1;
		}
		else{
			softError = 0;
		}

	}
}





//INTERRUPT TIMER
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


	if( htim->Instance == TIM2){

		//printf("Timer\r\n");
		//TOGGLA il pin OutputInterrupt
		HAL_GPIO_TogglePin(OutputTimer_GPIO_Port, OutputTimer_Pin);



		//osThreadResume(InterruptSyncHandle);
		osSemaphoreRelease(P1ITSemHandle);
		osSemaphoreRelease(P2ITSemHandle);
	}

	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}

	if (htim->Instance == TIM7){
		osSemaphoreRelease(MissionTimerSemHandle);
		HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);

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

telemetry_data telemetry_queue_receive(osMessageQId queue_id){

	osEvent event = osMessageGet(queue_id, 0);
	if (event.status == osEventMessage){
		return *((telemetry_data *)event.value.v);
	}else{
		return telemetry_no_data;
	}
}

new_mission_data new_mission_data_queue_receive(osMessageQId queue_id, uint32_t timeout){

	osEvent event = osMessageGet(queue_id, timeout);
	if (event.status == osEventMessage){
		return *((new_mission_data *)event.value.v);
	}else{
		return new_mission_no_data;
	}
}

void get_I2C_confermation( uint8_t * pData, uint8_t * receiveBuf){		// uint8_t * pData == uint8_y pData[]  <<-- stessa cosa

	HAL_I2C_Slave_Transmit(&hi2c3, pData, (uint16_t) 32, HAL_MAX_DELAY);

	printf("Wait for CMD...\n");
	HAL_I2C_Slave_Receive(&hi2c3, receiveBuf, (uint16_t) 1, HAL_MAX_DELAY);
	printf("	Received CMD: %d\n", receiveBuf[0]);

	//	HAL_I2C_Slave_Receive_IT(&hi2c3, pData, (uint16_t) 32);



}


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c3){

	if (hi2c3->Instance == I2C3){


	}


}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c3){

	//printf("Callback - Transmission completed\n");

}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void P1EntryFunc(void const * argument);
void P2EntryFunc(void const * argument);
void SensorReadFunc(void const * argument);
void DebugThreadFunc(void const * argument);
void InitTaskFunc(void const * argument);
void TelemetryThreadFunc(void const * argument);

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

  /* definition and creation of TelemetryThreadSem */
  osSemaphoreDef(TelemetryThreadSem);
  TelemetryThreadSemHandle = osSemaphoreCreate(osSemaphore(TelemetryThreadSem), 1);

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

  /* definition and creation of P1TelemetryQueue */
  osMessageQDef(P1TelemetryQueue, 1, float);
  P1TelemetryQueueHandle = osMessageCreate(osMessageQ(P1TelemetryQueue), NULL);

  /* definition and creation of P2TelemetryQueue */
  osMessageQDef(P2TelemetryQueue, 1, float);
  P2TelemetryQueueHandle = osMessageCreate(osMessageQ(P2TelemetryQueue), NULL);

  /* definition and creation of MissionTimerQueue */
  osMessageQDef(MissionTimerQueue, 1, int);
  MissionTimerQueueHandle = osMessageCreate(osMessageQ(MissionTimerQueue), NULL);

  /* definition and creation of MewMissionP1Queue */
  osMessageQDef(MewMissionP1Queue, 1, int);
  MewMissionP1QueueHandle = osMessageCreate(osMessageQ(MewMissionP1Queue), NULL);

  /* definition and creation of MewMissionP2Queue */
  osMessageQDef(MewMissionP2Queue, 1, int);
  MewMissionP2QueueHandle = osMessageCreate(osMessageQ(MewMissionP2Queue), NULL);

  /* definition and creation of MpuDataQueue */
  osMessageQDef(MpuDataQueue, 1, uint32_t);
  MpuDataQueueHandle = osMessageCreate(osMessageQ(MpuDataQueue), NULL);

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
  osThreadDef(InitTask, InitTaskFunc, osPriorityAboveNormal, 0, 512);
  InitTaskHandle = osThreadCreate(osThread(InitTask), NULL);

  /* definition and creation of TelemetryThread */
  osThreadDef(TelemetryThread, TelemetryThreadFunc, osPriorityBelowNormal, 0, 512);
  TelemetryThreadHandle = osThreadCreate(osThread(TelemetryThread), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */


	osThreadSuspend(TelemetryThreadHandle);

	//sets semaphores to 0
	osSemaphoreWait(P2ITSemHandle, osWaitForever);
	osSemaphoreWait(P1ITSemHandle, osWaitForever);
	osSemaphoreWait(DebugThreadSemHandle, osWaitForever);
	osSemaphoreWait(CountingSemaphoreHandle, osWaitForever);
	osSemaphoreWait(CountingSemaphoreHandle, osWaitForever);
	//fermo il thread della board slave, in modo da riesumarlo dall'interrupt

	osThreadSuspend(ThreadP1Handle);
	osThreadSuspend(ThreadP2Handle);
	osThreadSuspend(SensorReadHandle);
	osThreadSuspend(DebugThreadHandle);



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
	/* Infinite loop */
	for(;;)
	{
		//			HAL_GPIO_TogglePin(GreenLED_GPIO_Port, GreenLED_Pin);
		osDelay(1000);
	}
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

	float current_time;
	float elapsed_time;
	float old_time;
	float starting_time;
	float mission_time;



	angle_data data;
	uint32_t crc;

	float current_angle=0.0;
	float new_angle = 0.0;
	float first_angle = 0.0;


	hpid PidHandler;

	int last_motor_state;


	int pid_DAC = 0;
	int first_loop = 1 ;

	telemetry_data telemetry;

	new_mission_data mission_data = new_mission_data_queue_receive(MewMissionP1QueueHandle, osWaitForever);



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
					starting_time = xTaskGetTickCount();

				}

				current_angle = data.angle.f[0] - first_angle ;

				// Soft error

				if(softError == 1 ){
					data.angle.f[0] = data.angle.f[0] + 80.0;
					printf("Soft Error \n\r");
				}

				//calculate crc and check if is equal to the one received
				crc = HAL_CRC_Calculate(&hcrc, data.angle.i,1);

				if (crc == data.crc){

					if(last_motor_state == MOTOR_OFF){
						HAL_GPIO_WritePin(Alive_GPIO_Port, Alive_Pin, MOTOR_ON);
						last_motor_state = MOTOR_ON;
					}


					current_time = xTaskGetTickCount() - starting_time;
					elapsed_time = (current_time - old_time)/1000;
					old_time = current_time;
					mission_time = mission_time + elapsed_time;


					new_angle =  mission_data.degrees_per_second * mission_time;

					debug_current_angle_P1 = new_angle;

					if (new_angle >= mission_data.desired_angle ){
						new_angle = mission_data.desired_angle ;
					}

					//calculates pid

					PID_Calculate(&PidHandler, xTaskGetTickCount(), current_angle, new_angle, kp, ki, kd);


					//output direction on gpio:     0--> left     1 --> rigth

					HAL_GPIO_WritePin(DirectionP1_GPIO_Port, DirectionP1_Pin, PID_Get_Direction(&PidHandler));


					//converts pid for DAC

					pid_DAC = (uint32_t) fabs(PID_Get_Actuation(&PidHandler));
					pid_DAC = UTIL_Constrain(pid_DAC, 0, 4096);

					//output pid on DAC

					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, pid_DAC);


					//telemetry

					telemetry.angle = current_angle;
					telemetry.direction = PID_Get_Direction(&PidHandler);
					telemetry.pid = PID_Get_Actuation(&PidHandler);

					if (telemetry_enabled){

						osMessagePut(P1TelemetryQueueHandle, (uint32_t) &telemetry, 0);
					}


				} else {
					printf("P1 - CRC Error\r\n");

					//set alive to False
					HAL_GPIO_WritePin(Alive_GPIO_Port, Alive_Pin, MOTOR_OFF);
					last_motor_state = MOTOR_OFF;



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

	float current_time;
	float elapsed_time;
	float old_time;
	float starting_time;
	float mission_time;



	angle_data data;
	uint32_t crc;

	float new_angle = 0.0;
	float current_angle = 0.0;

	hpid PidHandler;

	int pid_DAC = 0;

	float first_angle = 0.0;
	int first_loop = 1;
	int last_motor_state;

	telemetry_data telemetry;

	new_mission_data mission_data = new_mission_data_queue_receive(MewMissionP2QueueHandle, osWaitForever);



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
					printf("[!] First ANGLE: %f\r\n", first_angle );
					first_loop = 0;
					starting_time = xTaskGetTickCount();



				}

				current_angle = data.angle.f[0] - first_angle;


				//calculates crc and check if is equal to the one received
				crc = HAL_CRC_Calculate(&hcrc, data.angle.i,1);

				if (crc == data.crc){

					if(last_motor_state == MOTOR_OFF){
						HAL_GPIO_WritePin(Alive_GPIO_Port, Alive_Pin, MOTOR_ON);
						last_motor_state = MOTOR_ON;
					}


					current_time = xTaskGetTickCount() - starting_time;
					elapsed_time = (current_time - old_time)/1000;
					old_time = current_time;
					mission_time = mission_time + elapsed_time;




					new_angle =  mission_data.degrees_per_second * mission_time;

					if (new_angle >= mission_data.desired_angle ){
						new_angle = mission_data.desired_angle ;
					}



					//pid

					PID_Calculate(&PidHandler, xTaskGetTickCount(), current_angle, new_angle, kp, ki, kd);



					//output direction on gpio:     0--> left     1 --> rigth

					HAL_GPIO_WritePin(DirectionP2_GPIO_Port, DirectionP2_Pin, PID_Get_Direction(&PidHandler));


					//converts pid for DAC

					pid_DAC = (uint32_t) fabs(PID_Get_Actuation(&PidHandler));
					pid_DAC = UTIL_Constrain(pid_DAC, 0, 4096);


					//writes PID to DAC

					HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, pid_DAC);



					//telemetry


					telemetry.angle = current_angle;
					telemetry.direction = PID_Get_Direction(&PidHandler);
					telemetry.pid = PID_Get_Actuation(&PidHandler);

					if (telemetry_enabled){
						osMessagePut(P2TelemetryQueueHandle, (uint32_t) &telemetry, 0);
					}



				}else{
					printf("P2 - CRC Error\r\n");

					HAL_GPIO_WritePin(Alive_GPIO_Port, Alive_Pin, MOTOR_OFF);

					last_motor_state = MOTOR_OFF;
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

	float gyroRawZ, gyroZ;


	yaw.angle.f[0] = 0.0;
	yaw_new.angle.f[0] = 0.0;

	long elapsedTime, currentTime, previousTime;
	MPU_Data *mpu_data;

	osEvent event = osMessageGet(MpuDataQueueHandle, osWaitForever);
			if (event.status == osEventMessage){
				mpu_data = ((MPU_Data *) event.value.v);

				printf("MPU data received");
				printf("%f,%f,%f,%d\n", mpu_data->mean,mpu_data->stdev,mpu_data->variance,mpu_data->FS_Mult_Factor);
			} else {
				printf("Error receiving mpu data\n");
			}

	//uint32_t crc;

	/* Infinite loop */
	for(;;)
	{


		if(osSemaphoreWait(CountingSemaphoreHandle, osWaitForever) == osOK){
			if(osSemaphoreWait(CountingSemaphoreHandle, osWaitForever)== osOK){


				gyroRawZ = MPU6050_Read_Gyro(mpu_data);

				printf("GYRO z %f\n", gyroRawZ);
				previousTime = currentTime;
				currentTime = xTaskGetTickCount();
				elapsedTime = (currentTime - previousTime) / 1000;   //  (/1000) =>  ms->s

				gyroZ = (gyroRawZ - mpu_data->mean);
				yaw_new.angle.f[0] = yaw.angle.f[0] + gyroZ * elapsedTime;


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
			printf("[DEBUG] P1_pid=%f;  P2_pid=%f | P1_angle=%f, P2_angle=%f \r\n", debug_PID_P1, debug_PID_P2, debug_f1,debug_f2);
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

	mission_data mission_data;
	mission_data.timer = -5;
	mission_data.state = 0;

	MPU_Data *mpu_data;


	object_pool_id = osPoolCreate(osPool(object_pool));
	mpu_data = (MPU_Data *) osPoolAlloc(object_pool_id);




	int in_powered_ascent = 0;

	uint8_t receiveBuf[1];

	for(;;){



		switch(mission_data.state){

		//CASE 0: Inizializzo l'mpu

		case 0:
			mission_data.state=0;

			//first loop initialization



			if (MPU6050_Init(mpu_data,3) == MPU_OK){



				//transmit mpu init success
				printf("[MPU6050] init DONE!\n");
				mission_data.state = 1;
				break;


				//TODO inizializza l'angolo del servo a 0 (invia ai processi quest'informazione)


			} else {

				printf("[MPU6050] init Failed!\r\n");
				mission_data.state = -1;
				break;
			}




			//CASE 1: aspetto il comando di via, di ricalibrazione o di abort


		case 1:
			mission_data.is_master = is_Master;



			get_I2C_confermation( (uint8_t *)"[MPU] Wait for command", receiveBuf);



			if ( receiveBuf[0] == 1){
				printf("command 1: re-calibration\n");
				MPU6050_Calculate_IMU_Error(mpu_data, 2);

				printf("Gz: %f Gz_error: %f\r\n", mpu_data->mean, mpu_data->variance);
				printf("[OS] Calibration done\r\n");
				receiveBuf[0] = 0;

				//HAL_I2C_Slave_Transmit_DMA(&hi2c3, (uint8_t *)"[MPU] Calibration done", (uint16_t) 64);

				break;

			}



			if ( receiveBuf[0] == 2){
				printf("command 2: starting mission countdown\n");

				mission_data.state = 2;
				break;

			}



			if (receiveBuf[0] == -1){
				printf("command -1: ABORT\n");

				mission_data.state = -1;
				break;

			}



			//CASE 2 - Avvio il countdown
			//TODO implementare l'i2c non bloccante oppure un thread a parte per le comunicazioni i2c

		case 2:



			//mission_data.state=2;

			//se arriva un messaggio i2c -1 devo abortire

			//start MISSION timer

			HAL_TIM_Base_Start_IT(&htim7);
			if(telemetry_enabled)
				osThreadResume(TelemetryThreadHandle);

			mission_data.state = 3;
			break;



		case 3:


			//mission_data.state=3;
			//wait 1 second timer

			if(osSemaphoreWait(MissionTimerSemHandle, osWaitForever) == HAL_OK){


				mission_data.timer ++;
				printf("Mission timer %d\n",mission_data.timer);

				if (mission_data.timer == 0 && !in_powered_ascent){

					//set alive to true
					HAL_GPIO_WritePin(Alive_GPIO_Port, Alive_Pin, MOTOR_ON);



					printf("[OS] MPU is MASTER - ");

					osThreadResume(SensorReadHandle);
					osThreadResume(ThreadP1Handle);
					osThreadResume(ThreadP2Handle);
					osThreadResume(DebugThreadHandle);

					int status = osMessagePut(MpuDataQueueHandle, (uint32_t) mpu_data, osWaitForever);


					if(status == HAL_OK)
						printf("Mpu data sent to sensor read func\n");
					else{
						printf("Mpu data send ERROR\n");
					}



					new_mission_data new_m_data;
					new_m_data.desired_angle = 180.0;
					new_m_data.degrees_per_second = 6.0;

					//send new mission data to threads
					osMessagePut(MewMissionP1QueueHandle, (uint32_t) &new_m_data, osWaitForever);
					osMessagePut(MewMissionP2QueueHandle, (uint32_t) &new_m_data, osWaitForever);

					if(is_Master){

						if (HAL_TIM_Base_Start_IT(&htim2) == HAL_OK){ // custom: init timer for timer interrupt
							printf(" Stating sampling timer...\r\n\n");

						} else {
							printf("[OS] Timer failed to start - aborting.\r\n");
							mission_data.state = -1;
							break;
						}

					}

					in_powered_ascent = 1;
				}



				if (mission_data.timer == 30){

					mission_data.timer ++;

					//set alive to false
					HAL_GPIO_WritePin(Alive_GPIO_Port, Alive_Pin, MOTOR_OFF);

					// stop interrupt timer esterno

					mission_data.state = 4;

					break;
				}


				if (telemetry_enabled){
					//osSemaphoreRelease(TelemetryThreadSemHandle);
					//osMessagePut(MissionTimerQueueHandle, (uint32_t) & mission_data, 1);
				}



				debug_mission_timer = mission_data.timer;


			}


			break;


		case 4:
			//mission_data.state=4;
			HAL_GPIO_WritePin(Alive_GPIO_Port, Alive_Pin, MOTOR_OFF);

			printf("[state 4 = COASTING]\n");
			HAL_I2C_Slave_Transmit_DMA(&hi2c3, (uint8_t *)"[MPU] Coasting", (uint16_t) 64);

			osDelay(5000);

			if (is_Master)
				HAL_TIM_Base_Stop_IT(&htim2);

			osThreadSuspend(SensorReadHandle);
			osThreadSuspend(DebugThreadHandle);
			osThreadSuspend(ThreadP1Handle);
			osThreadSuspend(ThreadP2Handle);
			osThreadSuspend(TelemetryThreadHandle);



			// TODO get angles from P1,P2 and if needed command a trajectory adjustment

			break;


		case -1:
			mission_data.state=-1;
			printf("[state = ABORT]\n");

			osThreadSuspendAll();

			break;
		}



		osDelay(1);
	}


	//Gyro_Z_RAW = 0;
	//					HAL_GPIO_WritePin(Alive_backup_GPIO_Port, Alive_backup_Pin, 1);
	//kill this thread
	//osThreadTerminate(InitTaskHandle);

  /* USER CODE END InitTaskFunc */
}

/* USER CODE BEGIN Header_TelemetryThreadFunc */
/**
 * @brief Function implementing the TelemetryThread thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_TelemetryThreadFunc */
void TelemetryThreadFunc(void const * argument)
{
  /* USER CODE BEGIN TelemetryThreadFunc */


	//TODO: get mission timer without a queue

	telemetry_data telemetry_p1, telemetry_p2;
	char msg[64];
	mission_data m_data;

	/* Infinite loop */
	for(;;)
	{

		//get telemetry from P1 and P2
		if (telemetry_enabled){



			telemetry_p1 = telemetry_queue_receive(P1TelemetryQueueHandle);
			telemetry_p2 = telemetry_queue_receive(P2TelemetryQueueHandle);

			//get mission timer



			osEvent event = osMessageGet(MissionTimerQueueHandle, 0);
			if (event.status == osEventMessage){

				m_data = *((mission_data *) event.value.v);
			}


			// assemble telemetry message

			snprintf(msg, sizeof(msg), "%d;%d;%d;%0.2f;%0.2f,%0.0f;%0.2f,%0.0f",

					is_Master,
					m_data.state,
					//m_data.timer,
					debug_mission_timer,
					debug_current_angle_P1,
					telemetry_p1.angle,
					telemetry_p1.pid,
					telemetry_p2.angle,
					telemetry_p2.pid

			);




			//HAL_I2C_Slave_Receive(&hi2c3, (uint8_t *) msgrcv, 1, HAL_MAX_DELAY);

			HAL_I2C_Slave_Transmit_DMA(&hi2c3, (uint8_t *)msg, (uint16_t) 64);



			//HAL_I2C_Slave_Transmit_DMA(hi2c, pData, Size);


			if (debug_active){

				//
				//				printf("I2C message: ");
				//				printf(msg);
				//				printf("\n");
				//				if(status == HAL_OK){
				//					printf("Telemetry SENT!\n");
				//					telemetry_sent ++;
				//				}
				//
				//				if(status == HAL_BUSY){
				//					printf("Telemetry busy!\n");
				//					hal_busy++;
				//				}
				//
				//				if(status == HAL_ERROR){
				//					printf("Telemetry sent=%d, hal_busy=%d\n", telemetry_sent, hal_busy);
				//				}
			}


			//printf("Telemetry send ERROR!\n");


		}
		osDelay(1);
	}
  /* USER CODE END TelemetryThreadFunc */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
