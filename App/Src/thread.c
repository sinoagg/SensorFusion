/**
  ******************************************************************************
  * File Name          : thread.c
  * Description        : Code for freertos applications
  ******************************************************************************
**/
/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */
#include "thread.h"
#include "ADAS.h"
#include "ARS408.h"
#include "can.h"
#include "cmd.h"
#include "dac.h"
#include "EMRR.h"
#include "main.h"
#include "usart.h"

#include "tim.h"
#include "vehicle.h"
#include "aebs.h"
#include "user_adas.h"
#include "user_can.h"
#include "user_radar.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/

/* USER CODE BEGIN Variables */

osThreadId defaultTaskHandle;
osThreadId PrepareCANDataHandle;
osThreadId radarCalcHandle;
#ifdef ADAS_COMM
osThreadId ADAS_CommTaskHandle;
#endif
osThreadId CAN_XBR_TaskHandle;
osThreadId CAN_AEBS1_TaskHandle;

osSemaphoreId bSemPrepareCANDataSigHandle;
osSemaphoreId bSemRadarCalcSigHandle;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const *argument);	 //always running
#ifdef ADAS_COMM
void StartADAS_CommTask(void const *argument); //always running
#endif
void StartPrepareCANDataTask(void const *argument);
void StartRadarCalcTask(void const *argument);
void StartCAN_XBR_TX_Task(void const *argument);
void StartCAN_AEBS1_TX_Task(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

uint8_t minRadarDistFlag = 0; //最近目标状态位
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* definition and creation of bSemPrepareCANDataSig */

	osSemaphoreDef(bSemPrepareCANDataSig);
	bSemPrepareCANDataSigHandle = osSemaphoreCreate(osSemaphore(bSemPrepareCANDataSig), 1);
	/* definition and creation of bSemRadarCalcSig */
	osSemaphoreDef(bSemRadarCalcSig);
	bSemRadarCalcSigHandle = osSemaphoreCreate(osSemaphore(bSemRadarCalcSig), 1);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	osSemaphoreWait(bSemPrepareCANDataSigHandle, osWaitForever); //read once to disable
	osSemaphoreWait(bSemRadarCalcSigHandle, osWaitForever);			 //read once to disable
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

#ifdef ADAS_COMM
	osThreadDef(ADAS_CommTask, StartADAS_CommTask, osPriorityNormal, 0, 128);
	ADAS_CommTaskHandle = osThreadCreate(osThread(ADAS_CommTask), NULL);
#endif
	/* definition and creation of PrepareCANData */
	osThreadDef(PrepareCANData, StartPrepareCANDataTask, osPriorityNormal, 0, 128);
	PrepareCANDataHandle = osThreadCreate(osThread(PrepareCANData), NULL);

	/* definition and creation of RadarCalc */
	osThreadDef(RadarCalc, StartRadarCalcTask, osPriorityNormal, 0, 128);
	radarCalcHandle = osThreadCreate(osThread(RadarCalc), NULL);
	
	osThreadDef(CAN_XBR_TX, StartCAN_XBR_TX_Task, osPriorityNormal, 0, 128);
	osThreadId CAN_XBR_TaskHandle = osThreadCreate(osThread(CAN_XBR_TX), NULL);
	
	osThreadDef(CAN_AEBS1_TX, StartCAN_AEBS1_TX_Task, osPriorityNormal, 0, 128);
	osThreadId CAN_AEBS1_TaskHandle = osThreadCreate(osThread(CAN_AEBS1_TX), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */
#if ADAS_COMM
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //ADAS
#endif

#if RADAR_TYPE == ARS408
	ARS_Init(&hcan3);
#elif RADAR_TYPE == EMRR
	EMRR_Init(&hcan3);
#endif

#if CAN_READ_VEHICLE
	CAN1_Init(&hcan2);
	Vehicle_CAN_Init(&hcan2);
#endif
	DisableAEBS(&vAEBS_Status);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
}

/* StartDefaultTask function */
void StartDefaultTask(void const *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	
	/* Infinite loop */
	for (;;)
	{
		if (AEB_CAN_TxReady == 1)
		{
			AEB_CAN_TxReady = 0;
			#if VEHICLE_MODEL == KINGLONG
			HAL_CAN_AddTxMessage(&hcan2, &AEB_CAN_TxHeader, AEB_CAN_TxBuf, &AEB_CAN_TxMailBox);
			#endif
//			if(XBR_Flag == 0)
//			{
//				XBR_Flag = 1;
//				XBRCalc(&hcan2, 0.5f, RadarObject.VrelLong, RadarObject.MinRangeLong);
//			}
//			else
//			{
//				XBR_Flag = 0;
//				XBRCalc(&hcan2, 3.5f, RadarObject.VrelLong, RadarObject.MinRangeLong);
//			}

		}			
		osDelay(50);
	}
	/* USER CODE END StartDefaultTask */
}

void StartADAS_CommTask(void const *argument)
{
	for (;;)
	{
		if (ADASRxComplete == 1)
		{
			ADASRxComplete = 0;
			//DispADASData(ADASRxBuf, ADASDispBuf, RadarObject.MinRangeLong, RadarObject.VrelLong, TimetoCrash_g);//发送给显示器
			//HAL_UART_Transmit(&huart2, ADASDispBuf, 32, 100);//transmit ADAS data to screen
			CalADASData(&ADAS_dev, ADASHexBuf);
			LED_ADAS_TOGGLE();
			#ifdef LDW
			if (ADAS_dev.LDW_warning == 0x01 || ADAS_dev.LDW_warning == 0x02) //Lane departure warning(left/right)
			{
				osSemaphoreRelease(bSemPrepareCANDataSigHandle);
			}
			#endif
		}
		osDelay(10);
	}
}

/* StartPrepareCANDataTask function */
void StartPrepareCANDataTask(void const *argument)
{
	/* USER CODE BEGIN StartPrepareCANDataTask */
	/* Infinite loop */
	uint8_t tempTTC;
	for (;;)
	{
		osSemaphoreWait(bSemPrepareCANDataSigHandle, osWaitForever);
		//Step3.向整车CAN发AEBS报警数据准备
		AEB_CAN_TxBuf[0] = crashWarningLv; //5, 6, 7对应金龙车三个等级
		if (TimetoCrash_g < 0)
			tempTTC = 0;
		else if (TimetoCrash_g > 9.9f)
			tempTTC = 9.9f * 20;
		else
			tempTTC = TimetoCrash_g * 20;
		AEB_CAN_TxBuf[2] = tempTTC;
		AEB_CAN_TxBuf[3] = (uint8_t)RadarObject.MinRangeLong;
		AEB_CAN_TxBuf[4] = (uint8_t)(RadarObject.VrelLong * 3.6f);
		AEB_CAN_TxBuf[5] = (uint8_t)vehicle.speed;
		AEB_CAN_TxBuf[6] = ADAS_dev.crash_level;
		//定时中断发送//HAL_CAN_AddTxMessage(&hcan2, &CAN_TxAEBHeader, AEB_CAN_TxBuf, &AEB_CAN_TxMailBox);

		osDelay(50);
	}
	/* USER CODE END StartPrepareCANDataTask */
}

void StartCAN_XBR_TX_Task(void const *argument)
{
	for (;;)
	{
		#if ADAS_COMM
		if (ADAS_dev.crash_level > 0)
		{
			if(crashWarningLv==WARNING_NONE || crashWarningLv==WARNING_LOW)
			{
				osDelay(50);
				XBRCalc(&hcan2, TimetoCrash_g, 0, RadarObject.VrelLong, RadarObject.MinRangeLong);
			}
			else
			{
				osDelay(20);
				XBRCalc(&hcan2, TimetoCrash_g, 1, RadarObject.VrelLong, RadarObject.MinRangeLong);
			}
		}
		else if(vAEBS_Status.AEBStimes >3 && RadarObject.MinRangeLong < 4)
		{
			if(crashWarningLv==WARNING_NONE || crashWarningLv==WARNING_LOW)
			{
				osDelay(50);
				XBRCalc(&hcan2, TimetoCrash_g, 0, RadarObject.VrelLong, RadarObject.MinRangeLong);
			}
			else
			{
				osDelay(20);
				XBRCalc(&hcan2, TimetoCrash_g, 1, RadarObject.VrelLong, RadarObject.MinRangeLong);
			}
		}
		#else
		if(crashWarningLv==WARNING_NONE || crashWarningLv==WARNING_LOW)
		{
			osDelay(50);
			XBRCalc(&hcan2, TimetoCrash_g, 0, RadarObject.VrelLong, RadarObject.MinRangeLong);
		}
		else
		{
			osDelay(20);
			XBRCalc(&hcan2, TimetoCrash_g, 1, RadarObject.VrelLong, RadarObject.MinRangeLong);
		}
		#endif
		#if ADAS_COMM
		else
		{
			osDelay(50);
			XBRCalc(&hcan2, TimetoCrash_g, 0, RadarObject.VrelLong, RadarObject.MinRangeLong);
		}
		#endif

	}
}

void StartCAN_AEBS1_TX_Task(void const *argument)
{
	for (;;)
	{
		if(crashWarningLv==WARNING_NONE)
		{
			PrePareAEBS1Data(&hcan2, BRAKE_SYS_RDY, WARNING_NONE, OBJECT_NOT_DETECTED, 20, &RadarObject);
		}
		else if(crashWarningLv==WARNING_LOW)
		{
			PrePareAEBS1Data(&hcan2, COLLISION_WARNING_ACTIVE, WARNING_LOW, OBJECT_DETECTED, TimetoCrash_g, &RadarObject);
		}
		else if(crashWarningLv==WARNING_MID)
		{
			PrePareAEBS1Data(&hcan2, BRAKE_SYS_ON, WARNING_MID, OBJECT_DETECTED, TimetoCrash_g, &RadarObject);
		}
		else if(crashWarningLv==WARNING_HIGH)
		{
			PrePareAEBS1Data(&hcan2, BRAKE_SYS_EMER, WARNING_HIGH, OBJECT_DETECTED, TimetoCrash_g, &RadarObject);
		}
		osDelay(50);
	}
}

/* StartRadarCalcTask function */
void StartRadarCalcTask(void const *argument)
{
	/* USER CODE BEGIN StartRadarCalcTask */
	/* Infinite loop */
	for (;;)
	{
		osSemaphoreWait(bSemRadarCalcSigHandle, osWaitForever);

#if RADAR_TYPE == ARS408
		uint8_t AEBS_Deal = 0;
		uint8_t objectInLane = 0;
		MW_RadarObjStatus RadarObjStatus;
		if (RadarCAN_RxHeader.StdId == 0x60A) //read all messages before 60B, start calculate
		{
			minRadarDistFlag = 1; //下一个60B传回是距离最近的目标
			ARS_GetRadarObjStatus(RadarCANRxBuf, &RadarObjStatus);
			if (RadarObjStatus.Obj_NofObjects == 0) //目标数量为0
			{
				crashWarningLv = WARNING_NONE;
				TimetoCrash_g = 20;							//适配CAN线
				RadarObject.MinRangeLong = 255; //适配CAN线
				StopBuzzer(&vAEBS_Status);
				DisableAEBS(&vAEBS_Status);
			}
		}
		else if (RadarCAN_RxHeader.StdId == 0x60B)
		{
			AEBS_Deal = 0;
			if (vehicle.speed >= VEHICLE_SPEED_THRESHOLD && minRadarDistFlag!=0) //车速报警阈值,如果此值太大，制动后期低速时不再制动 并且是最近目标
			{
					minRadarDistFlag = 0; //最近目标状态位清零
					//step1.发送到CAN1分析
					uint16_t dist = 0;
					uint8_t temp;
					dist = (uint16_t)(((*(RadarCANRxBuf + 1)) << 5) | ((*(RadarCANRxBuf + 2)) >> 3));
					dist -= ((vehicle.speed / 22) - 0.0) * 5;
					*(RadarCANRxBuf + 1) = (dist >> 5);
					temp = ((dist << 3) & 0xF8);
					*(RadarCANRxBuf + 2) &= 0x07;
					*(RadarCANRxBuf + 2) |= temp;
					//暂时不用往DBC发数据
					//HAL_CAN_AddTxMessage(&hcan1, &DBC_CAN_TxHeader, RadarCANRxBuf, &DBC_CAN_TxMailBox);

					//step2.解析大陆雷达数据
					ARS_GetRadarObjGeneral(RadarCANRxBuf, RadarGeneral); //get closet obj data
					uint16_t MinRangeLong = RadarGeneral[0].Obj_DistLong;
					uint16_t MinRangeLat  = RadarGeneral[0].Obj_DistLat;
					uint32_t relSpeedLong = RadarGeneral[0].Obj_VrelLong;
					uint32_t relSpeedLat  = RadarGeneral[0].Obj_VrelLat;
					float invadeLaneTime = 0.0f;

					RadarObject.MinRangeLat  = 0.2f * (float)MinRangeLat  - 204.6f;	//get real range(latitude)
					RadarObject.VrelLat = 0.25f * (float)relSpeedLat - 64;					//get real relative latitude  speed
					if(RadarObject.VrelLat != 0)
						invadeLaneTime = - RadarObject.MinRangeLat / RadarObject.VrelLat;
					
					if(RadarObject.MinRangeLat < LANEWIDTH && RadarObject.MinRangeLat > -LANEWIDTH)
						objectInLane = 1;
					else
						objectInLane = 0;
					//在本车道内 或 在旁边车道且在靠近本车道
					if(objectInLane || (!objectInLane && (invadeLaneTime < INVADE_LANE_TIME_THRESHOLD) && (invadeLaneTime > 0)))
					{
						RadarObject.MinRangeLong = 0.2f * (float)MinRangeLong - 500; 		//get real range(longitude)
						if ((RadarObject.MinRangeLong) < LIMIT_RANGE && (RadarObject.MinRangeLong > 0) && MinRangeLong != 0) //calculate when dist is near enough
						{
							RadarObject.VrelLong = 0.25f * (float)relSpeedLong - 128; //get real relative longitude speed
							if (RadarObject.VrelLong < 0)
							{
								TimetoCrash_g = -(float)RadarObject.MinRangeLong / RadarObject.VrelLong; //relative Velocity is minus
								if (TimetoCrash_g < HIGH_WARNING_TIME)
								{
									AEBS_Deal = 1; //处理了报警
									crashWarningLv = WARNING_HIGH;
									#if ADAS_COMM
									if (ADAS_dev.crash_level > 0)
									{
										StartBuzzer(&vAEBS_Status, WARNING_HIGH);
										EnableAEBS(TimetoCrash_g, WARNING_HIGH);
										vAEBS_Status.AEBStimes += 1;
										vAEBS_Status.onlyRadarTimes = 20;
									}
									else if(vAEBS_Status.onlyRadarTimes > 0)
									{
										StartBuzzer(&vAEBS_Status, WARNING_HIGH);
										EnableAEBS(TimetoCrash_g, WARNING_HIGH);
									}
									else if(vAEBS_Status.AEBStimes > 3 && RadarObject.MinRangeLong < 4)
									{
										StartBuzzer(&vAEBS_Status, WARNING_HIGH);
										EnableAEBS(TimetoCrash_g, WARNING_HIGH);
									}
									#else
										StartBuzzer(&vAEBS_Status, WARNING_HIGH);
										EnableAEBS(TimetoCrash_g, WARNING_HIGH);
									#endif
								}
								else if (TimetoCrash_g < MID_WARNING_TIME)
								{
									AEBS_Deal = 1; //处理了报警
									crashWarningLv = WARNING_MID;
									#if ADAS_COMM
									if (ADAS_dev.crash_level > 0)
									{
										StartBuzzer(&vAEBS_Status, WARNING_MID);
										EnableAEBS(TimetoCrash_g, WARNING_MID);
										vAEBS_Status.AEBStimes += 1;
										vAEBS_Status.onlyRadarTimes = 20;
									}
									else if(vAEBS_Status.onlyRadarTimes > 0)
									{
										StartBuzzer(&vAEBS_Status, WARNING_MID);
										EnableAEBS(TimetoCrash_g, WARNING_MID);
									}
									else if(vAEBS_Status.AEBStimes > 3 && RadarObject.MinRangeLong < 4)
									{
										StartBuzzer(&vAEBS_Status, WARNING_MID);
										EnableAEBS(TimetoCrash_g, WARNING_MID);
									}
									#else
										StartBuzzer(&vAEBS_Status, WARNING_MID);
										EnableAEBS(TimetoCrash_g, WARNING_MID);
									#endif
									
								}
								else if (TimetoCrash_g < LOW_WARNING_TIME)
								{
									AEBS_Deal = 1; //处理了报警
									crashWarningLv = WARNING_LOW;
									#if ADAS_COMM
									if (ADAS_dev.crash_level > 0)
									{
										StartBuzzer(&vAEBS_Status, WARNING_LOW);
										DisableAEBS(&vAEBS_Status);
										vAEBS_Status.onlyRadarTimes = 20;
									}
									else if(vAEBS_Status.AEBStimes > 3 && RadarObject.MinRangeLong < 4)
									{
										StartBuzzer(&vAEBS_Status, WARNING_LOW);
										DisableAEBS(&vAEBS_Status);
									}
									#else
										StartBuzzer(&vAEBS_Status, WARNING_LOW);
										DisableAEBS(&vAEBS_Status);
									#endif
								}
							}
						}
					}
			}
			if (AEBS_Deal == 0) //没有处理报警
			{
				if(vAEBS_Status.AEBStimes > 3)
					vAEBS_Status.AEBStimes -= 10;
				if(vAEBS_Status.AEBStimes <= 3)
				{
					crashWarningLv = WARNING_NONE;
					TimetoCrash_g = 20;							//适配CAN线
					RadarObject.MinRangeLong = 255; //适配CAN线
					RadarObject.VrelLong = 0;
					StopBuzzer(&vAEBS_Status);
					DisableAEBS(&vAEBS_Status);
				}
			}
			osSemaphoreRelease(bSemPrepareCANDataSigHandle); //发送雷达准备数据信号量
		}
#elif RADAR_TYPE == EMRR
		LED_RADAR_TOGGLE();
		RadarObject.MinRangeLong = EMRRGeneral_Closet.trackRange; 
		RadarObject.VrelLong = EMRRGeneral_Closet.trackSpeed;
		//if (!Turning_Flag || (Turning_Flag && Turning_Collision))
		{
			if (RadarObject.MinRangeLong < LIMIT_RANGE && RadarObject.MinRangeLong > 0 && RadarObject.VrelLong < 0)
			{
				TimetoCrash_g = -RadarObject.MinRangeLong / RadarObject.VrelLong;
				if (TimetoCrash_g < HIGH_WARNING_TIME)
				{
					crashWarningLv = WARNING_HIGH;
					osSemaphoreRelease(bSemPrepareCANDataSigHandle);
				}
				else if (TimetoCrash_g < LOW_WARNING_TIME)
				{
					crashWarningLv = WARNING_LOW;
					//osSemaphoreRelease(bSemPrepareCANDataSigHandle);
				}
				else
					crashWarningLv = WARNING_NONE;
			}
		}
#endif

		osDelay(10);
	}
	/* USER CODE END StartRadarCalcTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
