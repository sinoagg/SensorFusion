/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "ADAS.h"
#include "adc.h"
#include "ARS408.h"
#include "can.h"
#include "cmd.h"
#include "EMRR.h"
#include "main.h"
#include "MPU6050.h"
#include "usart.h"
#include "switch.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId RadarCommHandle;
osThreadId ADASCommHandle;
osThreadId SoundWarningHandle;
osThreadId GyroCommHandle;
osThreadId CANSpeedReadHandle;
osThreadId RadarCalcHandle;
osThreadId UART1RxHandle;
osThreadId RadarDataTxHandle;
osSemaphoreId bSemRadarCANRxSigHandle;
osSemaphoreId bSemADASRxSigHandle;
osSemaphoreId bSemSoundWarningSigHandle;
osSemaphoreId bSemGyroCommSigHandle;
osSemaphoreId bSemSpeedRxSigHandle;
osSemaphoreId bSemRadarCalcSigHandle;
osSemaphoreId bSemUART1RxSigHandle;

/* USER CODE BEGIN Variables */
#define YAWRATE_LIMIT 327.68f

extern CAN_RxHeaderTypeDef RadarCANRxHeader;
extern ADAS_HandleTypeDef ADAS_dev;
extern uint8_t ADASRxBuf[];
extern uint8_t ADASDispBuf[];
extern uint8_t RadarCANRxBuf[];
extern uint8_t CrashWarningLv;
extern uint8_t VehicleCANRxBuf[];
extern uint8_t YawCANRxBuf[];
extern uint8_t VehicleSpeed_g;
extern uint8_t Vehicle_CAN_Flag;
extern uint8_t CmdRxBuf[];
extern uint8_t CmdRadarDataTxBuf[];
extern uint8_t ADASRxComplete;
extern uint16_t ADC_ConvertedValue[];
uint8_t Turning_Collision = 0;
uint8_t Turning_Flag = 0;
//ARS408
extern MW_RadarObjStatus RadarObjStatus;
extern MW_RadarGeneral RadarGeneral[16];
//EMRR
extern EMRR_RadarGeneral aEMRRGeneral[];
extern EMRR_RadarGeneral EMRRGeneral_Closet;
extern float YawRate_g;
extern float MinRangeLong_g;
extern float VrelLong_g;
extern float TimetoCrash_g;
extern __IO float ADC_ConvertedValueF[2];

extern uint8_t RadarTimes;
extern uint8_t RadarYawTimes;

struct
{
  uint8_t brake;
  uint8_t right_turn;
  uint8_t left_turn;
}VehicleSwitch;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartRadarCommTask(void const * argument);
void StartADASCommTask(void const * argument);
void StartSoundWarningTask(void const * argument);
void StartGyroCommTask(void const * argument);
void StartCANSpeedReadTask(void const * argument);
void StartRadarCalcTask(void const * argument);
void StartUART1RxTask(void const * argument);
void StartRadarDataTxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t DBC_Init(CAN_HandleTypeDef *hcan);
extern uint8_t Gyro_CAN_Init(CAN_HandleTypeDef *hcan);
extern uint8_t Vehicle_CAN_Init(CAN_HandleTypeDef *hcan);
extern uint8_t DBC_SendDist(CAN_HandleTypeDef *hcan, float Dist);
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
  /* definition and creation of bSemRadarCANRxSig */
  osSemaphoreDef(bSemRadarCANRxSig);
  bSemRadarCANRxSigHandle = osSemaphoreCreate(osSemaphore(bSemRadarCANRxSig), 1);

  /* definition and creation of bSemADASRxSig */
  osSemaphoreDef(bSemADASRxSig);
  bSemADASRxSigHandle = osSemaphoreCreate(osSemaphore(bSemADASRxSig), 1);

  /* definition and creation of bSemSoundWarningSig */
  osSemaphoreDef(bSemSoundWarningSig);
  bSemSoundWarningSigHandle = osSemaphoreCreate(osSemaphore(bSemSoundWarningSig), 1);

  /* definition and creation of bSemGyroCommSig */
  osSemaphoreDef(bSemGyroCommSig);
  bSemGyroCommSigHandle = osSemaphoreCreate(osSemaphore(bSemGyroCommSig), 1);

  /* definition and creation of bSemSpeedRxSig */
  osSemaphoreDef(bSemSpeedRxSig);
  bSemSpeedRxSigHandle = osSemaphoreCreate(osSemaphore(bSemSpeedRxSig), 1);

  /* definition and creation of bSemRadarCalcSig */
  osSemaphoreDef(bSemRadarCalcSig);
  bSemRadarCalcSigHandle = osSemaphoreCreate(osSemaphore(bSemRadarCalcSig), 1);

  /* definition and creation of bSemUART1RxSig */
  osSemaphoreDef(bSemUART1RxSig);
  bSemUART1RxSigHandle = osSemaphoreCreate(osSemaphore(bSemUART1RxSig), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	osSemaphoreWait(bSemRadarCANRxSigHandle, osWaitForever);		//read once to disable
  #if ADAS_COMM
  osSemaphoreWait(bSemADASRxSigHandle, osWaitForever);				//read once to disable
  #endif
  osSemaphoreWait(bSemSoundWarningSigHandle, osWaitForever);	//read once to disable
  #if CAN_READ_VEHICLE
  osSemaphoreWait(bSemGyroCommSigHandle, osWaitForever);	    //read once to disable
  osSemaphoreWait(bSemSpeedRxSigHandle, osWaitForever);				//read once to disable
  #endif
  osSemaphoreWait(bSemRadarCalcSigHandle, osWaitForever);			//read once to disable
  #if RADAR_DATA_SEND
  osSemaphoreWait(bSemUART1RxSigHandle, osWaitForever);       //read once to disable
  #endif
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of RadarComm */
  osThreadDef(RadarComm, StartRadarCommTask, osPriorityNormal, 0, 128);
  RadarCommHandle = osThreadCreate(osThread(RadarComm), NULL);

  /* definition and creation of ADASComm */
  #if ADAS_COMM
  osThreadDef(ADASComm, StartADASCommTask, osPriorityBelowNormal, 0, 128);
  ADASCommHandle = osThreadCreate(osThread(ADASComm), NULL);
  #endif
  /* definition and creation of SoundWarning */
  osThreadDef(SoundWarning, StartSoundWarningTask, osPriorityIdle, 0, 64);
  SoundWarningHandle = osThreadCreate(osThread(SoundWarning), NULL);

  #if CAN_READ_VEHICLE
  /* definition and creation of GyroComm */
  osThreadDef(GyroComm, StartGyroCommTask, osPriorityIdle, 0, 128);
  GyroCommHandle = osThreadCreate(osThread(GyroComm), NULL);

  /* definition and creation of CANSpeedRead */
  osThreadDef(CANSpeedRead, StartCANSpeedReadTask, osPriorityIdle, 0, 128);
  CANSpeedReadHandle = osThreadCreate(osThread(CANSpeedRead), NULL);
  #endif

  /* definition and creation of RadarCalcTask */
  osThreadDef(RadarCalcTask, StartRadarCalcTask, osPriorityIdle, 0, 128);
  RadarCalcHandle = osThreadCreate(osThread(RadarCalcTask), NULL);

	#if RADAR_DATA_SEND
  /* definition and creation of UART1RxTask */
  osThreadDef(UART1RxTask, StartUART1RxTask, osPriorityIdle, 0, 64);
  UART1RxHandle = osThreadCreate(osThread(UART1RxTask), NULL);

  /* definition and creation of RadarDataTxTask */
  osThreadDef(RadarDataTxTask, StartRadarDataTxTask, osPriorityIdle, 0, 128);
  RadarDataTxHandle = osThreadCreate(osThread(RadarDataTxTask), NULL);
	
	osThreadSuspend(RadarDataTxHandle);		//suspend Radar data send task
	#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
	//hcan1~hcan3 init, start
  #if GYRO_CAN == 1
  Gyro_CAN_Init(&hcan1);
  #else
	DBC_Init(&hcan1);
  #endif
	//ARS408
	#if RADAR_TYPE
  ARS_Init(&hcan2);//hcan2 must use hcan1
	//EMRR
	#else
	EMRR_Init(&hcan2);
	#endif
	#if CAN_READ_VEHICLE
  Vehicle_CAN_Init(&hcan3);
  #endif
	
	#if ATM_READ
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_ConvertedValue, 2);
	#endif
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    #if RADAR_DATA_SEND
		HAL_UART_Receive_DMA(&huart1, CmdRxBuf, 4);//receive cmd from labview
		if(UART1RxComplete==1)
		{
			UART1RxComplete=0;
			osSemaphoreRelease(bSemUART1RxSigHandle);
		}
    #endif

    #if ADAS_COMM
    HAL_UART_Receive_DMA(&huart3, ADASRxBuf, 32);//receive ADAS warning
		if(ADASRxComplete==1)
		{
      ADASRxComplete=0;
      DispADASData(ADASRxBuf, ADASDispBuf, MinRangeLong_g, VrelLong_g, TimetoCrash_g);
			HAL_UART_Transmit(&huart5, ADASDispBuf, 32, 100);//transmit ADAS data to screen
      osSemaphoreRelease(bSemADASRxSigHandle);
		}
    #endif
		
    #if DBC_SEND
    DBC_SendDist(&hcan1, MinRangeLong_g);
    #endif
		
		#if ATM_READ
		/* Vref = 3.3(fullscale), 2^12=4096(scale),
       when input is 3.3V, the result is 4096 */    
    ADC_ConvertedValueF[0] =(double)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; 	// ADC_ConvertedValue only lowest 12bit
		ADC_ConvertedValueF[1] =(double)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096; 	// ADC_ConvertedValue only lowest 12bit
		#endif

		//HAL_GPIO_TogglePin(VALVE_FRONT_GPIO_Port, VALVE_FRONT_Pin|VALVE_REAR_Pin);

		osDelay(30);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartRadarCommTask function */
void StartRadarCommTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarCommTask */
  /* Infinite loop */
 	//ARS408
	#if RADAR_TYPE
	uint8_t minRadarDistFlag = 0;
	#endif
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(bSemRadarCANRxSigHandle, osWaitForever);
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		
    //ARS408
		#if RADAR_TYPE
		if(RadarCANRxHeader.StdId==0x60A)												//read all messages before 60B, start calculate
		{
			minRadarDistFlag = 1;
		}
		else if(RadarCANRxHeader.StdId==0x60B)									//0x60B, read distance & relVelocity
		{
			if(minRadarDistFlag)
			{
				minRadarDistFlag = 0;
				ARS_GetRadarObjGeneral(RadarCANRxBuf, RadarGeneral);//get closet obj data
				osSemaphoreRelease(bSemRadarCalcSigHandle);
				RadarTimes += 1;
			}
		}
		
    //EMRR
		#else
		EMRR_GetRaderObjCloset(RadarCANRxBuf, aEMRRGeneral, &EMRRGeneral_Closet);
		if(EMRRGeneral_Closet.trackRange != 0)
			osSemaphoreRelease(bSemRadarCalcSigHandle);
		#endif
		osDelay(1);
  }
  /* USER CODE END StartRadarCommTask */
}

/* StartADASCommTask function */
void StartADASCommTask(void const * argument)
{
  /* USER CODE BEGIN StartADASCommTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemADASRxSigHandle, osWaitForever);
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		CalADASData(&ADAS_dev, ADASRxBuf);
		if(ADAS_dev.LDW_warning==0x01 || ADAS_dev.LDW_warning == 0x02)//Lane departure warning(left/right)
		{
      //start warning task
			osSemaphoreRelease(bSemSoundWarningSigHandle);
    }
    osDelay(10);
  }
  /* USER CODE END StartADASCommTask */
}

/* StartSoundWarningTask function */
void StartSoundWarningTask(void const * argument)
{
  /* USER CODE BEGIN StartSoundWarningTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemSoundWarningSigHandle, osWaitForever);
		if(VehicleSpeed_g > 20)
		{
			switch(CrashWarningLv)				//Forward collision warning
			{
				case WARNING_HIGH:
					#if ADAS_COMM
					if(0 != ADAS_dev.crash_level)
					{
					#endif
						HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET);
						//HAL_GPIO_WritePin(VALVE_FRONT_GPIO_Port, VALVE_FRONT_Pin, GPIO_PIN_RESET);
						//WTN6_Broadcast(BELL_BB_500MS);
						osDelay(200);
						HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
						//HAL_GPIO_WritePin(VALVE_FRONT_GPIO_Port, VALVE_FRONT_Pin, GPIO_PIN_SET);
					#if ADAS_COMM
					}
					#endif
					break;
				case WARNING_LOW:
					#if ADAS_COMM
					if(0 != ADAS_dev.crash_level)
					{
					#endif
						HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET);
						//WTN6_Broadcast(BELL_BB_1000MS);
						osDelay(500);
						HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
					#if ADAS_COMM
					}
					#endif
					break;
				default:
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
					break;
			}

			#if ADAS_COMM
			switch(ADAS_dev.LDW_warning)	//Lane departure warning
			{
				case 0x01:	//left
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET);
					//WTN6_Broadcast(BELL_BB_500MS);
					osDelay(2000);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
					break;
				case 0x02:	//right
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_RESET);
					//WTN6_Broadcast(BELL_BB_1000MS);
					osDelay(2000);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
					break;
				default:
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
					break;
			}
			#endif
		}
		osDelay(10);
  }
  /* USER CODE END StartSoundWarningTask */
}

/* StartGyroCommTask function */
void StartGyroCommTask(void const * argument)
{
  /* USER CODE BEGIN StartGyroCommTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemGyroCommSigHandle, osWaitForever);
    YawRate_g =  MPU_GetYawRate(YawCANRxBuf);
		if(YawRate_g < 0)		//clockwise
		{
			YawRate_g = (YawRate_g < -YAWRATE_LIMIT) ? -YAWRATE_LIMIT : YawRate_g;
		}
		else
			YawRate_g = (YawRate_g > YAWRATE_LIMIT) ? YAWRATE_LIMIT: YawRate_g;
		//ARS408
		#if RADAR_TYPE
			//ARS_SendVehicleYaw(&hcan2, YawRate_g);  //send VehicleYaw to Radar-ARS408
			//osDelay(2);
			//ARS_SendVehicleSpeed(&hcan2, VehicleSpeed_g);
			RadarYawTimes += 1;
		
		
			if(YawRate_g > 5 || YawRate_g < -5)
      {
        Turning_Flag = 1;
		  	Turning_Collision = ARS_CalcTurn(RadarGeneral, YawRate_g, VehicleSpeed_g);
      }
			else
			{
				Turning_Flag = 0;
				Turning_Collision = 0;
			}
		
		//EMRR
		#else
      if(YawRate_g > 5 || YawRate_g < -5)
      {
        Turning_Flag = 1;
		  	Turning_Collision = EMRR_CalcTurn(&EMRRGeneral_Closet, YawRate_g, VehicleSpeed_g);
      }
			else
			{
				Turning_Flag = 0;
				Turning_Collision = 0;
			}
    #endif
		osDelay(100);
  }
  /* USER CODE END StartGyroCommTask */
}

/* StartCANSpeedReadTask function */
void StartCANSpeedReadTask(void const * argument)
{
  /* USER CODE BEGIN StartCANSpeedReadTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemSpeedRxSigHandle, osWaitForever);
    #if VEHICLE_MODEL == 2    //BYD

    #elif VEHICLE_MODEL == 1  //YUTONG
    if(0xD1 == VehicleCANRxBuf[0] && 0xD1 == VehicleCANRxBuf[2])
    {
      VehicleSpeed_g = VehicleCANRxBuf[1];	//vehicle speed in hex,km/h
			//ARS408
			#if RADAR_TYPE
			ARS_SendVehicleSpeed(&hcan2, VehicleSpeed_g);	//send VehicleSpeed to Radar
			//EMRR
			#else
			#endif
    }
    #else   //KINGLONG
    if(1 == Vehicle_CAN_Flag) //VehicleSpeed ID
    {
      VehicleSpeed_g = VehicleCANRxBuf[7];  	//vehicle speed in hex,km/h

			//ARS408
      #if RADAR_TYPE
      ARS_SendVehicleSpeed(&hcan2, VehicleSpeed_g);	//send VehicleSpeed to Radar
      //EMRR
      #else
      #endif
			
			Vehicle_CAN_Flag = 0;
    }
    else if(2 == Vehicle_CAN_Flag)	//VehicleSwitch ID
    {
      if(VehicleCANRxBuf[0] == 1)		//pack number == 1
      {
        VehicleSwitch.brake = VehicleCANRxBuf[2] & 0x3;
				VehicleSwitch.right_turn = (VehicleCANRxBuf[4] & 0xC) >> 2;
				VehicleSwitch.left_turn = (VehicleCANRxBuf[4] & 0x30) >> 4;
      }
			Vehicle_CAN_Flag = 0;
    }
    #endif
    
		osDelay(100);
  }
  /* USER CODE END StartCANSpeedReadTask */
}

/* StartRadarCalcTask function */
void StartRadarCalcTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarCalcTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemRadarCalcSigHandle, osWaitForever);
    //ARS408
		#if RADAR_TYPE
		uint16_t MinRange=255;
		uint32_t relSpeed=0;
		MinRange = RadarGeneral[0].Obj_DistLong;
		relSpeed = RadarGeneral[0].Obj_VrelLong;
		
		if(!Turning_Flag || (Turning_Flag && Turning_Collision))
		{
			if((0.2*MinRange-500) < LIMIT_RANGE && MinRange != 0)	//calculate when dist is near enough
			{
				VrelLong_g = 0.25 * relSpeed - 128;						//get real relative speed
				MinRangeLong_g = 0.2 * MinRange - 500;				//get real range(longitude)
				TimetoCrash_g = -(float)MinRangeLong_g/VrelLong_g;//relative Velocity is minus
				if(TimetoCrash_g < 2.1f && VrelLong_g < 0 && MinRangeLong_g > 0)
				{
					CrashWarningLv = WARNING_HIGH;
					osSemaphoreRelease(bSemSoundWarningSigHandle);
				}
				else if(TimetoCrash_g < 2.4f && VrelLong_g < 0 && MinRangeLong_g > 0)
				{
					CrashWarningLv = WARNING_LOW;
					osSemaphoreRelease(bSemSoundWarningSigHandle);
				}
				else
					CrashWarningLv = WARNING_NONE;
			}
		}

    //EMRR
		#else
		MinRangeLong_g = EMRRGeneral_Closet.trackRange;
    VrelLong_g = EMRRGeneral_Closet.trackSpeed;
    if(!Turning_Flag || (Turning_Flag && Turning_Collision))
    {
			/*if(MinRangeLong_g < 30)
			{
				 CrashWarningLv = WARNING_HIGH;
          osSemaphoreRelease(bSemSoundWarningSigHandle);
			}
      else */if(MinRangeLong_g < LIMIT_RANGE && MinRangeLong_g != 0 && VrelLong_g != 0)
      {
        TimetoCrash_g = - MinRangeLong_g / VrelLong_g;
        if(TimetoCrash_g < 3 && VrelLong_g < 0 && MinRangeLong_g > 0)
        {
          CrashWarningLv = WARNING_HIGH;
          osSemaphoreRelease(bSemSoundWarningSigHandle);
        }
        else if(TimetoCrash_g < 3.5f && VrelLong_g < 0 && MinRangeLong_g > 0)
        {
          CrashWarningLv = WARNING_LOW;
          osSemaphoreRelease(bSemSoundWarningSigHandle);
        }
        else
          CrashWarningLv = WARNING_NONE;
      }
    }
		#endif
		
		osDelay(2);
  }
  /* USER CODE END StartRadarCalcTask */
}

/* StartUART1RxTask function */
void StartUART1RxTask(void const * argument)
{
  /* USER CODE BEGIN StartUART1RxTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemUART1RxSigHandle, osWaitForever);
      
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		osDelay(1000);
		if(0x01 == CmdRxBuf[0] && 0xA5 == CmdRxBuf[2] && 0x5A == CmdRxBuf[3]) //cmd from labview via UART1 to start/stop sending radar data
		{
			switch(CmdRxBuf[1])
			{
				case 0x12:  //start sending radar data
					//RS485 EN = 1;
					osThreadResume(RadarDataTxHandle);
					break;
				case 0x13:  //stop sending radar data
					osThreadSuspend(RadarDataTxHandle);
					break;
				default:
					break;
			}   
    }
    //osSemaphoreRelease(bSemRadarDataTxSigHandle);
    osDelay(10);
  }
  /* USER CODE END StartUART1RxTask */
}

/* StartRadarDataTxTask function */
void StartRadarDataTxTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarDataTxTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
    uint8_t speed = (uint8_t)VrelLong_g;
    if(GetRadarData(CrashWarningLv, speed, MinRangeLong_g, TimetoCrash_g) == 0)	//get radar data successed
    {
      RadarData.Sys_State = RADAR_OK;			//radar data sending sys ok
      FillRadarDataTxBuf(CmdRadarDataTxBuf, RadarData);
      HAL_UART_Transmit(&huart1, CmdRadarDataTxBuf, 11, 1000);
      //DBC_SendDist(&hcan1, MinRangeLong_g);
			//if using RS485, EN = 0;
    }
    else
    {
      RadarData.Sys_State = RADAR_ERROR;	//radar data sending sys error
      FillRadarDataTxBuf(CmdRadarDataTxBuf, RadarData);
      HAL_UART_Transmit(&huart1, CmdRadarDataTxBuf, 11, 1000);
      //DBC_SendDist(&hcan1, MinRangeLong_g);
			//if using RS485, EN = 0;
    }
    osDelay(100);
  }
  /* USER CODE END StartRadarDataTxTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
