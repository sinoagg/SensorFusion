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
#include "can.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId RadarCommHandle;
osThreadId ADASCommHandle;
osThreadId SoundWarningHandle;
osThreadId GyroCommHandle;
osThreadId CANSpeedReadHandle;
osThreadId RadarCalcTaskHandle;
osThreadId UART1RxTaskHandle;
osThreadId RadarDataTxTaskHandle;
osSemaphoreId bSemRadarCANRxSigHandle;
osSemaphoreId bSemADASRxSigHandle;
osSemaphoreId bSemSoundWarningSigHandle;
osSemaphoreId bSemGyroCommSigHandle;
osSemaphoreId bSemSpeedRxSigHandle;
osSemaphoreId bSemRadarCalcSigHandle;
osSemaphoreId bSemUART1RxSigHandle;

/* USER CODE BEGIN Variables */

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
extern uint8_t Vehicle_CAN_Init(CAN_HandleTypeDef *hcan);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
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

  /* definition and creation of GyroComm */
  osThreadDef(GyroComm, StartGyroCommTask, osPriorityIdle, 0, 128);
  GyroCommHandle = osThreadCreate(osThread(GyroComm), NULL);

  /* definition and creation of CANSpeedRead */
  #if CAN_READ_VEHICLE
  osThreadDef(CANSpeedRead, StartCANSpeedReadTask, osPriorityIdle, 0, 128);
  CANSpeedReadHandle = osThreadCreate(osThread(CANSpeedRead), NULL);
  #endif

  /* definition and creation of RadarCalcTask */
  osThreadDef(RadarCalcTask, StartRadarCalcTask, osPriorityIdle, 0, 128);
  RadarCalcTaskHandle = osThreadCreate(osThread(RadarCalcTask), NULL);

	#if RADAR_DATA_SEND
  /* definition and creation of UART1RxTask */
  osThreadDef(UART1RxTask, StartUART1RxTask, osPriorityIdle, 0, 64);
  UART1RxTaskHandle = osThreadCreate(osThread(UART1RxTask), NULL);

  /* definition and creation of RadarDataTxTask */
  osThreadDef(RadarDataTxTask, StartRadarDataTxTask, osPriorityIdle, 0, 128);
  RadarDataTxTaskHandle = osThreadCreate(osThread(RadarDataTxTask), NULL);
	
	osThreadSuspend(RadarDataTxHandle);		//suspend Radar data send task
	#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	
	//hcan1~hcan3 init, start
	DBC_Init(&hcan1);
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
      osSemaphoreRelease(bSemADASRxSigHandle);
		}
    #endif
		
    #if DBC_SEND
    DBC_SendDist(&hcan1, MinRangeLong);
    #endif
		
		#if ATM_READ
		/* Vref = 3.3(fullscale), 2^12=4096(scale),
       when input is 3.3V, the result is 4096 */    
    ADC_ConvertedValueF[0] =(double)(ADC_ConvertedValue[0]&0xFFF)*3.3/4096; 	// ADC_ConvertedValue only lowest 12bit
		ADC_ConvertedValueF[1] =(double)(ADC_ConvertedValue[1]&0xFFF)*3.3/4096; 	// ADC_ConvertedValue only lowest 12bit
		#endif
		
		osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartRadarCommTask function */
void StartRadarCommTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarCommTask */
  /* Infinite loop */
  for(;;)
  {
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
    osDelay(1);
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
    osDelay(1);
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
    osDelay(1);
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
    osDelay(1);
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
    osDelay(1);
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
    osDelay(1);
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
    osDelay(1);
  }
  /* USER CODE END StartRadarDataTxTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
