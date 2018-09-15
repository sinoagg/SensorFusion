
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ADAS.h"
#include "InternalFlash.h"
#include "WTN6040.h"
#include "ARS408.h"
#include "cmd.h"
#include "EMRR.h"
#include "MPU6050.h"
#include "switch.h"


/* Defines -------------------------------------------------------------------*/
//  ARS408
#if RADAR_TYPE
  #define RADAR_OFFSET	0.4f
//  EMRR
#else
  #define RADAR_OFFSET  0.0f
#endif
#define LANEWIDTH   1.5f
#define MAX_DECELARATION 0.4*9.8f
#define DELAY_TIME	0.4f

//	can3 id, vehicle
#if VEHICLE_MODEL == 2		//BYD
	#define VEHICLE_SPEED_ADDR	0x18FEF100
#elif VEHICLE_MODEL == 1	//YUTONG
	#define VEHICLE_SPEED_ADDR	0x18FE6E0B
#else	//KINGLONG
	#define VEHICLE_SPEED_ADDR	0x18FE6C00
	#define VEHICLE_SWITCH_ADDR	0x18FA0500
#endif
//	can3 id, gyro
#define GYRO_ADDR 0x18FEE0D8
#define GYRO_ADDR2 0x18FEE1D8
//	can1 id, dbc
#define DBC_ADDR  0x509

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//ARS408
MW_RadarObjStatus RadarObjStatus;
MW_RadarGeneral RadarGeneral[16];
//EMRR
EMRR_RadarGeneral aEMRRGeneral[64];
EMRR_RadarGeneral EMRRGeneral_Closet;

Cmd_RadarData RadarData;

CAN_TxHeaderTypeDef CAN_TxDBCHeader={DBC_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_RxHeaderTypeDef RadarCANRxHeader;
CAN_RxHeaderTypeDef VehicleCANRxHeader;

ADAS_HandleTypeDef ADAS_dev;
uint8_t MW_RadarRxComplete=0;
uint8_t EMRR_RadarRxComplete=0;
uint8_t ADASRxComplete=0;
uint8_t UART1RxComplete=0;
uint8_t SpeedRxComplete=0;
uint8_t CmdRxComplete=0;
uint8_t ADASRxBuf[32]={0};
uint8_t ADASDispBuf[32] = {0};
uint8_t CmdRxBuf[4]={0};
uint8_t CmdRadarDataTxBuf[11];
uint8_t RadarCANRxBuf[8]={0};
uint8_t VehicleCANRxBuf[8]={0};
uint8_t YawCANRxBuf[8] = {0};
uint8_t CrashWarningLv = WARNING_NONE;
uint8_t VehicleSpeed_g = 0;
uint8_t Vehicle_CAN_Flag = 0;
uint16_t ADC_ConvertedValue[2] = {0};
uint32_t DMA_Transfer_Complete_Count=0;

uint8_t RadarTimes = 0;
uint8_t RadarYawTimes = 0;

float Yaw_g = 0.0;
float YawRate_g = 0.0;
float XAcc_g = 0.0;
float VrelLong_g = 0.0;
float MinRangeLong_g = 0.0;
float TimetoCrash_g = 0.0;
__IO float ADC_ConvertedValueF[2];


extern osSemaphoreId bSemRadarCANRxSigHandle;
extern osSemaphoreId bSemGyroCommSigHandle;
extern osSemaphoreId bSemSpeedRxSigHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_CAN3_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	delay_init(100);
  #if ADAS_COMM
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);	//ADAS
  #endif
  #if RADAR_DATA_SEND
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //RADAR
  #endif

	WTN6_Broadcast(BELL_LOUDEST);
	delay_ms(100);
	//WTN6_Broadcast(0X02);
	WTN6_Broadcast(BELL_ADAS_START);
	delay_ms(3000);
	WTN6_Broadcast(BELL_BB_1000MS);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */
/** 
 * @brief  can start for sending DBC(in ChongQing)
 * @note   not using in Benz version
 * @param  *hcan: may be hcan1
 * @retval 0 for ok
 */
uint8_t DBC_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  return 0;
}

uint8_t Gyro_CAN_Init(CAN_HandleTypeDef *hcan)
{
	//config CAN filter to receive Gyro
	//ID_HIGH,\
	ID_LOW,\
	MASK_HIGH,\
	MASK_LOW,\
	FIFO 0/1, filter_bank(0-13/14-27), filter_mode(LIST/MASK), filter_scale, EN/DISABLE filter, SlaveStartFilterBank
	CAN_FilterTypeDef GyroCANFilter = {
		(GYRO_ADDR>>13) & 0xFFFF,\
		((GYRO_ADDR & 0xFFFF) <<3) | 0x4,\
		0xFF<<3 | 0xF,\
		0xFF00<<3, \
    CAN_FILTER_FIFO0, 0, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,1
	};
	HAL_CAN_ConfigFilter(hcan, &GyroCANFilter);

	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  return 0;
}

uint8_t Vehicle_CAN_Init(CAN_HandleTypeDef *hcan)
{
	//config CAN3 filter to receive Vehicle Speed
	//ID_HIGH,\
	ID_LOW,\
	MASK_HIGH,\
	MASK_LOW,\
	FIFO 0/1, filter_bank(0-13/14-27), filter_mode(LIST/MASK), filter_scale, EN/DISABLE filter, SlaveStartFilterBank
	CAN_FilterTypeDef VehicleCANFilter = {
		VEHICLE_SPEED_ADDR>>13 & 0xFFFF,\
		((VEHICLE_SPEED_ADDR & 0xFFFF) <<3) | 0x4,\
		0xFF<<3 | 0xF,\
		0xFF00<<3,\
		CAN_FILTER_FIFO0, 1, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,1
	};
	HAL_CAN_ConfigFilter(hcan, &VehicleCANFilter);
  //config CAN3 filter to receive Vehicle switch data
	//ID_HIGH,\
	ID_LOW,\
	MASK_HIGH,\
	MASK_LOW,\
	FIFO 0/1, filter_bank(0-13/14-27), filter_mode(LIST/MASK), filter_scale, EN/DISABLE filter, SlaveStartFilterBank
  #if VEHICLE_MODEL == 0
	CAN_FilterTypeDef VehicleSwitchCANFilter = {
		VEHICLE_SWITCH_ADDR>>13 & 0xFFFF,\
		((VEHICLE_SWITCH_ADDR & 0xFFFF) <<3) | 0x4,\
		0xFF<<3 | 0xF,\
		0xFF00<<3,\
		CAN_FILTER_FIFO0, 2, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,1
	};
	HAL_CAN_ConfigFilter(hcan, &VehicleSwitchCANFilter);
  #endif
	
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	return 0;
}

/** 
 * @brief  CAN Callback function
 * @note   can2 for Radar, can3(can1 in Benz version) for Gyro & Vehicle Speed
 * @param  *hcan: 
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == hcan1.Instance)
	{
		#if GYRO_CAN == 1
		HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &VehicleCANRxHeader, YawCANRxBuf);
		//Gyroscope
    if(GYRO_ADDR == VehicleCANRxHeader.ExtId)      //gyroscope ID
		{
			//start gyro semaphore
      osSemaphoreRelease(bSemGyroCommSigHandle);
			HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
		}
		#endif
	}
  if(hcan->Instance == hcan2.Instance)
  {
  	HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO0, &RadarCANRxHeader, RadarCANRxBuf);
		// send RADAR(ARS408) data to CAN1(for debug)
		#if RADAR_TYPE == 1
		uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
		CAN_TxDBCHeader.StdId = RadarCANRxHeader.StdId;
		if(RadarCANRxHeader.StdId == 0x60B)
		{
			uint16_t dist = 0;
			uint16_t temp=0;
			dist = (uint16_t)(((*(RadarCANRxBuf+1))<<5) | ((*(RadarCANRxBuf+2))>>3));
			dist -= ((VehicleSpeed_g / 22)-0.0)*5;		
			*(RadarCANRxBuf + 1) =(dist>>5);
			temp =((dist<<3)&0xF8);
			*(RadarCANRxBuf + 2) &=0x07;
			*(RadarCANRxBuf + 2) |=temp;
			
		}
		HAL_CAN_AddTxMessage(&hcan1, &CAN_TxDBCHeader, RadarCANRxBuf, &CAN_TxMailBox);
		#endif
		
  	osSemaphoreRelease(bSemRadarCANRxSigHandle);
		//ARS408
		#if RADAR_TYPE
		
		//EMRR
		#else
		EMRR_RadarRxComplete = 1;
		#endif
  }
	if(hcan->Instance == hcan3.Instance)
	{
		HAL_CAN_GetRxMessage(&hcan3, CAN_FILTER_FIFO0, &VehicleCANRxHeader, VehicleCANRxBuf);
		HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
		
		#if GYRO_CAN == 3
		//Gyroscope
    if(GYRO_ADDR == VehicleCANRxHeader.ExtId)      //gyroscope ID
    {
      for(uint8_t i = 0; i < 8; i++)
      {
        YawCANRxBuf[i] = VehicleCANRxBuf[i];
      }
      //start gyro semaphore
      osSemaphoreRelease(bSemGyroCommSigHandle);
    }
		#endif
      
		#if CAN_READ_VEHICLE
    //  2 bytes(mid)must fit
		//	Vehicle Speed
    if((VEHICLE_SPEED_ADDR & 0x00FFFF00) == (VehicleCANRxHeader.ExtId & 0x00FFFF00)) //VehicleSpeed ID
    {
      osSemaphoreRelease(bSemSpeedRxSigHandle);
      Vehicle_CAN_Flag = 1;
    }
    //  KINGLONG
    #if VEHICLE_MODEL == 0
		//	Vehicle Switch data
    else if((VEHICLE_SWITCH_ADDR & 0x00FFFF00) == (VehicleCANRxHeader.ExtId & 0x00FFFF00)) //VehicleSwitch ID
    {
      osSemaphoreRelease(bSemSpeedRxSigHandle);
      Vehicle_CAN_Flag = 2;
    }
    #endif
		else
			Vehicle_CAN_Flag = 0;

    #endif
	}
	
}

/** 
 * @brief  Send Radar Distance in DBC protocol
 * @note   only use in ChongQing
 * @param  *hcan: 
 * @param  Dist: /m
 * @retval 0 for ok
 */
uint8_t DBC_SendDist(CAN_HandleTypeDef *hcan, float Dist)
{
  uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
  uint32_t Dist_mm = (Dist - RADAR_OFFSET) * 1000;   //distance /mm
  uint8_t CANTxBuf[4] = {0};
  CANTxBuf[3] = Dist_mm;
  CANTxBuf[2] = Dist_mm >> 8;
  CANTxBuf[1] = Dist_mm >> 16;
  CANTxBuf[0] = Dist_mm >> 24;
  HAL_CAN_AddTxMessage(hcan, &CAN_TxDBCHeader, CANTxBuf, &CAN_TxMailBox);
  return 0;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
