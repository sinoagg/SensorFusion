
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ADAS.h"
#include "InternalFlash.h"
#include "WTN6040.h"
#include "vehicle.h"
#include "aebs.h"
#include "user_can.h"
#include "user_radar.h"
#include "user_adas.h"
#include "thread.h"
#if GYRO_TYPE == GYRO_MPU6050
#include "MPU6050.h"
#endif

/* Defines -------------------------------------------------------------------*/

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t Turning_Collision = TURNING_COLLISION_NONE;
uint8_t Turning_Flag = STRAIGHT;
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
	MX_TIM3_Init(AEB_CAN_TX_TIME);
	MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	delay_init(100);

	//delay_ms(2000);
	WTN6_Broadcast(BELL_LOUDEST);
	delay_ms(100);
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
 * @brief  CAN Callback function
 * @note   can3 for Radar
 *				 can1 for Gyroscope
 * @param  *hcan: 
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{  	
  if(hcan->Instance == hcan1.Instance)
  {
    HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &VehicleCANRxHeader, YawCANRxBuf);
		//Gyroscope
    if(GYRO_ADDR == VehicleCANRxHeader.ExtId)      //gyroscope ID
		{
			//start gyro semaphore
			#if GYRO_TYPE == GYRO_MPU6050
      vehicle.yawRate = MPU_GetYawRate(YawCANRxBuf);
			vehicle.longAcc = MPU_GetXAcc(YawCANRxBuf);
			LED_GYRO_TOGGLE();
			
				#if RADAR_TYPE == ARS408
				ARS_SendVehicleYaw(&hcan3, vehicle.yawRate);	//send vehicle yawRate to Radar
				#endif
			
			#endif
		}
  }
	if(hcan->Instance == hcan3.Instance)
	{		
		static uint8_t i=0;
		HAL_CAN_GetRxMessage(&hcan3, CAN_FILTER_FIFO0, &RadarCAN_RxHeader, RadarCANRxBuf);
		if(i++>10)
		{
			i=0;
			LED_RADAR_TOGGLE();
		}

		#if RADAR_TYPE == ARS408
		osSemaphoreRelease(bSemRadarCalcSigHandle);	

		#elif RADAR_TYPE == EMRR
		if((RadarCANRxBuf[0]!=0) || ((RadarCANRxBuf[1]&0x7F)!=0))								//获取有效目标
		{
			EMRR_GetRadarObjData(&RadarCAN_RxHeader, RadarCANRxBuf, aEMRRGeneral+EMRR_RadarObjCount);
			EMRR_RadarObjCount ++;
		}
		if(RadarCAN_RxHeader.StdId==0x053F) 
		{
			EMRR_CalcRaderObjCloset(RadarCANRxBuf, aEMRRGeneral, &EMRRGeneral_Closet);
			EMRR_RadarObjCount=0;			//起始状态清零
			osSemaphoreRelease(bSemRadarCalcSigHandle);
		}
		#endif
	}
}
/** 
 * @brief  CAN Callback function
 * @note   can2 for Vehicle
 * @param  *hcan: 
 * @retval None
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == hcan2.Instance)
  {
		static uint8_t i=0;
		HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO1, &VehicleCANRxHeader, VehicleCANRxBuf);
		if(i++>10)
		{
			i=0;
			LED_VEHICLE_TOGGLE();
		}
		#if CAN_READ_VEHICLE
		
			#if VEHICLE_MODEL == DONGFENG
				if(VEHICLE_SPEED_ADDR == VehicleCANRxHeader.ExtId) //VehicleSpeed ID
				{
					vehicle.speed = VehicleCANRxBuf[2]; 		//vehicle speed in hex,km/h
					vehicleSwitch.brake = (VehicleCANRxBuf[3] >> 4) & 0x01;
					#if RADAR_TYPE == ARS408
					ARS_SendVehicleSpeed(&hcan3, vehicle.speed);	//send VehicleSpeed to Radar
					#elif RADAR_TYPE == EMRR
					#endif
				}
				if(VEHICLE_SWITCH_ADDR == VehicleCANRxHeader.ExtId) //VehicleSpeed ID
				{
					vehicleSwitch.right_turn = ((VehicleCANRxBuf[1] & 0x0F) == 0x02);
					vehicleSwitch.left_turn  = ((VehicleCANRxBuf[1] & 0x0F) == 0x01);
				}
				if(VEHICLE_ANGLE_ADDR == VehicleCANRxHeader.ExtId) //VehicleSpeed ID
				{
					uint16_t TW_angle, YawRate, LatAcc;
					TW_angle = ((uint16_t)VehicleCANRxBuf[1])<<8 | VehicleCANRxBuf[0];
					vehicle.tw_angle  = ((float)(TW_angle))/1024 - 31.374f;
					
					vehicle.tw_circle = (VehicleCANRxBuf[2] & 0x3F) - 32;
					vehicle.tw_type = (VehicleCANRxBuf[2]>>6) & 0x3;
					
					YawRate = ((uint16_t)VehicleCANRxBuf[4])<<8 | VehicleCANRxBuf[3];
					vehicle.yawRate = ((float)(YawRate))/8192 - 3.92f;
					
					LatAcc = ((uint16_t)VehicleCANRxBuf[6])<<8 | VehicleCANRxBuf[5];
					vehicle.latAcc  = ((float)(LatAcc))/2048 - 15.687f;
					
					vehicle.longAcc = (float)(VehicleCANRxBuf[7])*0.1f - 12.5f;
					
					#if RADAR_TYPE == ARS408
					ARS_SendVehicleYaw(&hcan3, vehicle.yawRate);	//send vehicle yawRate to Radar
					#endif
				}
			#elif VEHICLE_MODEL == BENZ
				if(VEHICLE_SPEED_ADDR == VehicleCANRxHeader.ExtId) //VehicleSpeed ID
				{
					vehicle.speed = VehicleCANRxBuf[7]; 		//vehicle speed in hex,km/h
					#if RADAR_TYPE == ARS408
					ARS_SendVehicleSpeed(&hcan3, vehicle.speed);	//send VehicleSpeed to Radar
					#elif RADAR_TYPE == EMRR
					#endif
				}
			#elif VEHICLE_MODEL == BYD
			#elif VEHICLE_MODEL == YUTONG
				if(0xD1 == VehicleCANRxBuf[0] && 0xD1 == VehicleCANRxBuf[2])
				{
					vehicle.speed = VehicleCANRxBuf[1];			//vehicle speed in hex,km/h
					#if RADAR_TYPE == ARS408
					ARS_SendVehicleSpeed(&hcan3, vehicle.speed);	//send VehicleSpeed to Radar
					#elif RADAR_TYPE == EMRR
					#endif
				}
		  #elif VEHICLE_MODEL == KINGLONG
				if(VEHICLE_SPEED_ADDR == VehicleCANRxHeader.ExtId) //VehicleSpeed ID
				{
					vehicle.speed = VehicleCANRxBuf[7]; 		//vehicle speed in hex,km/h
					#if RADAR_TYPE == ARS408
					ARS_SendVehicleSpeed(&hcan3, vehicle.speed);	//send vehicle speed to Radar
					#endif
				}
				else if(VEHICLE_SWITCH_ADDR == VehicleCANRxHeader.ExtId)
				{
					if(VehicleCANRxBuf[0] == 1)		//pack number == 1
					{
						vehicleSwitch.brake = VehicleCANRxBuf[2] & 0x3;
						vehicleSwitch.right_turn = (VehicleCANRxBuf[4] & 0xC) >> 2;
						vehicleSwitch.left_turn = (VehicleCANRxBuf[4] & 0x30) >> 4;
					}
				}
				else if(VEHICLE_ANGLE_ADDR == VehicleCANRxHeader.ExtId)
				{
					uint16_t TW_angle, YawRate, LatAcc;
					TW_angle = ((uint16_t)VehicleCANRxBuf[1])<<8 | VehicleCANRxBuf[0];
					vehicle.tw_angle  = ((float)(TW_angle))/1024 - 31.374f;
					
					vehicle.tw_circle = (VehicleCANRxBuf[2] & 0x3F) - 32;
					vehicle.tw_type = (VehicleCANRxBuf[2]>>6) & 0x3;
					
					YawRate = ((uint16_t)VehicleCANRxBuf[4])<<8 | VehicleCANRxBuf[3];
					vehicle.yawRate = ((float)(YawRate))/8192 - 3.92f;
					
					LatAcc = ((uint16_t)VehicleCANRxBuf[6])<<8 | VehicleCANRxBuf[5];
					vehicle.latAcc  = ((float)(LatAcc))/2048 - 15.687f;
					
					vehicle.longAcc = (float)(VehicleCANRxBuf[7])*0.1f - 12.5f;
					
					#if RADAR_TYPE == ARS408
					ARS_SendVehicleYaw(&hcan3, vehicle.yawRate);	//send vehicle yawRate to Radar
					#endif
					
          /*#if RADAR_TYPE == ARS408
          if(vehicle.tw_angle > 5 || vehicle.tw_angle < -5)
          {
            Turning_Flag = TURNING;
            Turning_Collision = ARS_CalcTurn(RadarGeneral, vehicle.tw_angle, vehicle.speed);
          }
          else
          {
            Turning_Flag = STRAIGHT;
            Turning_Collision = TURNING_COLLISION_NONE;
          }
        
          #elif RADAR_TYPE == EMRR
          if(vehicle.tw_angle > 5 || vehicle.tw_angle < -5)
          {
            Turning_Flag = TURNING;
            Turning_Collision = EMRR_CalcTurn(&EMRRGeneral_Closet, vehicle.tw_angle, vehicle.speed);
          }
          else
          {
            Turning_Flag = STRAIGHT;
            Turning_Collision = TURNING_COLLISION_NONE;
          }
          #endif
					*/
    
				}
			#endif
		#endif  		
  }
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
  if (htim->Instance == TIM4) 
	{
    HAL_IncTick();
  }
	else if (htim->Instance == TIM2) 
	{
		BUZZER_TOGGLE();
		LED_WARNING_TOGGLE();
  }
	else if (htim->Instance == TIM3)
	{
		AEB_CAN_TxReady = 1;
		if(vAEBS_Status.onlyRadarTimes > 0)
			vAEBS_Status.onlyRadarTimes -=1;
		if(vAEBS_Status.onlyRadarTimes % 10 == 8)
			LED_GYRO_TOGGLE();
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
