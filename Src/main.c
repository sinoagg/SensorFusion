
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

/* USER CODE BEGIN Includes */
#include "ADAS.h"
#include "InternalFlash.h"
#include "WTN6040.h"
#include "ARS408.h"

#define MAX_OBJ_NUM	4										//最大目标识别数量
#define LANEWIDTH 2											//车道线宽度
#define MAX_DECELARATION 0.4*9.8				//制动系统最大减速度
#define DELAY_TIME	0.4									//系统延迟时间
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;
osThreadId RadarCommHandle;
osThreadId ADASCommHandle;
osThreadId SoundWarningHandle;
osThreadId LightWarningHandle;
osThreadId CANSpeedReadHandle;
osThreadId StartCalculateHandle;
osSemaphoreId bSemRadarCANRxSigHandle;
osSemaphoreId bSemADASRxSigHandle;
osSemaphoreId bSemSoundWarningSigHandle;
osSemaphoreId bSemLightWarningSigHandle;
osSemaphoreId bSemSpeedRxSigHandle;
osSemaphoreId bSemCalculateSigHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define	WARNING_NONE 0
#define	WARNING_LOW 1
#define	WARNING_HIGH 2

MW_RadarConfig RadarConfig;
MW_RadarFilterConfig RadarFilterConfig;
MW_RadarObjStatus RadarObjStatus;
MW_RadarGeneral RadarGeneral[16];

CAN_RxHeaderTypeDef RadarCANRxHeader;

ADAS_HandleTypeDef ADAS_dev;
uint8_t MW_RadarRxComplete=0;
uint8_t ADASRxComplete=0;
uint8_t SpeedRxComplete=0;
uint8_t ADASRxBuf[32]={0};
uint8_t RadarCANRxBuf[8]={0};
uint8_t CrashWarningLv=WARNING_NONE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_CAN3_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartRadarCommTask(void const * argument);
void StartADASCommTask(void const * argument);
void StartSoundWarningTask(void const * argument);
void StartLightWarningTask(void const * argument);
void StartCANSpeedReadTask(void const * argument);
void StartCalculateTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


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
  //MX_CAN3_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);	//ADAS串口接收使能
	ARS_Init(&hcan2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(bSemRadarCANRxSig);
  bSemRadarCANRxSigHandle = osSemaphoreCreate(osSemaphore(bSemRadarCANRxSig), 1);

  osSemaphoreDef(bSemADASRxSig);

  bSemADASRxSigHandle = osSemaphoreCreate(osSemaphore(bSemADASRxSig), 1);
	osSemaphoreDef(bSemSoundWarningSig);
  bSemSoundWarningSigHandle = osSemaphoreCreate(osSemaphore(bSemSoundWarningSig), 1);
	osSemaphoreDef(bSemLightWarningSig);
  bSemLightWarningSigHandle = osSemaphoreCreate(osSemaphore(bSemLightWarningSig), 1);
	osSemaphoreDef(bSemSpeedRxSig);
  bSemSpeedRxSigHandle = osSemaphoreCreate(osSemaphore(bSemSpeedRxSig), 1);
	osSemaphoreDef(bSemCalculateSig);
	bSemCalculateSigHandle = osSemaphoreCreate(osSemaphore(bSemCalculateSig), 1);
	
	osSemaphoreWait(bSemRadarCANRxSigHandle, osWaitForever);		//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
	osSemaphoreWait(bSemADASRxSigHandle, osWaitForever);				//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  osSemaphoreWait(bSemSoundWarningSigHandle, osWaitForever);	//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  osSemaphoreWait(bSemLightWarningSigHandle, osWaitForever);	//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  osSemaphoreWait(bSemSpeedRxSigHandle, osWaitForever);				//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  osSemaphoreWait(bSemCalculateSigHandle, osWaitForever);			//老版本默认信号量创建时是有效的，所以需要读一遍使其无效

	
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
  osThreadDef(ADASComm, StartADASCommTask, osPriorityBelowNormal, 0, 128);
  ADASCommHandle = osThreadCreate(osThread(ADASComm), NULL);

  /* definition and creation of SoundWarning */
  osThreadDef(SoundWarning, StartSoundWarningTask, osPriorityIdle, 0, 128);
  SoundWarningHandle = osThreadCreate(osThread(SoundWarning), NULL);

  /* definition and creation of LightWarning */
  osThreadDef(LightWarning, StartLightWarningTask, osPriorityIdle, 0, 128);
  LightWarningHandle = osThreadCreate(osThread(LightWarning), NULL);

  /* definition and creation of CANSpeedRead */
  osThreadDef(CANSpeedRead, StartCANSpeedReadTask, osPriorityIdle, 0, 128);
  CANSpeedReadHandle = osThreadCreate(osThread(CANSpeedRead), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(CalculateTask, StartCalculateTask, osPriorityNormal, 0, 128);
  StartCalculateHandle = osThreadCreate(osThread(CalculateTask), NULL);
	
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN3 init function */
static void MX_CAN3_Init(void)
{

  hcan3.Instance = CAN3;
  hcan3.Init.Prescaler = 10;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = DISABLE;
  hcan3.Init.AutoRetransmission = DISABLE;
  hcan3.Init.ReceiveFifoLocked = DISABLE;
  hcan3.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BELL_BUSY_GPIO_Port, BELL_BUSY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED3_Pin|LED4_Pin|LED5_Pin|LED6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BELL_DATA_Pin */
  GPIO_InitStruct.Pin = BELL_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BELL_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BELL_BUSY_Pin */
  GPIO_InitStruct.Pin = BELL_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BELL_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LED5_Pin LED6_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LED5_Pin|LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO0, &RadarCANRxHeader, RadarCANRxBuf);
	osSemaphoreRelease(bSemRadarCANRxSigHandle);
	//__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FF0);
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		osDelay(100);
  }
  /* USER CODE END 5 */ 
}

/* StartRadarCommTask function */
void StartRadarCommTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarCommTask */
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(bSemRadarCANRxSigHandle, osWaitForever);
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		ARS_GetRadarObjGeneral(RadarCANRxBuf, RadarGeneral);
		if(RadarCANRxBuf[0]==0x03)	//一组4个读取完毕
			osSemaphoreRelease(bSemCalculateSigHandle);
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
		if(ADAS_dev.crash_level==0x03)//严重报警
		{}
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
		if(CrashWarningLv==WARNING_HIGH)
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		}
		else if(CrashWarningLv==WARNING_LOW)
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		}
		else
		{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
		}
		osDelay(10);
  }
  /* USER CODE END StartSoundWarningTask */
}

/* StartLightWarningTask function */
void StartLightWarningTask(void const * argument)
{
  /* USER CODE BEGIN StartLightWarningTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemLightWarningSigHandle, osWaitForever);
		osDelay(10);
  }
  /* USER CODE END StartLightWarningTask */
}

/* StartCANSpeedReadTask function */
void StartCANSpeedReadTask(void const * argument)
{
  /* USER CODE BEGIN StartCANSpeedReadTask */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bSemSpeedRxSigHandle, osWaitForever);
		osDelay(10);
  }
  /* USER CODE END StartCANSpeedReadTask */
}

void StartCalculateTask(void const * argument)
{
	for(;;)
	{
		osSemaphoreWait(bSemCalculateSigHandle, osWaitForever);
		uint8_t i;
		uint8_t MinRange=255;									//初始化为最大距离
		uint32_t relSpeed=0;
		for(i=0;i<MAX_OBJ_NUM;i++)						//获取可能碰撞的最小距离和相对速度
		{
			if(( 0.2*(RadarGeneral[i].Obj_DistLat - 204.6) * 2.0) < LANEWIDTH && RadarGeneral[i].Obj_DistLong < MinRange && RadarGeneral[i].Obj_DistLong != 0) 
			//if(( 0.2*(RadarGeneral[i].Obj_DistLat-500) * 2.0) < LANEWIDTH && RadarGeneral[i].Obj_DistLong < MinRange) 
      {
				MinRange = RadarGeneral[i].Obj_DistLong;				//此处仍然保留着整数原始状态
				relSpeed = RadarGeneral[i].Obj_VrelLong;				//以减小计算量
      }
		}
		if(MinRange<100)											//如果此距离小于一个足够小的距离，再开始计算，否则浪费时间		
		{
			float VrelLong = 0.25* (relSpeed - 128);	//获取真实车速
			float TimetoCrash = (float)MinRange/VrelLong;
			if(TimetoCrash<0.8)
			{
				CrashWarningLv=WARNING_HIGH;
			}
			else if(TimetoCrash<1)
			{
				CrashWarningLv=WARNING_LOW;
			}
			else
				CrashWarningLv=WARNING_NONE;
		}
		CrashWarningLv=WARNING_NONE;
		osSemaphoreRelease(bSemSoundWarningSigHandle);
		osDelay(1);
	}
}

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
