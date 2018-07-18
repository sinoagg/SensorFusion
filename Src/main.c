
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
#include "cmd.h"
#include "EMRR.h"

#define MAX_OBJ_NUM	4										//最大目标识别数量
#define LANEWIDTH 1.5f									//车道线宽度
#define MAX_DECELARATION 0.4*9.8f				//制动系统最大减速度
#define DELAY_TIME	0.4f								//系统延迟时间
#define LIMIT_RANGE 200									//计算碰撞时间的极限距离/m
#define VEHICLE_SPEED_ADDR_HIGH 0x18FE
#define VEHICLE_SPEED_ADDR_LOW	0x6E0B
#define DBC_ADDR 0x509

//switches
/**
 * RADAR_TYPE
 * 0  EMRR
 * 1  ARS408
*/
#define RADAR_TYPE 0
#define DBC_SEND 0
#define ADAS_COMM 1
//labview
#define RADAR_DATA_SEND 0
//Vehicle Speed & gyro via CAN3
#define CAN_READ_VEHICLE 0
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
osThreadId GyroCommHandle;
osThreadId CANSpeedReadHandle;
osThreadId StartCalculateHandle;
osThreadId UART1RxHandle;
osThreadId RadarDataTxHandle;
osSemaphoreId bSemRadarCANRxSigHandle;
osSemaphoreId bSemADASRxSigHandle;
osSemaphoreId bSemSoundWarningSigHandle;
osSemaphoreId bSemGyroCommSigHandle;
osSemaphoreId bSemSpeedRxSigHandle;
osSemaphoreId bSemCalculateSigHandle;
osSemaphoreId bSemUART1RxSigHandle;
osSemaphoreId bSemRadarDataTxSigHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define	WARNING_NONE 0
#define	WARNING_LOW 1
#define WARNING_MID 2
#define	WARNING_HIGH 3

//ARS408
MW_RadarObjStatus RadarObjStatus;
MW_RadarGeneral RadarGeneral[16];
//EMRR
EMRR_RadarGeneral aEMRRGeneral[64];	//接受到的64组数据
EMRR_RadarGeneral EMRRGeneral_Closet;//距离最近的目标

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
uint8_t CmdRxBuf[4]={0};
uint8_t CmdRadarDataTxBuf[11];
uint8_t RadarCANRxBuf[8]={0};
uint8_t VehicleCANRxBuf[6]={0};
uint8_t CrashWarningLv = WARNING_NONE;
uint8_t VehicleSpeed = 0;

float yaw = 0.0;
float yawRate = 0.0;
float VrelLong = 0.0;
float MinRangeLong = 0.0;
float TimetoCrash = 0.0;

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
void StartGyroCommTask(void const * argument);
void StartCANSpeedReadTask(void const * argument);
void StartCalculateTask(void const * argument);
void StartUART1RxTask(void const * argument);
void StartRadarDataTxTask(void const * argument);

uint8_t DBC_Init(CAN_HandleTypeDef *hcan);
uint8_t Vehicle_CAN_Init(CAN_HandleTypeDef *hcan);

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
  /* USER CODE BEGIN 2 */
	delay_init(100);
	HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
  #if ADAS_COMM
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);	//ADAS串口接收使能
  #endif
  #if RADAR_DATA_SEND
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //雷达数据发送串口接收使能
  #endif

	WTN6_Broadcast(BELL_LOUDEST);									//设置喇叭为最大音量
	delay_ms(100);
	WTN6_Broadcast(BELL_ADAS_START);
	delay_ms(5000);
	WTN6_Broadcast(BELL_BB_1000MS);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreDef(bSemRadarCANRxSig);
  bSemRadarCANRxSigHandle = osSemaphoreCreate(osSemaphore(bSemRadarCANRxSig), 1);
  #if ADAS_COMM
  osSemaphoreDef(bSemADASRxSig);
  bSemADASRxSigHandle = osSemaphoreCreate(osSemaphore(bSemADASRxSig), 1);
  #endif
	osSemaphoreDef(bSemSoundWarningSig);
  bSemSoundWarningSigHandle = osSemaphoreCreate(osSemaphore(bSemSoundWarningSig), 1);
  #if CAN_READ_VEHICLE
	osSemaphoreDef(bSemGyroCommSig);
  bSemGyroCommSigHandle = osSemaphoreCreate(osSemaphore(bSemGyroCommSig), 1);
  osSemaphoreDef(bSemSpeedRxSig);  
  bSemSpeedRxSigHandle = osSemaphoreCreate(osSemaphore(bSemSpeedRxSig), 1);
	#endif
  osSemaphoreDef(bSemCalculateSig);
	bSemCalculateSigHandle = osSemaphoreCreate(osSemaphore(bSemCalculateSig), 1);
  #if RADAR_DATA_SEND
  osSemaphoreDef(bSemUART1RxSig);
  bSemUART1RxSigHandle = osSemaphoreCreate(osSemaphore(bSemUART1RxSig), 1);
  osSemaphoreDef(bSemRadarDataTxSig);
  bSemRadarDataTxSigHandle = osSemaphoreCreate(osSemaphore(bSemRadarDataTxSig), 1);
  #endif

	osSemaphoreWait(bSemRadarCANRxSigHandle, osWaitForever);		//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  #if ADAS_COMM
  osSemaphoreWait(bSemADASRxSigHandle, osWaitForever);				//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  #endif
  osSemaphoreWait(bSemSoundWarningSigHandle, osWaitForever);	//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  #if CAN_READ_VEHICLE
  osSemaphoreWait(bSemGyroCommSigHandle, osWaitForever);	    //老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  osSemaphoreWait(bSemSpeedRxSigHandle, osWaitForever);				//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  #endif
  osSemaphoreWait(bSemCalculateSigHandle, osWaitForever);			//老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  #if RADAR_DATA_SEND
  osSemaphoreWait(bSemUART1RxSigHandle, osWaitForever);       //老版本默认信号量创建时是有效的，所以需要读一遍使其无效
  //osSemaphoreWait(bSemRadarDataTxSigHandle, osWaitForever);	//Radar Data Tx at the beginning of the system
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(CalculateTask, StartCalculateTask, osPriorityNormal, 0, 128);
  StartCalculateHandle = osThreadCreate(osThread(CalculateTask), NULL);

  #if RADAR_DATA_SEND
  osThreadDef(UART1RxTask, StartUART1RxTask, osPriorityNormal, 0, 128);
  UART1RxHandle = osThreadCreate(osThread(UART1RxTask), NULL);

  osThreadDef(RadarDataTxTask, StartRadarDataTxTask, osPriorityNormal, 0, 128);
  RadarDataTxHandle = osThreadCreate(osThread(RadarDataTxTask), NULL);
	
	osThreadSuspend(RadarDataTxHandle);		//挂起串口发送雷达数据线程
  #endif

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

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
  hcan3.Init.Prescaler = 20;
  hcan3.Init.Mode = CAN_MODE_NORMAL;
  hcan3.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan3.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan3.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan3.Init.TimeTriggeredMode = DISABLE;
  hcan3.Init.AutoBusOff = DISABLE;
  hcan3.Init.AutoWakeUp = ENABLE;
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
  huart3.Init.BaudRate = 19200;
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
  HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BELL_BUSY_GPIO_Port, BELL_BUSY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED0_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, BUZZER_Pin|LED3_Pin|LED4_Pin|LED5_Pin|LED6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BELL_DATA_Pin */
  GPIO_InitStruct.Pin = BELL_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BELL_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BELL_BUSY_Pin */
	GPIO_InitStruct.Pin = BELL_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BELL_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED0_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin|LED3_Pin LED4_Pin LED5_Pin LED6_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|LED3_Pin|LED4_Pin|LED5_Pin|LED6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance == hcan2.Instance)
  {
  	HAL_CAN_GetRxMessage(&hcan2, CAN_FILTER_FIFO0, &RadarCANRxHeader, RadarCANRxBuf);
  	osSemaphoreRelease(bSemRadarCANRxSigHandle);
		//ARS408
		#if RADAR_TYPE
		
		//EMRR
		#else
		EMRR_RadarRxComplete = 1;
		#endif
  	//__HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FF0);
		//HAL_CAN_DeactivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);		// 关闭中断
  }
	if(hcan->Instance == hcan3.Instance)
	{
    #if CAN_READ_VEHICLE
		HAL_CAN_GetRxMessage(&hcan3, CAN_FILTER_FIFO0, &VehicleCANRxHeader, VehicleCANRxBuf);
		HAL_GPIO_TogglePin(LED6_GPIO_Port,LED6_Pin);
    if(0x18FF0DE6 == VehicleCANRxHeader.ExtId)      //gyroscope ID
      //start gyro semaphore
      osSemaphoreRelease(bSemGyroCommSigHandle);
    else if(0x18FE6E0B == VehicleCANRxHeader.ExtId) //VehicleSpeed ID
  	  osSemaphoreRelease(bSemSpeedRxSigHandle);
    #endif
	}
	
}

uint8_t DBC_Init(CAN_HandleTypeDef *hcan)
{
  //配置CAN滤波器接收Objct_General信息，即相对目标的距离、速度等
  //CAN_FilterTypeDef MW_RadarCANFilter={OBJ_GENERAL_ADDR<<5,0,0xEFE<<5,0,CAN_FILTER_FIFO0, 14, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,14};   //0x60B 和 0x60A同时检测
  //CAN_FilterTypeDef MW_RadarCANFilter = {0,OBJ_GENERAL_ADDR,0,0xEFF,CAN_FILTER_FIFO0,CAN_FILTERMODE_IDLIST,CAN_FILTERSCALE_32BIT,ENABLE,0};
  //HAL_CAN_ConfigFilter(hcan, &MW_RadarCANFilter);
  HAL_CAN_Start(hcan);
  //HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  return 0;
}

uint8_t Vehicle_CAN_Init(CAN_HandleTypeDef *hcan)
{
	//配置CAN3滤波器接收车速信息
	CAN_FilterTypeDef VehicleCANFilter={VEHICLE_SPEED_ADDR_HIGH<<3,VEHICLE_SPEED_ADDR_LOW<<3 | 0x4,0xF66<<3,0<<3,CAN_FILTER_FIFO0, 1, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,1};
	//CAN_FilterTypeDef VehicleCANFilter = {0xC7F3, 0x706C, 0, 0, CAN_FilterFIFO1, 15, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE, 15};
	//CAN_FilterTypeDef VehicleCANFilter = {0x0000, 0x0, 0, 0, CAN_FilterFIFO1, 1, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE, 1};

	HAL_CAN_ConfigFilter(hcan, &VehicleCANFilter);
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	return 0;
}

uint8_t DBC_SendDist(CAN_HandleTypeDef *hcan, float Dist)
{
  uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
  uint32_t Dist_mm = (Dist - 0.4f) * 1000;   //以毫米为单位的距离
  uint8_t CANTxBuf[4] = {0};
  CANTxBuf[3] = Dist_mm;
  CANTxBuf[2] = Dist_mm >> 8;
  CANTxBuf[1] = Dist_mm >> 16;
  CANTxBuf[0] = Dist_mm >> 24;
  HAL_CAN_AddTxMessage(hcan, &CAN_TxDBCHeader, CANTxBuf, &CAN_TxMailBox);
  return 0;
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    #if RADAR_DATA_SEND
		HAL_UART_Receive_DMA(&huart1, CmdRxBuf, 4);//接收指令信息
		if(UART1RxComplete==1)
		{
			UART1RxComplete=0;
			osSemaphoreRelease(bSemUART1RxSigHandle);
		}
    #endif

    #if ADAS_COMM
    HAL_UART_Receive_DMA(&huart3, ADASRxBuf, 32);//接收指令信息
		if(ADASRxComplete==1)
		{
      ADASRxComplete=0;
      osSemaphoreRelease(bSemADASRxSigHandle);
		}
    #endif
		
    #if DBC_SEND
    DBC_SendDist(&hcan1, MinRangeLong);
    #endif
		/*
		uint32_t CAN_TxMailBox = CAN_TX_MAILBOX1;
		uint8_t CAN3TxBuf[8]={3};
		HAL_CAN_AddTxMessage(&hcan3, &CAN_TxDBCHeader, CAN3TxBuf, &CAN_TxMailBox);*/
		osDelay(20);
  }
  /* USER CODE END 5 */ 
}

/* StartRadarCommTask function */
void StartRadarCommTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarCommTask */
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
		if(RadarCANRxHeader.StdId==0x60A)												//60B的之前都读取完毕，开始计算
		{
			minRadarDistFlag = 1;
		}
		else																										//0x60B时读取目标距离、速度信息
		{
			if(minRadarDistFlag)
			{
				minRadarDistFlag = 0;
				ARS_GetRadarObjGeneral(RadarCANRxBuf, RadarGeneral);//获取最近目标数据，即收到的第一个目标
				osSemaphoreRelease(bSemCalculateSigHandle);
			}
		}
		
    //EMRR
		#else
		EMRR_GetRaderObjCloset(RadarCANRxBuf, aEMRRGeneral, &EMRRGeneral_Closet);
		if(EMRRGeneral_Closet.trackRange != 0)
			osSemaphoreRelease(bSemCalculateSigHandle);
		#endif
		osDelay(1);
  }
  /* USER CODE END StartRadarCommTask */
}

void StartUART1RxTask(void const * argument)
{
  /* USER CODE BEGIN StartUART1RxTask */
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(bSemUART1RxSigHandle, osWaitForever);
      
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		osDelay(1000);
		if(0x01 == CmdRxBuf[0] && 0xA5 == CmdRxBuf[2] && 0x5A == CmdRxBuf[3]) //接收到启动或停止指令
		{
			switch(CmdRxBuf[1])
			{
				case 0x12:  //启动输出数据
					//RS485需要让EN = 1; //发送状态
					osThreadResume(RadarDataTxHandle);
					break;
				case 0x13:  //停止发送数据
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

/**
 * [StartRadarDataTxTask: send Radar Data to other device]
 * @param argument [no need]
 */
void StartRadarDataTxTask(void const * argument)
{
  /* USER CODE BEGIN StartRadarDataTxTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED5_GPIO_Port,LED5_Pin);
    uint8_t speed = (uint8_t)VrelLong;
    if(GetRadarData(CrashWarningLv, speed, MinRangeLong, TimetoCrash) == 0)	//获取雷达数据成功
    {
      RadarData.Sys_State = RADAR_OK;			//雷达数据发送系统正常工作
      FillRadarDataTxBuf(CmdRadarDataTxBuf, RadarData);
      HAL_UART_Transmit(&huart1, CmdRadarDataTxBuf, 11, 1000);
      DBC_SendDist(&hcan1, MinRangeLong);
			//使用RS485时需要让EN = 0;//转换接收状态
    }
    else
    {
      RadarData.Sys_State = RADAR_ERROR;	//雷达数据发送系统错误
      FillRadarDataTxBuf(CmdRadarDataTxBuf, RadarData);
      HAL_UART_Transmit(&huart1, CmdRadarDataTxBuf, 11, 1000);
      DBC_SendDist(&hcan1, MinRangeLong);
			//使用RS485时需要让EN = 0;//转换接收状态
    }
    osDelay(100);
  }
  /* USER CODE END StartRadarDataTxTask */
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
		if(ADAS_dev.LDW_warning==0x01 || ADAS_dev.LDW_warning == 0x02)//左侧或右侧车道偏移报警
		{
      //报警
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
    switch(CrashWarningLv)				//前向碰撞
    {
      case WARNING_HIGH:
				#if ADAS_COMM
				if(0 != ADAS_dev.crash_level)
				{
				#endif
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
					HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
					//WTN6_Broadcast(BELL_BB_500MS);
					osDelay(1000);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
					HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
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
					HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
					//WTN6_Broadcast(BELL_BB_1000MS);
					osDelay(500);
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
					HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
				#if ADAS_COMM
				}
				#endif
        break;
      default:
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
        break;
    }

    #if ADAS_COMM
    switch(ADAS_dev.LDW_warning)	//车道偏移
    {
      case 0x01:	//left
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
        HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
        //WTN6_Broadcast(BELL_BB_500MS);
        osDelay(2000);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
        break;
      case 0x02:	//right
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_SET);
        HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
        //WTN6_Broadcast(BELL_BB_1000MS);
        osDelay(2000);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
        HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
        break;
      default:
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
        break;
    }
    #endif
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
    uint8_t sum = 0;
    uint8_t i = 0;
    uint16_t yawH = 0;
    uint16_t yawL = 0;
    uint16_t yawRateH = 0;
    uint16_t yawRateL = 0;
    for(; i < 5; i++)
      sum += VehicleCANRxBuf[i];
    if(VehicleCANRxBuf[5] == sum)   //校验和
    {
      yawL = VehicleCANRxBuf[1];
      yawH = VehicleCANRxBuf[2];
      yawRateL = VehicleCANRxBuf[3];
      yawRateH = VehicleCANRxBuf[4];
      yaw = ((float)((yawH<<8) | yawL)) / 32768.0f * 180;  //单位是°
      yawRate = ((float)((yawRateH<<8) | yawRateL)) / 32768.0f * 2000;  //单位是°/s
      ARS_SendVehicleYaw(&hcan2, yawRate);  //send VehicleYaw to Radar
    }
		osDelay(10);
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
    if(0xD1 == VehicleCANRxBuf[0] && 0xD1 == VehicleCANRxBuf[2])
    {
      VehicleSpeed = VehicleCANRxBuf[1];	//车速16进制,km/h
			//ARS408
			#if RADAR_TYPE
			ARS_SendVehicleSpeed(&hcan2, VehicleSpeed);	//send VehicleSpeed to Radar
			//EMRR
			#else
			#endif
    }
		osDelay(10);
  }
  /* USER CODE END StartCANSpeedReadTask */
}

void StartCalculateTask(void const * argument)
{
	for(;;)
	{
		osSemaphoreWait(bSemCalculateSigHandle, osWaitForever);
    //ARS408
		#if RADAR_TYPE
		uint16_t MinRange=255;									//初始化为最大距离
		uint32_t relSpeed=0;
		MinRange = RadarGeneral[0].Obj_DistLong;
		relSpeed = RadarGeneral[0].Obj_VrelLong;
		
		if((0.2*MinRange-500) < LIMIT_RANGE && MinRange != 0)	//如果此距离小于一个足够小的距离，再开始计算，否则浪费时间		
		{
			VrelLong = 0.25 * relSpeed - 128;						//获取真实相对速度
			MinRangeLong = 0.2 * MinRange - 500;				//获取真实距离
			TimetoCrash = -(float)MinRangeLong/VrelLong;//相对速度为负
			if(TimetoCrash < 3 && VrelLong < 0 && MinRangeLong > 0)
			{
				CrashWarningLv = WARNING_HIGH;
			}
			else if(TimetoCrash < 3.5f && VrelLong < 0 && MinRangeLong > 0)
			{
				CrashWarningLv = WARNING_LOW;
			}
			else
				CrashWarningLv = WARNING_NONE;
		}

    //EMRR
		#else
		MinRangeLong = EMRRGeneral_Closet.trackRange;
    VrelLong = EMRRGeneral_Closet.trackSpeed;

		if(MinRangeLong < LIMIT_RANGE && MinRangeLong != 0 && VrelLong != 0)
    {
      TimetoCrash = - MinRangeLong / VrelLong;
      if(TimetoCrash < 3 && VrelLong < 0 && MinRangeLong > 0)
      {
        CrashWarningLv = WARNING_HIGH;
      }
      else if(TimetoCrash < 3.5f && VrelLong < 0 && MinRangeLong > 0)
      {
        CrashWarningLv = WARNING_LOW;
      }
      else
        CrashWarningLv = WARNING_NONE;
    }
		#endif
		osSemaphoreRelease(bSemSoundWarningSigHandle);
		osDelay(2);
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
