#include "aebs.h"

extern DAC_HandleTypeDef hdac;
uint8_t CrashWarningLv = WARNING_NONE;
float TimetoCrash_g;
AEBS_Status vAEBS_Status={OFF, OFF};

void StopBeeper(AEBS_Status* pAEBS_Status)
{
	//if(pAEBS_Status->beeperStatus==ON)
	//{
			pAEBS_Status->beeperStatus=OFF;
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin, GPIO_PIN_SET);
	//}
}

void StartBeeper(uint8_t warningLv)
{
	if(warningLv==WARNING_HIGH||warningLv==WARNING_MID)
	{
		if(htim2.Init.Period!=HIGH_WARNING_TIM_PERIOD)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			MX_TIM2_Init(HIGH_WARNING_TIM_PERIOD);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
	else if(warningLv==WARNING_LOW)
	{
		if(htim2.Init.Period!=LOW_WARNING_TIM_PERIOD)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			MX_TIM2_Init(LOW_WARNING_TIM_PERIOD);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
}

void DisableAEBS(AEBS_Status* pAEBS_Status)
{
	if(pAEBS_Status->valveStatus==ON)
	{
			pAEBS_Status->valveStatus=OFF;
			HAL_GPIO_WritePin(VALVE_ENABLE_GPIO_Port, VALVE_ENABLE_Pin, GPIO_PIN_RESET);	//禁止刹车电源
			HAL_DACEx_DualSetValue(&hdac, DAC_ALIGN_12B_R, 0, 0);			//清空DAC输出
	}
}

void EnableAEBS(float ttc, uint8_t warningLv)
{
	if(vAEBS_Status.valveStatus==OFF)
		vAEBS_Status.valveStatus=ON;
	ValveCalc(&hdac, ttc);			//计算刹车强度
	if(HAL_GPIO_ReadPin(VALVE_ENABLE_GPIO_Port, VALVE_ENABLE_Pin)!=GPIO_PIN_SET)					
		HAL_GPIO_WritePin(VALVE_ENABLE_GPIO_Port, VALVE_ENABLE_Pin, GPIO_PIN_SET);		//使能刹车电源
}

/** 
 * @brief  Calculate Proportional Relay Valve
 * @note   
 * @param  *hdac: 
 * @retval 0 for ok
 */
uint8_t ValveCalc(DAC_HandleTypeDef* hdac, float ttc)
{
	uint32_t data1;
	if(ttc > HIGH_WARNING_TIME)
		data1 = 0;
	else
	{
		data1 = (HIGH_WARNING_TIME - ttc) /HIGH_WARNING_TIME * 4/3.3f * 4096;
		if(data1>2/3.3f * 4096)
			data1=2/3.3f * 4096;
	}
	HAL_DACEx_DualSetValue(hdac, DAC_ALIGN_12B_R, data1, data1);
  return 0;
}

