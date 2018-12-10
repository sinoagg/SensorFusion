#include "aebs.h"
#include "vehicle.h"
#include "user_adas.h"

uint8_t adas_switch = OFF;
uint8_t aebs_switch = OFF;
uint8_t aebs_quit = OFF;

extern DAC_HandleTypeDef hdac;
uint8_t crashWarningLv = WARNING_NONE;
float TimetoCrash_g;
AEBS_Status vAEBS_Status = {OFF, OFF, 0, 0};
#if VEHICLE_MODEL == DONGFENG
CAN_TxHeaderTypeDef CAN_TxXBRHeader={0,VEHICLE_BRAKE_ADDR, CAN_ID_EXT,CAN_RTR_DATA,8,DISABLE};
CAN_TxHeaderTypeDef CAN_TxAEBS1Header={0,VEHICLE_AEBS1_ADDR, CAN_ID_EXT,CAN_RTR_DATA,8,DISABLE};
#endif

/**
 * @brief  clear AEBS status
 * @note   
 * @retval None
 */
void ClearAEBSStatus(void)
{
	crashWarningLv = WARNING_NONE;
	TimetoCrash_g = 20;
	RadarObject.MinRangeLong = 255;
	RadarObject.VrelLong = 0;
}

/**
 * @brief  Stop Buzzer(Beeper)
 * @note   
 * @param  *pAEBS_Status: vAEBS_Stutus address
 * @retval None
 */
void StopBuzzer(AEBS_Status *pAEBS_Status)
{
	if(pAEBS_Status->BuzzerStatus == ON)
	{
		pAEBS_Status->BuzzerStatus = OFF;
		HAL_TIM_Base_Stop_IT(&htim2);
		BUZZER_OFF();
		LED_WARNING_OFF();
	}
}

/**
 * @brief  Start Buzzer(Beeper)
 * @note   
 * @param  *pAEBS_Status: vAEBS_Status address
 * @param  warningLv: crashWarningLv
 * @retval None
 */
void StartBuzzer(AEBS_Status *pAEBS_Status, uint8_t warningLv)
{
	if (warningLv == WARNING_HIGH || warningLv == WARNING_MID)
	{
		pAEBS_Status->BuzzerStatus=ON;
		if (htim2.Init.Period != HIGH_WARNING_TIM_PERIOD)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			MX_TIM2_Init(HIGH_WARNING_TIM_PERIOD);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
	else if (warningLv == WARNING_LOW)
	{
		pAEBS_Status->BuzzerStatus=ON;
		if (htim2.Init.Period != LOW_WARNING_TIM_PERIOD)
		{
			HAL_TIM_Base_Stop_IT(&htim2);
			MX_TIM2_Init(LOW_WARNING_TIM_PERIOD);
			HAL_TIM_Base_Start_IT(&htim2);
		}
	}
}

/**
 * @brief  Disable AEBS
 * @note   
 * @param  *pAEBS_Status: vAEBS_Status address
 * @retval None
 */
void DisableAEBS(AEBS_Status *pAEBS_Status)
{
	if (pAEBS_Status->valveStatus == ON)
	{
		pAEBS_Status->valveStatus = OFF;
		#if VEHICLE_MODEL != DONGFENG
		VALVE_DIS();
		HAL_DACEx_DualSetValue(&hdac, DAC_ALIGN_12B_R, 0, 0);
		#else
		
		#endif
	}
}

/**
 * @brief  Enable AEBS
 * @note   
 * @param  ttc: time to collision 
 * @param  warningLv: crashWarningLv
 * @retval None
 */
void EnableAEBS(float ttc, uint8_t warningLv)
{
	if(aebs_switch == ON && aebs_quit == OFF)
	{
		if (vAEBS_Status.valveStatus == OFF)
			vAEBS_Status.valveStatus = ON;
		#if VEHICLE_MODEL != DONGFENG
		ValveCalc(&hdac, ttc); 
		if (HAL_GPIO_ReadPin(VALVE_ENABLE_GPIO_Port, VALVE_ENABLE_Pin) != GPIO_PIN_SET)
			VALVE_EN(); 
		#else

		#endif
	}
}

/**
 * @brief  Execute AEBS
 * @note   
 * @param  *pAEBS_Status: pointer of AEBS_Status 
 * @param  ttc: time to collision
 * @param  warningLv: 
 * @param  ADAS_Status: 1 for adas warning
 * @retval None
 */
void ExecuteAEBS(AEBS_Status *pAEBS_Status, float ttc, uint8_t warningLv, uint8_t ADAS_Status)
{
	StartBuzzer(pAEBS_Status, warningLv);
	EnableAEBS(ttc, warningLv);
	if(ADAS_Status == 1)
	{
		if(warningLv == WARNING_HIGH || warningLv == WARNING_MID)
			pAEBS_Status->AEBStimes += 1;
		pAEBS_Status->onlyRadarTimes = 20;
	}
}

/**
 * @brief  correct distance(using vehicle speed)
 * @note   
 * @param  speed: read from vehicle can 
 * @param  *RadarRxBuf: Radar CAN Rx buffer
 * @retval None
 */
void CorrectDistance(float speed, uint16_t *RadarRxBuf)
{
	uint16_t dist = 0;
	uint8_t temp;
	dist = (((*(RadarRxBuf + 1)) << 5) | ((*(RadarRxBuf + 2)) >> 3));
	dist -= ((speed / 22) - 0.0f) * 5;
	*(RadarRxBuf + 1) = (dist >> 5);
	temp = ((dist << 3) & 0xF8);
	*(RadarRxBuf + 2) &= 0x07;
	*(RadarRxBuf + 2) |= temp;
}

/** 
 * @brief  Calculate Proportional Relay Valve
 * @note   
 * @param  *hdac: 
 * @retval 0 for ok
 */
uint8_t ValveCalc(DAC_HandleTypeDef *hdac, float ttc)
{
	uint32_t data1;
	if (ttc > HIGH_WARNING_TIME)
		data1 = 0;
	else
	{
		data1 = (HIGH_WARNING_TIME - ttc) / HIGH_WARNING_TIME * 4 / 3.3f * 4096;
		if (data1 > 2 / 3.3f * 4096)
			data1 = 2 / 3.3f * 4096;
	}
	HAL_DACEx_DualSetValue(hdac, DAC_ALIGN_12B_R, data1, data1);
	return 0;
}

/** 
 * @brief  Calculate external brake request
 * @note   
 * @param  *hcan: 
 * @retval 0 for ok
 */
uint16_t XAcc_int = 0;
uint8_t XBRCalc(CAN_HandleTypeDef *hcan, float ttc, uint8_t XBR_Ctrl, float relSpeed, float range)
{
	static float XAcc=0;
	uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[8] = {0};
	uint8_t temp_checksum = 0, i = 0;
	static uint8_t message_counter = 0;
	message_counter += 1;
	message_counter %= 16;
	
//	if(relSpeed > 0)
//	{
//		XAcc = 0; 
//	}
//	else
//	{
//		if(range == 0)
//			range = 0.1f;
//		XAcc = -1.3f * 0.5f * relSpeed * relSpeed / range;
//	}
//	if(XAcc < -0.05f)
//		XAcc = -0.05f;
		
	if (ttc >= HIGH_WARNING_TIME)
	{	
		XAcc += 0.1f*(ttc - HIGH_WARNING_TIME) / HIGH_WARNING_TIME;
		if(XAcc>0) XAcc = 0; 
	}
	else
	{
		XAcc =  7 * (ttc - HIGH_WARNING_TIME) / HIGH_WARNING_TIME;
		//if(XAcc < -1.5f) XAcc = -1.5f;
	}
	XAcc_int = (uint16_t)((XAcc + 15.687f)*2048);			//Acc Demand, Res1/2048, offset -15.687m/s2
	CANTxBuf[0] = XAcc_int & 0xFF;
	CANTxBuf[1] = (XAcc_int >> 8) & 0xFF;
	if(XBR_Ctrl==1)
		CANTxBuf[2] = 0xD0;	//XBR mode, if XBR EBI mode is 00, XBR urgency not using
	else
		CANTxBuf[2] = 0xCC; //0xCC No Effect on brake system
	CANTxBuf[3] = 0xFF;//(-XAcc/10.0f) / 0.004f;	//XBR urgency %, Res 0.4%
	CANTxBuf[4] = 0xFF;
	CANTxBuf[5] = 0xFF;
	CANTxBuf[6] = 0xFF;
	
	for(i = 0;i < 7; i++)
		temp_checksum += CANTxBuf[i];
	temp_checksum += message_counter & 0x0F;
	temp_checksum += 0x2A;//VEHICLE_BRAKE_ADDR & 0x00FF;
	temp_checksum += 0x0B;//(VEHICLE_BRAKE_ADDR & 0xFF00) >> 16;
	temp_checksum += 0x04;//(VEHICLE_BRAKE_ADDR & 0xFF0000) >> 32;
	temp_checksum += 0x0C;//(VEHICLE_BRAKE_ADDR & 0xFF000000) >> 48;
	CANTxBuf[7] = ((((temp_checksum >> 4) + temp_checksum) & 0x0F) << 4) | (message_counter & 0x0F);
	
	HAL_CAN_AddTxMessage(hcan, &CAN_TxXBRHeader, CANTxBuf, &CAN_TxMailBox);
	return 0;
}

/**
 * @brief  Enter Lane Calculate
 * @note   
 * @param  obj: Radar obj 
 * @param  *penterLaneTime: pointer of enterLaneTime
 * @retval 
 */
uint8_t EnterLaneCalc(ObjectTypeDef obj, float *pEnterLaneTime)
{
	if (obj.VrelLat != 0)
		*pEnterLaneTime = -obj.MinRangeLat / obj.VrelLat;

	if (obj.MinRangeLat < LANEWIDTH && obj.MinRangeLat > -LANEWIDTH)
	{
		return 1;
	}//--obj in lane
	else if((*pEnterLaneTime < ENTER_LANE_TIME_THRESHOLD) && (*pEnterLaneTime > 0))
	{
		return 1;
	}//--obj entering lane
	else
		return 0;
}

/**
 * @brief  prepare AEBS1 data
 * @note   
 * @param  *hcan: vehicle can
 * @param  brakeSysState: 
 * @param  warningLv: 
 * @param  objectDetected: 
 * @param  ttc: time to collison
 * @param  *object: RadarObject
 * @retval 0 for OK
 */
uint8_t PrePareAEBS1Data(CAN_HandleTypeDef *hcan, uint8_t brakeSysState, uint8_t warningLv, uint8_t objectDetected, float ttc, ObjectTypeDef *object)
{
	uint8_t CANTxBuf[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
	CANTxBuf[0]=(warningLv<<4)|brakeSysState;
	CANTxBuf[1] &= 0xF8;
	CANTxBuf[1] |= objectDetected&0x07;
	CANTxBuf[1] |= (ADAS_dev.crash_level << 4) & 0x30;
	CANTxBuf[2] = (int8_t)(-object->VrelLong / 0.25f);
	CANTxBuf[3] = XAcc_int & 0xFF;
	CANTxBuf[4] = (XAcc_int >> 8) & 0xFF;
	CANTxBuf[5] = (uint8_t)(ttc / 0.05f);
	CANTxBuf[6] = (uint8_t)object->MinRangeLong;
	
	HAL_CAN_AddTxMessage(hcan, &CAN_TxAEBS1Header, CANTxBuf, &CAN_TxMailBox);
	return 0;
}
