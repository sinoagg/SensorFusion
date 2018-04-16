#include "ARS408.h"

CAN_TxHeaderTypeDef CAN_TxConfigRadarHeader={RADAR_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_TxHeaderTypeDef CAN_TxConfigFilterHeader={FILTER_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};	

uint8_t ARS_Init(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef MW_RadarCANFilter={};

	HAL_CAN_ConfigFilter(hcan, &MW_RadarCANFilter);
	HAL_CAN_Start(hcan);
	#ifdef CONFIG_ARS408_RADAR
		ARS_ConfigRadar(hcan);
	#endif
	#ifdef CONFIG_ARS408_FILTER
		ARS_ConfigFilter(hcan);
	#endif
}

uint8_t ARS_ConfigRadar(CAN_HandleTypeDef *hcan)
{
	uint32_t CAN_TxMailBox=CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[8]={0};
	CANTxBuf[0]=RadarConfig.StoreInNVM_valid|RadarConfig.SortIndex_valid|RadarConfig.OutputType_valid|RadarConfig.RadarPower_valid|RadarConfig.MaxDistance_valid;
	CANTxBuf[1]=RadarConfig.MaxDistance>>2;
	CANTxBuf[2]=RadarConfig.MaxDistance<<6&0xFF;
	CANTxBuf[3]=RadarConfig.RadarPower|RadarConfig.OutputType;
	CANTxBuf[4]=RadarConfig.StoreInNVM|RadarConfig.SortIndex;
	CANTxBuf[5]=RadarConfig.RCS_Threshold|RadarConfig.RCS_Threshold_valid;
	//CAN总线发送配置
	HAL_CAN_AddTxMessage(hcan, &CAN_TxConfigRadarHeader, CANTxBuf, &CAN_TxMailBox);
	return 0;
}

uint8_t ARS_ConfigFilter(CAN_HandleTypeDef *hcan)
{
	uint32_t CAN_TxMailBox=CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[8]={0};
	CANTxBuf[0]=RadarFilterConfig.FilterCfg_Type|RadarFilterConfig.FilterCfg_Index|RadarFilterConfig.FilterCfg_Active|RadarFilterConfig.FilterCfg_Valid;
	CANTxBuf[1]=RadarFilterConfig.FilterCfg_Min_XXX>>8;
	CANTxBuf[2]=RadarFilterConfig.FilterCfg_Min_XXX&0xFF;
	CANTxBuf[3]=RadarFilterConfig.FilterCfg_Max_XXX>>8;
	CANTxBuf[4]=RadarFilterConfig.FilterCfg_Max_XXX&0xFF;
	//CAN总线发送配置
	HAL_CAN_AddTxMessage(hcan, &CAN_TxConfigFilterHeader, CANTxBuf, &CAN_TxMailBox);
	return 0;
}

void ARS_GetRadarObjStatus(uint8_t* pCANRxBuf)
{
	RadarObjStatus.Obj_NofObjects=*pCANRxBuf;
	RadarObjStatus.Obj_MeasCounter=((uint16_t)*(pCANRxBuf+1))<<8|*(pCANRxBuf+2);
}

void ARS_GetRadarObjGeneral(uint8_t* pCANRxBuf, MW_Radar_General *pRadarGeneral)
{
	(pRadarGeneral+(*pCANRxBuf))->Obj_ID=*pCANRxBuf;	//OBJ_ID
	(pRadarGeneral+(*pCANRxBuf))->Obj_DistLong= (((uint16_t)*(pCANRxBuf+1))<<8|(*(pCANRxBuf+2)<<3))>>3;
	(pRadarGeneral+(*pCANRxBuf))->Obj_DistLat= ((uint16_t)*(pCANRxBuf+2)&0x07)<<8|(*(pCANRxBuf+3));
	(pRadarGeneral+(*pCANRxBuf))->Obj_VrelLong= (((uint16_t)*(pCANRxBuf+4))<<8|(*(pCANRxBuf+5)<<5))>>5;
	(pRadarGeneral+(*pCANRxBuf))->Obj_VrelLat= (((uint16_t)*(pCANRxBuf+5)&0x3F)<<8|(*(pCANRxBuf+6)&0xE0))>>5;;
	(pRadarGeneral+(*pCANRxBuf))->Obj_DynProp= *(pCANRxBuf+6)&0x07;
	(pRadarGeneral+(*pCANRxBuf))->Obj_RCS= *(pCANRxBuf+7);
}



