/**
 * microwave radar - ARS408 by Continental
 * ---init & config
 * ARS_Init(hcan): start CAN2 for radar & (optional) config Radadr and RadarFilter
 * ARS_ConfigRadar(hcan): config Radar through can2
 * ARS_ConfigFilter(hcan): config Radar filter through can2
 * ---read Radar data
 * ARS_GetRadarObjStatus(...): read Radar Obj Status from can2, need to config can2 first
 * ARS_GetRadarObjGeneral(...): read Radar Obj General(distance, velocity...) from can2
 */
#include "ARS408.h"

CAN_TxHeaderTypeDef CAN_TxConfigRadarHeader={RADAR_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_TxHeaderTypeDef CAN_TxConfigFilterHeader={FILTER_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};	


uint8_t ARS_Init(CAN_HandleTypeDef *hcan)
{
	//配置CAN滤波器接收Objct_General信息，即相对目标的距离、速度等
	CAN_FilterTypeDef MW_RadarCANFilter={OBJ_GENERAL_ADDR<<5,0,0xEFE<<5,0,CAN_FILTER_FIFO0, 14, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,14};		//0x60B 和 0x60A同时检测
	//CAN_FilterTypeDef MW_RadarCANFilter = {0,OBJ_GENERAL_ADDR,0,0xEFF,CAN_FILTER_FIFO0,CAN_FILTERMODE_IDLIST,CAN_FILTERSCALE_32BIT,ENABLE,0};
	HAL_CAN_ConfigFilter(hcan, &MW_RadarCANFilter);
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	#ifdef CONFIG_ARS408_RADAR
		ARS_ConfigRadar(hcan);
		HAL_Delay(100);
	#endif
	#ifdef CONFIG_ARS408_FILTER
		ARS_ConfigFilter(hcan);
		HAL_Delay(100);
	#endif

	return 0;
}

void deRadarConfig()
{
	RadarConfig.MaxDistance_valid = RADARCFG_MAXDISTANCE_VALID;
  RadarConfig.SensorID_valid = RADARCFG_SENSORID_VALID;
  RadarConfig.RadarPower_valid = RADARCFG_RADARPOWER_VALID;
  RadarConfig.OutputType_valid = RADARCFG_OUTPUTTYPE_VALID;
  RadarConfig.SendQuality_valid = RADARCFG_SENDQUALITY_VALID;
  RadarConfig.SendExtInfo_valid = RADARCFG_SENDEXTINFO_VALID;
  RadarConfig.SortIndex_valid = RADARCFG_SORTINDEX_VALID;
  RadarConfig.StoreInNVM_valid = RADARCFG_STOREINNVM_VALID;
  RadarConfig.MaxDistance = RADARCFG_MAXDISTANCE;
  RadarConfig.SensorID = RADARCFG_SENSORID;
  RadarConfig.OutputType = RADARCFG_OUTPUTTYPE_OBJ;
  RadarConfig.RadarPower = RADARCFG_RADARPOWER_STD;
  RadarConfig.CtrlRelay_valid = RADARCFG_CTRLRELEY_VALID;
  RadarConfig.CtrlRelay = RADARCFG_CTRLRELEY;
  RadarConfig.SendQuality = RADARCFG_SENDQUALITY;
  RadarConfig.SendExtInfo = RADARCFG_SENDEXTINFO;
  RadarConfig.SortIndex = RADARCFG_SORTINDEX_RANGE;
  RadarConfig.StoreInNVM = RADARCFG_STOREINNVM;
  RadarConfig.RCS_Threshold_valid = RADARCFG_RCS_THRES_VALID;
  RadarConfig.RCS_Threshold = RADARCFG_RCSTHRES_HIGHSENSE;
}

void deRadarFilterConfig()
{
  RadarFilterConfig.FilterCfg_Valid = FILTERCFG_VALID;
  RadarFilterConfig.FilterCfg_Active = FILTERCFG_FILTERACTIVE_VALID;
  RadarFilterConfig.FilterCfg_Index = FILTERCFG_INDEX_DISTANCE;
  RadarFilterConfig.FilterCfg_Type = FILTERCFG_TYPE_OBJ;
}

uint8_t ARS_ConfigRadar(CAN_HandleTypeDef *hcan)
{
	uint32_t CAN_TxMailBox=CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[8]={0};
  deRadarConfig();
  //if change RadarConfig struct, add RadarConfig.*** = someValue here
	CANTxBuf[0]=RadarConfig.StoreInNVM_valid|RadarConfig.SortIndex_valid|RadarConfig.SendExtInfo_valid|RadarConfig.SendQuality_valid|\
		RadarConfig.OutputType_valid|RadarConfig.RadarPower_valid|RadarConfig.SensorID_valid|RadarConfig.MaxDistance_valid;
	CANTxBuf[1]=RadarConfig.MaxDistance>>2;
	CANTxBuf[2]=RadarConfig.MaxDistance<<6 & 0xC0;
	CANTxBuf[4]=RadarConfig.RadarPower|RadarConfig.OutputType|RadarConfig.SensorID;
	CANTxBuf[5]=RadarConfig.StoreInNVM|RadarConfig.SortIndex|RadarConfig.SendExtInfo|RadarConfig.SendQuality|RadarConfig.CtrlRelay|RadarConfig.CtrlRelay_valid;
	CANTxBuf[6]=RadarConfig.RCS_Threshold|RadarConfig.RCS_Threshold_valid;
	//CAN总线发送配置
	HAL_CAN_AddTxMessage(hcan, &CAN_TxConfigRadarHeader, CANTxBuf, &CAN_TxMailBox);
	return 0;
}

uint8_t ARS_ConfigFilter(CAN_HandleTypeDef *hcan)
{
	uint32_t CAN_TxMailBox=CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[8]={0};
  deRadarFilterConfig();
  //if change FilterConfig struct, add RadarFilterConfig.*** = someValue here
	CANTxBuf[0]=RadarFilterConfig.FilterCfg_Type|RadarFilterConfig.FilterCfg_Index|RadarFilterConfig.FilterCfg_Active|RadarFilterConfig.FilterCfg_Valid;
	CANTxBuf[1]=RadarFilterConfig.FilterCfg_Min_XXX>>8 & 0x1F;
	CANTxBuf[2]=RadarFilterConfig.FilterCfg_Min_XXX&0xFF;
	CANTxBuf[3]=RadarFilterConfig.FilterCfg_Max_XXX>>8 & 0x1F;
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

void ARS_GetRadarObjGeneral(uint8_t* pCANRxBuf, MW_RadarGeneral *pRadarGeneral)
{
	(pRadarGeneral)->Obj_ID=*pCANRxBuf;	//OBJ_ID
	(pRadarGeneral)->Obj_DistLong= (((uint16_t)*(pCANRxBuf+1))<<8|*(pCANRxBuf+2))>>3;
	(pRadarGeneral)->Obj_DistLat= ((uint16_t)*(pCANRxBuf+2)&0x07)<<8|(*(pCANRxBuf+3));
	(pRadarGeneral)->Obj_VrelLong= (((uint16_t)*(pCANRxBuf+4))<<8|*(pCANRxBuf+5))>>6;//纵向相对速度
	(pRadarGeneral)->Obj_VrelLat= (((uint16_t)*(pCANRxBuf+5)&0x3F)<<8|(*(pCANRxBuf+6)&0xE0))>>5;;//横向相对速度
	(pRadarGeneral)->Obj_DynProp= *(pCANRxBuf+6)&0x07;		//目标动态特性（运动还是静止）
	(pRadarGeneral)->Obj_RCS= *(pCANRxBuf+7);
}

/*
--> Find mostImportantObject  <--
 # Safe (green): There is no car in the ego lane (no MIO), the MIO is
   moving away from the car, or the distance is maintained constant.
 # Caution (yellow): The MIO is moving closer to the car, but is still at
   a distance above the FCW distance. FCW distance is calculated using the
   Euro NCAP AEB Test Protocol. Note that this distance varies with the
   relative speed between the MIO and the car, and is greater when the
   closing speed is higher.
 # Warn (red): The MIO is moving closer to the car, and its distance is
   less than the FCW distance.
*/
/*
uint8_t FindMIObj(MW_RadarGeneral *pRadarGeneral)
{
	uint8_t FCW = 0;
	uint8_t Find0x60B = 0;
	uint16_t i = 0;
    float MinRange = 0.0;
    float MaxRange = MAX_RANGE;
    float gAccel = 9.8;
    float maxDeceleration = 0.4 * gAccel;		//假设车辆最大减速度是0.4g
    float delayTime = 1.2;
    float relSpeed = 0.0;

    //ARS_GetRadarObjStatus(CANRxBuf);
    //ARS_GetRadarObjGeneral(CANRxBuf, RadarGeneral);
    
    for(i = 0; i < taskMsg->TaskMsgNum.Index[isRead] ; i++) //    for(i = 0; i < taskMsg->TaskMsgNum.Index[isRead] + 1; i++)
    {
        if(taskMsg->RxMessage[isRead][i].StdId == 0x60B)
        {
        	ARS_GetRadarObjGeneral(CANRxBuf, RadarGeneral);
            //ARS_ObjList = ARS_Obj_Handle(&ARSCANmsg.RxMessage[isRead][i]);
            //if(RadarGeneral.Obj_DynProp == 0x2)//0x2 means oncoming
            if( ARS_ObjList.Obj_LongDispl != 0 &&((ABS((ARS_ObjList.Obj_LatDispl) * 2.0)) < LANEWIDTH ) &&//是否在车道内
                    ARS_ObjList.Obj_LongDispl < MaxRange)//
            {
                MinRange = RadarGeneral.Obj_LongDispl;						//MaxRange赋值纵向最小距离
                MaxRange = MinRange;
                relSpeed = RadarGeneral.Obj_VrelLong;
            }
        }
    }

    Segment_Num = MinRange;
    if(MinRange == 0)
    {
        return FCW = 0;			//如果这个距离为0，则没有FCW报警
    }
    else if(MinRange > 0)		//如果此距离大于0，说明有可能有报警
    {
        if(relSpeed < 0)		//如果距离在靠近
        {
        	//计算刹车距离
            float distance = relSpeed * (-1)  * delayTime +  relSpeed * relSpeed / 2 / maxDeceleration;//物理公式-vt+v*v/2/a 计算距离

            if(MinRange <= distance)
            {
                return  FCW = 2; //red			//如果太近，红色警报		
            }
            else
            {
                return  FCW = 1; //yellow		//否则以最大减速度能刹住，但也是在报警距离范围之内了
            }
        }
        else
        {
            //there is a stationary object in front of the vehilce .//如果远离或者静止的障碍物
            return FCW = 0;
        }
    }
}*/



