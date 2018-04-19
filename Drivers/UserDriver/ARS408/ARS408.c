#include "ARS408.h"

CAN_TxHeaderTypeDef CAN_TxConfigRadarHeader={RADAR_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_TxHeaderTypeDef CAN_TxConfigFilterHeader={FILTER_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};	

uint8_t ARS_Init(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef MW_RadarCANFilter;//={};

	HAL_CAN_ConfigFilter(hcan, &MW_RadarCANFilter);
	HAL_CAN_Start(hcan);
	#ifdef CONFIG_ARS408_RADAR
		ARS_ConfigRadar(hcan);
	#endif
	#ifdef CONFIG_ARS408_FILTER
		ARS_ConfigFilter(hcan);
	#endif
	return 0;
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

/**
 * [config Radar Filter]
 * @param  hcan [hcan index]
 * @return      []
 */
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
	(pRadarGeneral+(*pCANRxBuf))->Obj_VrelLong= (((uint16_t)*(pCANRxBuf+4))<<8|(*(pCANRxBuf+5)<<5))>>5;//纵向相对速度
	(pRadarGeneral+(*pCANRxBuf))->Obj_VrelLat= (((uint16_t)*(pCANRxBuf+5)&0x3F)<<8|(*(pCANRxBuf+6)&0xE0))>>5;;//横向相对速度
	(pRadarGeneral+(*pCANRxBuf))->Obj_DynProp= *(pCANRxBuf+6)&0x07;		//目标动态特性（运动还是静止）
	(pRadarGeneral+(*pCANRxBuf))->Obj_RCS= *(pCANRxBuf+7);
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
uint8_t FindMIObj(struct Task *taskMsg)
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

    //struct ObjList ARS_ObjList;
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
}

