/**
 * @Author: wlm 
 * @Date: 2018-04-19 11:54:40 
 * @Last Modified by: wlm
 * @Last Modified time: 2018-08-20 11:59:14
 */

/**
 * microwave radar - ARS408 by Continental
 * 
 * ARS_***() functions are entries of this driver
 * ***_func() functions are practical functions called by ARS_***() functions
 * 
 * ---init & config
 * ARS_Init(hcan): start CAN for radar & (optional) config Radadr and RadarFilter
 * ARS_ConfigRadar(hcan): config Radar through can
 * ARS_ConfigFilter(hcan): config Radar filter through cano
 * 
 * //called by ARS_***() functions mentioned above, no entry outside this file
 * RadarConfig_func(): fill RadarConfig struct with #define in ARS408.h
 * RadarFilterConfig_func(index): fill RadarFilterConfig struct with #define in ARS408.h
 * FilterContentCfg_func(hcan, index, filter_min, filter_max): fill CANTxBuf[] && send 1 content of filter to can
 * 
 * ---read Radar data
 * ARS_GetRadarObjStatus(...): read Radar Obj Status from can, need to config can first
 * ARS_GetRadarObjGeneral(...): read Radar Obj General(distance, relVelocity...) from can
 * 
 * ---send Vehicle speed & yaw
 * ARS_SendVehicleSpeed(...): send VehicleSpeed through can, speed read from Vehicle-can
 * ARS_SendVehicleYaw(...): send VehicleYaw through can, yaw read from can3
 */
#include "ARS408.h"
#include "math.h"

#define CONFIG_ARS408_RADAR 0
#define CONFIG_ARS408_FILTER 0

#define VEHICLE_CENTRE_LEN	10.0f
#define VEHICLE_HALF_WIDTH	1.4f
#define OBSTACLE_ERR				-0.8f

CAN_TxHeaderTypeDef CAN_TxConfigRadarHeader={RADAR_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_TxHeaderTypeDef CAN_TxConfigFilterHeader={FILTER_CFG_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_TxHeaderTypeDef CAN_TxYawHeader={YAW_INFO_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};
CAN_TxHeaderTypeDef CAN_TxSpeedHeader={SPEED_INFO_ADDR,0,CAN_ID_STD,CAN_RTR_DATA,8,DISABLE};

MW_RadarConfig RadarConfig;
MW_RadarFilterConfig RadarFilterConfig;
MW_RadarFilterIndexContent FilterContent;
MW_RadarSpeed RadarSpeed;

/** 
 * @brief  ARS408 Radar Init(can & filter config, Radar&filter config)
 * @note   only using obj data(filtered by can)
 * @param  *hcan: can2(500kbps)
 * @retval 0 for ok
 */
uint8_t ARS_Init(CAN_HandleTypeDef *hcan)
{
	//config CAN filter to receive Objct_General(distance & relSpeed
	//ID_HIGH, ID_LOW,\
	MASK_HIGH, MASK_LOW,\
	FIFO 0/1, filter_bank(0-13/14-27), filter_mode(LIST/MASK), filter_scale, EN/DISABLE filter, SlaveStartFilterBank
	CAN_FilterTypeDef MW_RadarCANFilter = {
		OBJ_GENERAL_ADDR<<5, 0,\
		0xEFE<<5, 0,\
		CAN_FILTER_FIFO0, 13, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,13};		//0x60B & 0x60A at the same time
	HAL_CAN_ConfigFilter(hcan, &MW_RadarCANFilter);
		
	/*CAN_FilterTypeDef MW_RadarCANFilter1 = {
		0x201<<5, 0,\
		0xEFE<<5, 0,\
		CAN_FILTER_FIFO0, 14, CAN_FILTERMODE_IDMASK,CAN_FILTERSCALE_32BIT,ENABLE,15};		//0x60B & 0x60A at the same time
	HAL_CAN_ConfigFilter(hcan, &MW_RadarCANFilter1);*/
	
	HAL_CAN_Start(hcan);
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	#if CONFIG_ARS408_RADAR
		ARS_ConfigRadar(hcan);
		HAL_Delay(100);
	#endif
	#if CONFIG_ARS408_FILTER
		ARS_ConfigFilter(hcan);
		HAL_Delay(100);
	#endif

	return 0;
}

/* Radar Config------------------------------------------------------------------*/
void RadarConfig_func()
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

/** 
 * @brief  config radar
 * @note   
 * @param  *hcan: 
 * @retval 0 for ok
 */
uint8_t ARS_ConfigRadar(CAN_HandleTypeDef *hcan)
{
	uint32_t CAN_TxMailBox=CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[8]={0};
  RadarConfig_func();
  //if change RadarConfig struct, add RadarConfig.*** = someValue here
	CANTxBuf[0]=RadarConfig.StoreInNVM_valid|RadarConfig.SortIndex_valid|RadarConfig.SendExtInfo_valid|RadarConfig.SendQuality_valid|\
		RadarConfig.OutputType_valid|RadarConfig.RadarPower_valid|RadarConfig.SensorID_valid|RadarConfig.MaxDistance_valid;
	CANTxBuf[1]=RadarConfig.MaxDistance>>2;
	CANTxBuf[2]=(RadarConfig.MaxDistance<<6) & 0xC0;
	CANTxBuf[4]=RadarConfig.RadarPower|RadarConfig.OutputType|RadarConfig.SensorID;
	CANTxBuf[5]=RadarConfig.StoreInNVM|RadarConfig.SortIndex|RadarConfig.SendExtInfo|RadarConfig.SendQuality|RadarConfig.CtrlRelay|RadarConfig.CtrlRelay_valid;
	CANTxBuf[6]=RadarConfig.RCS_Threshold|RadarConfig.RCS_Threshold_valid;
	//CAN总线发送配置
	HAL_CAN_AddTxMessage(hcan, &CAN_TxConfigRadarHeader, CANTxBuf, &CAN_TxMailBox);
	return 0;
}

/* Filter Config------------------------------------------------------------------*/
void RadarFilterConfig_func(uint8_t index)
{
	RadarFilterConfig.FilterCfg_Valid = FILTERCFG_VALID;
	RadarFilterConfig.FilterCfg_Active = FILTERCFG_FILTERACTIVE_VALID;
	RadarFilterConfig.FilterCfg_Index = index;
	RadarFilterConfig.FilterCfg_Type = FILTERCFG_TYPE_OBJ;
}

/**
 * @brief  [config content of filter]
 * @param  hcan       [hcan index]
 * @param  index      [filter index]
 * @param  filter_min [content min]
 * @param  filter_max [content max]
 * @retval            [ok]
 */
uint8_t FilterContentCfg_func(CAN_HandleTypeDef *hcan, uint8_t index, uint16_t filter_min, uint16_t filter_max)
{
	HAL_Delay(10);
	uint32_t CAN_TxMailBox=CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[8]={0};
	uint8_t Valid_Bits;
	if(index == 0x0A)
		Valid_Bits = 0x1F;
	else
		Valid_Bits = 0x0F;
  RadarFilterConfig_func(index);
	CANTxBuf[0] = RadarFilterConfig.FilterCfg_Type|RadarFilterConfig.FilterCfg_Index|RadarFilterConfig.FilterCfg_Active|RadarFilterConfig.FilterCfg_Valid;
	CANTxBuf[1] = (filter_min>>8) & Valid_Bits;
	CANTxBuf[2] = filter_min & 0xFF;
	CANTxBuf[3] = (filter_max>>8) & Valid_Bits;
	CANTxBuf[4] = filter_max & 0xFF;
	HAL_CAN_AddTxMessage(hcan, &CAN_TxConfigFilterHeader, CANTxBuf, &CAN_TxMailBox);
	return 0;
}

uint8_t ARS_ConfigFilter(CAN_HandleTypeDef *hcan)
{
  FilterContent.FilterCfg_Min_NofObj = 0;
  FilterContent.FilterCfg_Max_NofObj = 1;
  FilterContent.FilterCfg_Min_Distance = (uint16_t)((0 - 0) / 0.1);     //0~200m, offset 0, Res 0.1
  FilterContent.FilterCfg_Max_Distance = (uint16_t)((200 - 0) / 0.1);
  FilterContent.FilterCfg_Min_Azimuth = (uint16_t)((-30 + 50) / 0.025);  //-9°~9°，offset -50, Res 0.025
  FilterContent.FilterCfg_Max_Azimuth = (uint16_t)((30 + 50) / 0.025);
  FilterContent.FilterCfg_Min_VrelOncome = (uint16_t)((0 - 0) / 0.0315); //0.1~100m/s, offset 0, Res 0.0315
  FilterContent.FilterCfg_Max_VrelOncome = (uint16_t)((100 - 0) / 0.0315);
  FilterContent.FilterCfg_Min_VrelDepart = (uint16_t)((0 - 0) / 0.0315); //0.1~100m/s, offset 0, Res 0.0315
  FilterContent.FilterCfg_Max_VrelDepart = (uint16_t)((100 - 0) / 0.0315);
  FilterContent.FilterCfg_Min_RCS = (uint16_t)((-20 + 50) / 0.025);   //0.025~30dBm2, offset -50, Res 0.025
  FilterContent.FilterCfg_Max_RCS = (uint16_t)((52.375 + 50) / 0.025);
  FilterContent.FilterCfg_Min_Lifetime = (uint16_t)((0.1 - 0) / 0.1);   //0.1~409.5s, offset 0, Res 0.1
  FilterContent.FilterCfg_Max_Lifetime = (uint16_t)((409.5 - 0) / 0.1);
  FilterContent.FilterCfg_Min_Size = (uint16_t)((0 - 0) / 0.025);   //0.025~20m2, offset 0, Res 0.025
  FilterContent.FilterCfg_Max_Size = (uint16_t)((102.375 - 0) / 0.025);
  FilterContent.FilterCfg_Min_ProbExists = 0x4;//99%~100%, 0x0: 0%, 0x1: 25%, 0x2: 50%
  FilterContent.FilterCfg_Max_ProbExists = 0x7;//0x3: 75%, 0x4: 90%, 0x5: 99%, 0x6: 99.9%, 0x7: 100%
  FilterContent.FilterCfg_Min_Y = (uint16_t)((-1.5 + 409.5) / 0.2);     //-1.5~1.5m, offset -409.5, Res 0.2
  FilterContent.FilterCfg_Max_Y = (uint16_t)((1.5 + 409.5) / 0.2);
  FilterContent.FilterCfg_Min_X = (uint16_t)((0 + 500) / 0.2);          //0~200m, offset -500, Res 0.2
  FilterContent.FilterCfg_Max_X = (uint16_t)((200 + 500) / 0.2);
  FilterContent.FilterCfg_Min_VYRightLeft = (uint16_t)((0 - 0) / 0.0315);//0.1~100m/s, offset 0, Res 0.0315
  FilterContent.FilterCfg_Max_VYRightLeft = (uint16_t)(100 - 0) / 0.0315;
  FilterContent.FilterCfg_Min_VXOncome = (uint16_t)((0 - 0) / 0.0315);   //0.1~100m/s, offset 0, Res 0.0315
  FilterContent.FilterCfg_Max_VXOncome = (uint16_t)((100 - 0) / 0.0315);
  FilterContent.FilterCfg_Min_VYLeftRight = (uint16_t)((0 - 0) / 0.0315);//0.1~100m/s, offset 0, Res 0.0315
  FilterContent.FilterCfg_Max_VYLeftRight = (uint16_t)((100 - 0) / 0.0315);
  FilterContent.FilterCfg_Min_VXDepart = (uint16_t)((0 - 0) / 0.0315);   //0.1~100m/s, offset 0, Res 0.0315
  FilterContent.FilterCfg_Max_VXDepart = (uint16_t)((100 - 0) / 0.0315);
  FilterContent.FilterCfg_Min_Object_Class = 0x0;//0x0: point, 0x1: car, 0x2: truck, 0x3: not used
  FilterContent.FilterCfg_Max_Object_Class = 0x2;//0x4: motorcyc, 0x5: bicycle, 0x6: wide, 0x7: reserved

  FilterContentCfg_func(hcan, FILTERCFG_INDEX_NOFOBJ, FilterContent.FilterCfg_Min_NofObj, FilterContent.FilterCfg_Max_NofObj);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_DISTANCE, FilterContent.FilterCfg_Min_Distance, FilterContent.FilterCfg_Max_Distance);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_AZIMUTH, FilterContent.FilterCfg_Min_Azimuth, FilterContent.FilterCfg_Max_Azimuth);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_VRELONCOME, FilterContent.FilterCfg_Min_VrelOncome, FilterContent.FilterCfg_Max_VrelOncome);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_VRELDEPART, FilterContent.FilterCfg_Min_VrelDepart, FilterContent.FilterCfg_Max_VrelDepart);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_RCS, FilterContent.FilterCfg_Min_RCS, FilterContent.FilterCfg_Max_RCS);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_LIFETIME, FilterContent.FilterCfg_Min_Lifetime, FilterContent.FilterCfg_Max_Lifetime);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_SIZE, FilterContent.FilterCfg_Min_Size, FilterContent.FilterCfg_Max_Size);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_PROBEXIST, FilterContent.FilterCfg_Min_ProbExists, FilterContent.FilterCfg_Max_ProbExists);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_Y, FilterContent.FilterCfg_Min_Y, FilterContent.FilterCfg_Max_Y);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_X, FilterContent.FilterCfg_Min_X, FilterContent.FilterCfg_Max_X);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_VYRIGHTLEFT, FilterContent.FilterCfg_Min_VYRightLeft, FilterContent.FilterCfg_Max_VYRightLeft);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_VXONCOME, FilterContent.FilterCfg_Min_VXOncome, FilterContent.FilterCfg_Max_VXOncome);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_VYLEFTRIGHT, FilterContent.FilterCfg_Min_VYLeftRight, FilterContent.FilterCfg_Max_VYLeftRight);
  FilterContentCfg_func(hcan, FILTERCFG_INDEX_VXDEPART, FilterContent.FilterCfg_Min_VXDepart, FilterContent.FilterCfg_Max_VXDepart);
  //FilterContentCfg_func(hcan, FILTERCFG_INDEX_OBJCLASS, FilterContent.FilterCfg_Min_Object_Class, FilterContent.FilterCfg_Max_Object_Class);

  return 0;
}

/* Get Object Status------------------------------------------------------------------*/
void ARS_GetRadarObjStatus(uint8_t* pCANRxBuf, MW_RadarObjStatus *pRadarObjStatus)
{
	pRadarObjStatus->Obj_NofObjects = *pCANRxBuf;
	pRadarObjStatus->Obj_MeasCounter = (uint16_t)(((*(pCANRxBuf+1))<<8)|(*(pCANRxBuf+2)));
}

/* Get Object General-----------------------------------------------------------------*/
void ARS_GetRadarObjGeneral(uint8_t* pCANRxBuf, MW_RadarGeneral *pRadarGeneral)
{
	(pRadarGeneral)->Obj_ID = *pCANRxBuf;	//OBJ_ID
	(pRadarGeneral)->Obj_DistLong = (uint16_t)(((*(pCANRxBuf+1))<<5) | ((*(pCANRxBuf+2))>>3));
	(pRadarGeneral)->Obj_DistLat = (uint16_t)((((*(pCANRxBuf+2))&0x07)<<8) | (*(pCANRxBuf+3)));
	(pRadarGeneral)->Obj_VrelLong = (uint16_t)(((*(pCANRxBuf+4))<<2) | ((*(pCANRxBuf+5))>>6));//纵向相对速度
	(pRadarGeneral)->Obj_VrelLat = (uint16_t)((((*(pCANRxBuf+5))&0x3F)<<3) | (((*(pCANRxBuf+6))&0xE0)>>5));//横向相对速度
	(pRadarGeneral)->Obj_DynProp = (*(pCANRxBuf+6))&0x07;		//目标动态特性（运动还是静止）
	(pRadarGeneral)->Obj_RCS = *(pCANRxBuf+7);
}

/* Send Vehicle Speed & gyro yawRate------------------------------------------------------*/
/** 
 * @brief  send VehicleSpeed to ARS408
 * @note   correction Radar in turning corners
 * @param  *hcan: can2(500kbps)
 * @param  VehicleSpeed: (km/h)
 * @retval None
 */
void ARS_SendVehicleSpeed(CAN_HandleTypeDef *hcan, uint16_t VehicleSpeed)
{
	uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[2] = {0};
  if(0 == VehicleSpeed)
  {
    RadarSpeed.RadarDevice_SpeedDirection = STANDSTILL;
    RadarSpeed.RadarDevice_Speed = 0;
  }
  if(VehicleSpeed > 0)
  {
    RadarSpeed.RadarDevice_SpeedDirection = FORWARD;
    RadarSpeed.RadarDevice_Speed = (uint16_t)((((float)VehicleSpeed / 3.6f) - 0) / 0.02f);    //km/h to m/s, offset 0, Res 0.02
  }
//  if(VehicleSpeed < 0)
//  {
//    RadarSpeed.RadarDevice_SpeedDirection = BACKWORD;
//    RadarSpeed.RadarDevice_Speed = (uint16_t)((((float)(-VehicleSpeed) / 3.6f) - 0) / 0.02f); //km/h to m/s, offset 0, Res 0.02
//  }
  CANTxBuf[0] = (RadarSpeed.RadarDevice_SpeedDirection<<6) | ((RadarSpeed.RadarDevice_Speed>>8) & 0x1F);
	CANTxBuf[1] = RadarSpeed.RadarDevice_Speed & 0xFF;
	HAL_CAN_AddTxMessage(hcan, &CAN_TxSpeedHeader, CANTxBuf, &CAN_TxMailBox);
}

/** 
 * @brief  send YawRate to ARS408
 * @note   correction Radar in turning corners
 * @param  *hcan: can2(500kbps)
 * @param  YawRate: (°/s)
 * @retval None
 */
void ARS_SendVehicleYaw(CAN_HandleTypeDef *hcan, float YawRate)
{
	uint32_t CAN_TxMailBox = CAN_TX_MAILBOX0;
	uint8_t CANTxBuf[2] = {0};
  uint16_t YawRate_int = ((YawRate + 327.68f) / 0.01f);     //offset -327.68, Res 0.01
  CANTxBuf[0] = (YawRate_int >> 8) & 0xFF;
	CANTxBuf[1] = YawRate_int & 0xFF;
	HAL_CAN_AddTxMessage(hcan, &CAN_TxYawHeader, CANTxBuf, &CAN_TxMailBox);
}


/** 
 * @brief  Calculate Turning cross 
 * @note   Vehicle Speed should read from Vehicle CAN
 * @param  *pRadargGeneral_Closet: Closet obj
 * @param  YawRate: °/s
 * @param  VehicleSpeed: km/h 
 * @retval 1 Obstacle is in the way
 *				 0 Obstacle is not in the way
 */
uint8_t ARS_CalcTurn(MW_RadarGeneral *pRadargGeneral_Closet, float YawRate, float VehicleSpeed)
{
	float Rotate_R, Min_R, Max_R, Obstacle_X, Obstacle_Y, Obstacle_Dis, RangeLong_Closet, RangeLat_Closet;
  RangeLong_Closet = (pRadargGeneral_Closet->Obj_DistLong * 0.2) - 500;
  RangeLat_Closet = (pRadargGeneral_Closet->Obj_DistLat * 0.2) - 204.6f;
	RangeLat_Closet = (RangeLat_Closet < 0) ? -RangeLat_Closet : RangeLat_Closet;
  VehicleSpeed /= (3.6f * 1.1f);  //  km/h to m/s
	

	Rotate_R = (VehicleSpeed * 180) / (YawRate * 3.14f);	//Rotate_R = V / ω
	Rotate_R = (Rotate_R < 0) ? -Rotate_R : Rotate_R;
	Min_R = Rotate_R - VEHICLE_HALF_WIDTH;
	Max_R = sqrt((Rotate_R + VEHICLE_HALF_WIDTH) * (Rotate_R + VEHICLE_HALF_WIDTH) + \
								 VEHICLE_CENTRE_LEN * VEHICLE_CENTRE_LEN);
	//如果最近位置在最大，最小距离之间，即转弯的车道内，就返回1，否则返回0
	Obstacle_X = RangeLong_Closet + VEHICLE_CENTRE_LEN;
	Obstacle_Y = -RangeLat_Closet + Rotate_R;
	Obstacle_Dis = sqrt(Obstacle_X * Obstacle_X + Obstacle_Y * Obstacle_Y);
	
	if((Obstacle_Dis < Max_R + OBSTACLE_ERR) && (Obstacle_Dis > Min_R - OBSTACLE_ERR))
		return 1;
	else
		return 0;
}

