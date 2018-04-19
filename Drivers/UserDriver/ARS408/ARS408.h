#ifndef __ARS408_H
#define __ARS408_H

#include "stm32f4xx_hal.h"

/* 地址定义--------------------------------------*/
//雷达状态和设置
#define RADAR_CFG_ADDR 					0x200
#define RADAR_STA_ADDR					0x201
//过滤器状态和设置
#define FILTER_CFG_ADDR					0x202
#define FILTER_STA_HEADER_ADDR			0x203
#define FILTER_STA_CFG_ADDR				0x204
//碰撞决策状态和设置
#define COLL_DET_CFG_ADDR				0x400
#define COLL_DET_REGN_CFG_ADDR  		0x401 
#define COLL_DET_STA_ADDR				0x408
#define COLL_DET_REGN_STA_ADDR  		0x402
//速度与转向输入信号
#define SPEED_INFO_ADDR					0x300
#define YAW_INFO_ADDR					0x301
//反射集群检测
#define CLUSTER_STA_ADDR				0x600
#define CLUSTER_GENERAL_ADDR			0x701
#define CLUSTER_QUALITY_ADDR			0x702
//目标检测
#define OBJ_STA_ADDR					0x60A
#define OBJ_GENERAL_ADDR				0x60B
#define OBJ_QUALITY_ADDR				0x60C
#define OBJ_EXTENDED_ADDR				0x60D
#define OBJ_WARNING_ADDR				0x60E
//软件版本
#define VERSION_ID_ADDR					0x700 
//碰撞继电器控制
#define COLL_DET_RELAY_ADDR				0x008

#define INVALID 	0x00
#define VALID		0x01

#define RADARCFG_MAXDISTANCE_VALID		(VALID<<0)
#define RADARCFG_RADARPOWER_VALID		(VALID<<2)
#define RADARCFG_OUTPUTTYPE_VALID		(VALID<<3)
#define RADARCFG_SORTINDEX_VALID		(VALID<<6)
#define RADARCFG_STOREINNVM_VALID		(VALID<<7)
#define RADARCFG_RCS_THRES_VALID		(VALID<<0)
#define RADARCFG_RADARPOWER_STD			(0<<5)
#define RADARCFG_RADARPOWER_3dB			(1<<5)
#define RADARCFG_RADARPOWER_6dB			(2<<5)
#define RADARCFG_RADARPOWER_9dB			(3<<5)
#define RADARCFG_OUTPUTTYPE_NONE		(0<<3)
#define RADARCFG_OUTPUTTYPE_OBJ			(1<<5)
#define RADARCFG_OUTPUTTYPE_CLUSTER		(2<<5)
#define RADARCFG_STOREINNVM				(1<<7)
#define RADARCFG_SORTINDEX_NONE			(0<<4)
#define RADARCFG_SORTINDEX_RANGE		(1<<4)
#define RADARCFG_SORTINDEX_RCS			(2<<4)
#define RADARCFG_RCSTHRES_STD			(0<<1)
#define RADARCFG_RCSTHRES_HIGHSENSE		(1<<1)

typedef struct
{
	uint8_t MaxDistance_valid;			//是否允许远端扫描距离变化
	uint8_t SenSorID_valid;				//是否允许传感器ID变化
	uint8_t RadarPower_valid;			//是否允许发射功率变化
	uint8_t OutputType_valid;			//是否允许输出内容变化
	uint8_t SendQuality_valid;			//是否允许输出目标或集群质量信息
	uint8_t SendExtInfo_valid;			//是否允许扩展信息输出变化
	uint8_t SortIndex_valid;			//允许目标排序顺序变化
	uint8_t StoreInNVM_valid;			//允许存储目前设置至NVM变化
	uint16_t MaxDistance;				//远端扫描最远距离，近端扫描距离固定为远端一半，标准距离196~260 分辨率1.79m for 200m far scan
	uint8_t SensorID;					//传感器ID从0到7
	uint8_t OutputType;					//输出内容 0x01 目标；0x02 反射集群
	uint8_t RadarPower;					//雷达发射功率 0x00 标准；0x01 -3dB; 0x02 -6dB; 0x03 -9dB;
	uint8_t CtrlRelay_valid;			//允许继电器控制
	uint8_t CtrlRelay;					//继电器控制
	uint8_t SendQuality;				//集群或目标质量信息发送使能
	uint8_t SendExtInfo;				//扩展信息输出使能，仅在目标信息输出时有效
	uint8_t SortIndex;					//选择目标排序顺序，仅在目标信息输出时有效，集群输出时总是按距离排序
	uint8_t StoreInNVM;					//存储目前设置至NVM
	uint8_t RCS_Threshold_valid;		//允许设置RCS灵敏度变化
	uint8_t RCS_Threshold;				//设置雷达散射截面积(radar cross section)检测灵敏度 0x00标准； 0x01 高灵敏度
} MW_RadarConfig;	

#define FILTERCFG_VALID (VALID<<1)
#define FILTERCFG_FILTERACTIVE_VALID (VALID<<2)
#define FILTERCFG_INDEX_NOFOBJ 		(0<<3)
#define FILTERCFG_INDEX_DISTANCE	(1<<3)
#define FILTERCFG_INDEX_AZIMUTH 	(2<<3)
#define FILTERCFG_INDEX_VRELONCOME 	(3<<3)
#define FILTERCFG_INDEX_VRELDEPART	(4<<3)
#define FILTERCFG_INDEX_RCS 		(5<<3)
#define FILTERCFG_INDEX_LIFETIME	(6<<3)
#define FILTERCFG_INDEX_SIZE 		(7<<3)
#define FILTERCFG_INDEX_PROBEXIST 	(8<<3)
#define FILTERCFG_INDEX_Y 			(9<<3)
#define FILTERCFG_INDEX_X 			(10<<3)
#define FILTERCFG_INDEX_VYRIGHTLEFT (11<<3)
#define FILTERCFG_INDEX_VXONCOME 	(12<<3)
#define FILTERCFG_INDEX_VYLEFTRIGHT (13<<3)
#define FILTERCFG_INDEX_VXDEPART 	(14<<3)
#define FILTERCFG_TYPE_CLUSTER 		(0<<7)
#define FILTERCFG_TYPE_OBJ 			(1<<7)


typedef struct
{
	uint8_t FilterCfg_Valid;				//是否允许修改FilterCfg
	uint8_t FilterCfg_Active;				//使能或者禁止相应的标准（FilterCfg）和种类
	uint8_t FilterCfg_Index;				//调整15项设置的目录
	uint8_t FilterCfg_Type;					//调整设置目标或者反射集群
	uint16_t FilterCfg_Min_XXX;				//设置目录项内容最小值
	uint8_t FilterCfg_Max_XXX;				//设置目录项内容最大值
}MW_RadarFilterConfig;


typedef struct
{
	uint16_t FilterCfg_Min_NofObj;			//最大目标或集群检测数量
	uint16_t FilterCfg_Max_NofObj;			//此值已忽略
	uint16_t FilterCfg_Min_Distance;		//最小过滤半径
	uint16_t FilterCfg_Max_Distance;		//最大过滤半径
	uint16_t FilterCfg_Min_Azimuth;			//最小过滤航向角度
	uint16_t FilterCfg_Max_Azimuth;			//最大过滤航向角度
	uint16_t FilterCfg_Min_VrelOncome;		//最小相对靠近速度
	uint16_t FilterCfg_Max_VrelOncome;		//最大相对靠近速度
	uint16_t FilterCfg_Min_VrelDepart;		//最小相对远离速度
	uint16_t FilterCfg_Max_VrelDepart;		//最大相对远离速度
	uint16_t FilterCfg_Min_RCS;				//最小反射截面积
	uint16_t FilterCfg_Min_Lifetime;		//最小被探测时间
	uint16_t FilterCfg_Min_Size;			//最小探测尺寸
	uint16_t FilterCfg_Min_ProbExists;		//最小物体存在概率
	uint16_t FilterCfg_Min_Y;				//Y轴方向最小距离
	uint16_t FilterCfg_Max_Y;				//Y轴方向最大距离
	uint16_t FilterCfg_Min_X;				//X轴方向最小距离
	uint16_t FilterCfg_Max_X;				//X轴方向最大距离
	uint16_t FilterCfg_Min_VYRightLeft;		//Y轴横向移动最小速度
	uint16_t FilterCfg_Max_VYRightLeft;		//Y轴横向移动最大速度
	uint16_t FilterCfg_Min_VXOncome;		//X轴相对驶来最小速度
	uint16_t FilterCfg_Max_VXDepart;		//X轴相互离开最大速度
}MW_RadarFilterIndexContent;
	
typedef struct
{
	uint8_t RadarState_NVMReadStatus;		//启动时从不变内存读配置参数状态
	uint8_t RadarState_NVMwriteStatus;		//成功存储的配置参数数目
	uint16_t RadarState_MaxDistanceCfg;		//当前最远距离扫描配置
	uint8_t RadarState_Persistent_Error;	//持续误差
	uint8_t RadarState_Interference;		//多个雷达的干扰
	uint8_t RadarState_Temperature_Error;	//温度误差
	uint8_t RadarState_Temporary_Error;		//临时误差，重新设置时消失的误差
	uint8_t RadarState_Voltage_Error;		//电压误差超过范围时为1
	uint8_t RadarState_SensorID;			//传感器ID 0-7
	uint8_t RadarState_SortIndex;			//排列顺序，对象列表的
	uint8_t RadarState_RadarPowerCfg;		//发送信号的增益
	uint8_t RadarState_CtrlRelayCfg;		//控制继电器
	uint8_t RadarState_OutputTypeCfg;		//输出类型，集群或对象
	uint8_t RadarState_SendQualityCfg;		//发送质量
	uint8_t RadarState_SendExtInfoCfg;		//发送额外信息
	uint8_t RadarState_MotionRxState;		//速度、偏航角度输入信号的状态
	uint8_t RadarState_RCS_Threshold;		//为真时传感器开启高敏感模式
}MW_RadarState;

typedef struct
{
	uint8_t RadarDevice_SpeedDirection;
	uint16_t RadarDevice_Speed;
}MW_RadarSpeed;

typedef struct
{
	uint8_t Obj_NofObjects;
	uint16_t Obj_MeasCounter;
	uint8_t Obj_InterfaceVer;
}MW_RadarObjStatus;

typedef struct
{
	uint8_t Obj_ID;
	uint16_t Obj_DistLong;
	uint16_t Obj_DistLat;
	uint16_t Obj_VrelLong;
	uint8_t Obj_DynProp;
	uint16_t Obj_VrelLat;
	uint8_t Obj_RCS;
}MW_Radar_General;

extern MW_RadarConfig RadarConfig;
extern MW_RadarFilterConfig RadarFilterConfig;
extern MW_RadarObjStatus RadarObjStatus;
extern MW_Radar_General RadarGeneral[64];

uint8_t ARS_ConfigRadar(CAN_HandleTypeDef *hcan);
uint8_t ARS_ConfigFilter(CAN_HandleTypeDef *hcan);

#endif

