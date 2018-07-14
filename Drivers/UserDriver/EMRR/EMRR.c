#include "EMRR.h"


extern CAN_HandleTypeDef hcan;
MW_RadarGeneral pRadarGeneral[64];  //接受到的64组数据
MW_RadarGeneral  spRadarGeneral;    //距离最小的数
uint8_t complete; //can接受标志
uint8_t RadarCANRxBuf[8];
void GetRadarObjGeneral(uint8_t* pCANRxBuf,MW_RadarGeneral *pRadarGeneral)
{
	uint16_t tempData;
	//id
	pRadarGeneral->trackId=hcan.pRxMsg->StdId- 0x4ff;

	//range
	pRadarGeneral->trackRange = (float)(pCANRxBuf[0] | ((pCANRxBuf[1] & 0x7F) << 8))*0.01f;

	//speed
	tempData =  ((pCANRxBuf[3] & 0x3F) << 8) | pCANRxBuf[2];
	pRadarGeneral->trackSpeed =  (float)(tempData > 8191 ? (tempData - 16384)*0.01f : tempData*0.01f);

	//angle
	tempData = ((pCANRxBuf[5] & 0xFC) >> 2) | ((pCANRxBuf[6] & 0x1F) << 6);
	pRadarGeneral->trackAngle = (float)((tempData) > 1023 ? (tempData- 2048) *0.1f : tempData *0.1f);

	//power
	if (pRadarGeneral->trackRange > 0)
	{
		tempData = ((pCANRxBuf[6] & 0xE0) >> 5) | ((pCANRxBuf[7] & 0x7F) << 3);
		pRadarGeneral->trackPower = (float)(tempData > 511 ? (tempData - 1024)*0.1f - 40 : (tempData)*0.1f - 40);
	}
	else
	{
		pRadarGeneral->trackPower = 0;
	}

}
void Rader_small()
{
		if(complete)//接受标志
	  {
		GetRadarObjGeneral(RadarCANRxBuf, &pRadarGeneral[hcan.pRxMsg->StdId-0x500]);
		if((hcan.pRxMsg->StdId-0x500+1)==64)
			{
				uint32_t i=0,min_lable;
				float min=1000,x;
				for(i=0;i<64;i++)
				{
					if(pRadarGeneral[i].trackRange!=0)//第一层判断
					{
						x=3.14*fabs(pRadarGeneral[i].trackAngle)/180;
						pRadarGeneral[i].trackCrossRange=(float)(pRadarGeneral[i].trackRange*sin(x));
						if(pRadarGeneral[i].trackCrossRange<1.8)//第二层判断
						{
							
						//min=rxCan[i].trackRange;
						//minu=min;
							 if(pRadarGeneral[i].trackPower>-50)
							 {
                 if(min>pRadarGeneral[i].trackRange)
						     {
							    min=pRadarGeneral[i].trackRange;
							    min_lable=i;
					       }
							 }
					  }
					}
				}
				spRadarGeneral.trackId=pRadarGeneral[min_lable].trackId;
				spRadarGeneral.trackCrossRange=pRadarGeneral[min_lable].trackCrossRange;
				spRadarGeneral.trackRange=pRadarGeneral[min_lable].trackRange;
				spRadarGeneral.trackSpeed=pRadarGeneral[min_lable].trackSpeed;
				spRadarGeneral.trackAngle=pRadarGeneral[min_lable].trackAngle;
				spRadarGeneral.trackPower=pRadarGeneral[min_lable].trackPower;
	     }
			complete=0;
		}
}
