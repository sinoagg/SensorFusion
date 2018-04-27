#include "ADAS.h"

uint8_t CalADASData(ADAS_HandleTypeDef *pADAS_dev, uint8_t *pRxBuf)
{
	if(*pRxBuf==0x4B&&*(pRxBuf+1)==0x59&&*(pRxBuf+2)==0x4B&&*(pRxBuf+3)==0x4A&&*(pRxBuf+4)==0x77)
	{
		//for(j=0;j<23;j++)	UART3_TX_BUF[j]=UART2_RX_BUF[j+i];
		//HAL_UART_Transmit(&huart3, UART3_TX_BUF, 23, 1000);		//传递报警信号给网关
		pADAS_dev->DEV_work_status=(*(pRxBuf+6)&0x80)>>7;
		pADAS_dev->LDW_work_status=(*(pRxBuf+6)&0x40)>>6;
		pADAS_dev->FCW_work_status=(*(pRxBuf+6)&0x20)>>5;
		pADAS_dev->PCW_work_status=(*(pRxBuf+6)&0x10)>>4;		//PCW工作状态
		pADAS_dev->DFW_work_status=(*(pRxBuf+6)&0x08)>>3;		//DFW工作状态
		pADAS_dev->self_check_main=(*(pRxBuf+6)&0x01)>>0;
		
		pADAS_dev->LDW_warning=(*(pRxBuf+7)&0xC0)>>6;		//LDW车道偏离 0x00正常，0x01左偏离 0x02右偏离 0x03保留			
		pADAS_dev->crash_type=(*(pRxBuf+7)&0x20)>>5;		//0x00 FCW ，0x01 PCW
		pADAS_dev->crash_level=(*(pRxBuf+7)&0x18)>>3;		//0x00正常 0x01一级预警 0x02二级预警 0x03三级预警
		pADAS_dev->driver_status=(*(pRxBuf+7)&0x07);			//0x000正常 0x001疲劳 0x002姿态异常 0x003打电话 0x004抽烟															
		
		pADAS_dev->distance_left_line=*(pRxBuf+8);			//与左车道线距离
		pADAS_dev->distance_right_line=*(pRxBuf+9);			//与右车道线距离
		
		pADAS_dev->line_circle_diameter=*(pRxBuf+10);		//车道曲率直径		
		
		pADAS_dev->line_width=(*(pRxBuf+11)&0xF8)>>3;		//车道线宽度
		pADAS_dev->line_type=(*(pRxBuf+11)&0x07);				//车道线类型														
										
		pADAS_dev->direction_angle=*(pRxBuf+12);				//行驶方向与车道线夹角
		
		pADAS_dev->distance_to_target=*(pRxBuf+13); 		//与前方目标距离
		pADAS_dev->speed_to_target=*(pRxBuf+14);				//与前方目标相对车速
		pADAS_dev->time_to_target=0.05**(pRxBuf+15);		//与前方目标预碰撞时间
		
//解析协议，点灯
//LDW状态获取//////////////////////////////////////////////////////////////	
					switch(pADAS_dev->LDW_warning)
					{
						case 0x00:
							//显示无车道偏离图片
							break;
						case 0x01:
							//显示左车道偏离图片
							break;
						case 0x02:
							//显示右车道偏离图片
							break;
						default:
							//显示无车道偏离图片
							break;
					}
//碰撞强度获取//////////////////////////////////////////////////////////////						
					switch(pADAS_dev->crash_level)
					{
						case 0x00:
							//显示无行人碰撞报警图片
							//显示无车辆碰撞报警图片
							break;
						case 0x01:
							//显示无行人碰撞报警图片
							//显示无车辆碰撞报警图片
							break;
						case 0x02:
							if(pADAS_dev->crash_type==0x00) 
							{
								//显示中等车辆碰撞报警图片
								//清除中等行人碰撞报警图片
							}
							else
							{
								//清除中等车辆碰撞报警图片
								//显示中等行人碰撞报警图片
							}
							
							break;
						case 0x03:
							if(pADAS_dev->crash_type==0x00) 
							{
								//显示严重车辆碰撞报警图片
								//清除严重行人碰撞报警图片
							}
							else
							{
								//清除严重车辆碰撞报警图片
								//显示严重行人碰撞报警图片
							}
							break;
						default:
							//显示无行人碰撞报警图片
							//显示无车辆碰撞报警图片
							break;
					}
//疲劳状态获取//////////////////////////////////////////////////////////////					
					switch(pADAS_dev->driver_status)
					{
						case 0x000:
							//Display_Face(0);
							break;
						case 0x001:
							//Display_Face(1);
							
							break;
						case 0x002:
							//Display_Face(1);
							
							break;
						case 0x003:
							//Display_Face(1);
							
							break;
						case 0x004:
							//Display_Face(1);
							
							break;
						default:
							//Display_Face(0);
							break;
					}
					return 0;
	}
	return 1;
}
