#include "stm32f4xx_hal.h"	
#include "delay.h"

uint8_t fac_us;

void delay_init(uint8_t SYSCLK)
{
    #if SYSTEM_SUPPORT_OS //?????? OS.
        u32 reload;
    #endif
    
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    //SysTick ??? HCLK
    fac_us=SYSCLK; //?????? OS,fac_us ?????
    
    #if SYSTEM_SUPPORT_OS //?????? OS.
        reload=SYSCLK; //???????? ??? K
        reload*=1000000/delay_ostickspersec; //?? delay_ostickspersec ??????
        //reload ? 24 ????,???:16777216,? 180M ?,?? 0.745s ??
        fac_ms=1000/delay_ostickspersec; //?? OS ?????????
        SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//?? SYSTICK ??
        SysTick->LOAD=reload; //? 1/OS_TICKS_PER_SEC ?????
        SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //?? SYSTICK
        #else
    #endif
}

void Delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD; 
    ticks=nus*fac_us; 
    told=SysTick->VAL; 
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    }
}
void Delay_ms(uint32_t ms)
{
	uint16_t i,num;
	num=ms;
	for(i=0;i<num;i++)
	{
		Delay_us(1000);
	}
}

