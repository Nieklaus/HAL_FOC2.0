#include "delay_JR.h"

/*
此.c文件专门用于解释HAL库中的滴答定时器问题
HAL库中的HAL_Inlt实际上已经配置了滴答定时器
所以HAL_Delay()的基准源也由此得来
并且我们之前只要是使用了HAL库开发都已经开启了滴答定时器
可由HAL_InitTick() HAL_SYSTICK_Config() SysTick_Config() 等函数得知
其中 SystemCoreClock 会发现非当前在CUBEMX设置中的主频或FCLK
但是其实 SystemCoreClock 会在之后的 SystemClock_Config()函数中重新进行配置成主频
且在HAL_InitTick()中会将 SystemCoreClock/1000 后输出至 Systick->LOAD
而 SysTick_Config() 会确立配置滴答定时器的寄存器 (注意：此时core_cmx.h的文件中配置它为灰色，但实际上该函数已编译)
*/

/*
滴答定时器也可以通过改变core_cmx.h里的宏定义后重新编写SysTick_Config()后在HAL_Init()中配置完成自己想要的程度
但是由于此方法较为麻烦且与自己编写delay_init(uint8_t SYSCLK)结果一样
所以可以根据自己的需求选择方法
*/

/*
注意：
若采用重新编写SysTick_Config()配置滴答定时器，可以继续使用HAL_DELAY()函数
但是在使用自己编写的delay_init(uint8_t SYSCLK)配置滴答定时器，若不对SysTick->LOAD进行改变，也可以继续使用HAL_DELAY()函数
反之，则不能继续使用HAL_DELAY()函数，因为误差太大。
但是经过实测HAL_DELAY()函数本身也不是很准，不如使用delay_ms(uint16_t nms)函数更准一点
*/

static uint32_t fac_us=0;	

void delay_init(uint8_t SYSCLK)
{
	//SYSTICK_CLKSOURCE_HCLK或SYSTICK_CLKSOURCE_HCLK_DIV8选择是否8分频
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);		
	//SysTick频率为HCLK    
	//该函数可以理解为HAL库再次赋值确认systick的时钟源，实际上若不调用该函数则时钟源默认为SYSTICK_CLKSOURCE_HCLK
	SysTick->LOAD = 0xFFFFFF - 1;
	SysTick->VAL  = 0;
	
	fac_us = SYSCLK;						    					//不论是否使用OS,fac_us都需要使用
}	

void ll(void)
{

}

void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};
}
 
//延时nms
//nms:要延时的ms数
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}

