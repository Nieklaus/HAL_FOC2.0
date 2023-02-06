#include "delay_JR.h"

/*
��.c�ļ�ר�����ڽ���HAL���еĵδ�ʱ������
HAL���е�HAL_Inltʵ�����Ѿ������˵δ�ʱ��
����HAL_Delay()�Ļ�׼ԴҲ�ɴ˵���
��������֮ǰֻҪ��ʹ����HAL�⿪�����Ѿ������˵δ�ʱ��
����HAL_InitTick() HAL_SYSTICK_Config() SysTick_Config() �Ⱥ�����֪
���� SystemCoreClock �ᷢ�ַǵ�ǰ��CUBEMX�����е���Ƶ��FCLK
������ʵ SystemCoreClock ����֮��� SystemClock_Config()���������½������ó���Ƶ
����HAL_InitTick()�лὫ SystemCoreClock/1000 ������� Systick->LOAD
�� SysTick_Config() ��ȷ�����õδ�ʱ���ļĴ��� (ע�⣺��ʱcore_cmx.h���ļ���������Ϊ��ɫ����ʵ���ϸú����ѱ���)
*/

/*
�δ�ʱ��Ҳ����ͨ���ı�core_cmx.h��ĺ궨������±�дSysTick_Config()����HAL_Init()����������Լ���Ҫ�ĳ̶�
�������ڴ˷�����Ϊ�鷳�����Լ���дdelay_init(uint8_t SYSCLK)���һ��
���Կ��Ը����Լ�������ѡ�񷽷�
*/

/*
ע�⣺
���������±�дSysTick_Config()���õδ�ʱ�������Լ���ʹ��HAL_DELAY()����
������ʹ���Լ���д��delay_init(uint8_t SYSCLK)���õδ�ʱ����������SysTick->LOAD���иı䣬Ҳ���Լ���ʹ��HAL_DELAY()����
��֮�����ܼ���ʹ��HAL_DELAY()��������Ϊ���̫��
���Ǿ���ʵ��HAL_DELAY()��������Ҳ���Ǻ�׼������ʹ��delay_ms(uint16_t nms)������׼һ��
*/

static uint32_t fac_us=0;	

void delay_init(uint8_t SYSCLK)
{
	//SYSTICK_CLKSOURCE_HCLK��SYSTICK_CLKSOURCE_HCLK_DIV8ѡ���Ƿ�8��Ƶ
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);		
	//SysTickƵ��ΪHCLK    
	//�ú����������ΪHAL���ٴθ�ֵȷ��systick��ʱ��Դ��ʵ�����������øú�����ʱ��ԴĬ��ΪSYSTICK_CLKSOURCE_HCLK
	SysTick->LOAD = 0xFFFFFF - 1;
	SysTick->VAL  = 0;
	
	fac_us = SYSCLK;						    					//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
}	

void ll(void)
{

}

void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};
}
 
//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}

