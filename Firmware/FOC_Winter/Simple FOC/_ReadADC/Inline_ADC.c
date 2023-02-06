#include "Inline_ADC.h"
#include "FOC_Motor.h"
//#include "string.h"

extern float Target;

uint32_t ADC_Value_Temp[ADC_Channel_Num];
uint8_t ADC_Flag,ADC_Start_Flag = 1;
    
//float ADC_Read_Filter(uint8_t channel)
//{
//    float Value,Value_Final;
//    double Value_New,Value_Temp;
//    Value_New = ADC_Value_Temp[channel];
//    for(uint16_t Count = 0;Count < ADC_Buffer;Count++)  
//    {
//      Value_Temp += Value_New;
//    }
//    Value = Value_Temp / ADC_Buffer;
//    Value_Final = Value * 3.3f /4096;

//    return Value_Final;
//}

float ADC_Read_Filter(uint8_t channel)
{

	HAL_ADCEx_InjectedStop_IT(&hadc1);
	float Value,Value_Final;
	double Value_New,Value_Temp;
//	if(ADC_Flag)
//	{
//		ADC_Start_Flag = 0;
		
//		Value_New = ADC_Value_Temp[channel];
//		for(uint16_t Count = 0;Count < ADC_Buffer;Count++)  
//		{
//		  Value_Temp += Value_New;
//		}
//		Value = Value_Temp / ADC_Buffer;
//		Value_Final = Value_New * 3.3f /4096;
//		printf("0 = %d\r\n",ADC_Value_Temp[channel]);
		Value_Final = (float)(ADC_Value_Temp[channel] * 3.3f /4096);
//		ADC_Flag = 0;
//		memset(ADC_Value_Temp,0,sizeof(ADC_Value_Temp));	
//		ADC_Start_Flag = 1;

//	}
//	printf("1 = %f\r\n",Value_Final);
//HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	return Value_Final;
}

void ADC_Start_lnit(void)
{
//    HAL_ADC_Start_DMA(&hadcX,(uint32_t *)ADC_Value_Temp,ADC_Channel_Num);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//	if(ADC_Start_Flag)
//	{
	

//	  if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_INJ_EOC))//就是判断转换完成标志位是否设置,HAL_ADC_STATE_REG_EOC表示转换完成标志位，转换数据可用
//  {
	ADC_Value_Temp[0] = hadc->Instance->JDR1;
    ADC_Value_Temp[1] = hadc->Instance->JDR2;
	ADC_Value_Temp[2] = hadc->Instance->JDR3;
//	Move(Target);
//	LoopFOC();
//  }
//	printf("0 = %d,1 = %d\r\n",ADC_Value_Temp[0],ADC_Value_Temp[1]);
//	ADC_Flag = 1;
//	}
}
