#ifndef __Inline_ADC_H__
#define __Inline_ADC_H__

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "stdio.h"

#define hadcX hadc1
#define ADC_Buffer 1000
#define ADC_Channel_Num 3

enum             //采用哪个通道的ADC需要把他置为0
{
    Channel_0,
    Channel_1,
    Channel_2,
    Channel_3,
    Channel_4,
    Channel_5,
    Channel_6,
    Channel_7,
    Channel_8,
    Channel_9,
    Channel_10,
    Channel_11,
    Channel_12,
    Channel_13,
    Channel_14,
    Channel_15,
};

void ADC_Start_lnit(void);
float ADC_Read_Filter(uint8_t channel);

#endif
