#ifndef __InlineCurrentSense_H__
#define __InlineCurrentSense_H__

#include "main.h"
#include "delay_JR.h"
#include "Inline_ADC.h"
#include "FOC_Define.h"

typedef struct gain
{
    float A;
    float B;
    float C;
}gain;

void Read_InlineCurrentSense(float Sense_resistor, float magnify, int _pinA, int _pinB, int _pinC);
void Current_calibrateOffsets(void);
void InlineCurrentSense_Init(void);

PhaseCurrent_s getPhaseCurrents(void);


#endif


