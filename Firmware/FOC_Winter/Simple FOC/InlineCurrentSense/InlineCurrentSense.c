#include "InlineCurrentSense.h"

gain vta;
float offset_ia,offset_ib,offset_ic;
int PinA,PinB,PinC;
float A1;

/*
Vout = INA_VCC/2 + Sense_resistor * I * magnify
电流计算公式
通过ADC读取电压后通过公式反推电流
*/
void Read_InlineCurrentSense(float Sense_resistor, float magnify, int _pinA, int _pinB, int _pinC)    //电流检测电阻阻值  电流检测运放放大倍数 A,B,C相
{
	float volts_to_amps_ratio;      
	
	PinA = _pinA;
	PinB = _pinB;
	PinC = _pinC;

	volts_to_amps_ratio = 1.0f / Sense_resistor / magnify;

	vta.A = volts_to_amps_ratio;
	vta.B = volts_to_amps_ratio;    //simple中由于布线方便反过来接了  若正接需要去掉负号
	vta.C = volts_to_amps_ratio;
	
	printf("gain_a:%.2f,gain_b:%.2f,gain_c:%.2f.\r\n",vta.A,vta.B,vta.C);
}

void Current_calibrateOffsets(void)   ////检测偏置电压，也就是电流0A时的运放输出电压值，理论值=1.65V
{

	offset_ia=0;
	offset_ib=0;
	offset_ic=0;

	for(int i = 0; i<1000; i++)
	{
		offset_ia += ADC_Read_Filter(PinA);
		offset_ib += ADC_Read_Filter(PinB);
		if(Set_if(PinC)) offset_ic += ADC_Read_Filter(PinC);
//		delay_ms(1);
	}
	
	offset_ia = offset_ia/1000;
	offset_ib = offset_ib/1000;
	if(Set_if(PinC)) offset_ic = offset_ic / 1000;
	
	printf("offset_ia:%.4f,offset_ib:%.4f,offset_ic:%.4f.\r\n",offset_ia,offset_ib,offset_ic);
}

void InlineCurrentSense_Init(void)
{
	ADC_Start_lnit();
	Current_calibrateOffsets();   //检测0A时的电压当做基准源
}

PhaseCurrent_s getPhaseCurrents(void)     //获取电流值并且存入结构体中
{
//	float a,b;
	PhaseCurrent_s current;
	
	current.a = (ADC_Read_Filter(PinA) - offset_ia) * vta.A;// amps
	current.b = (ADC_Read_Filter(PinB) - offset_ib) * vta.B;// amps
	current.c = (!(Set_if(PinC))) ? 0 : (ADC_Read_Filter(PinB) - offset_ic) * vta.C; // 如果C相选择未设置那么current.c = 0
	
//	a = ADC_Read_Filter(PinA);
//	b = ADC_Read_Filter(PinB);
//	printf("a = %f,b = %f\r\n",a,b);
//	
	
	
	return current;
}
