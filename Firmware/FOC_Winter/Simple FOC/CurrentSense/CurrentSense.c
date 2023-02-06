#include "CurrentSense.h"

DQCurrent_s getFOCCurrents(float angle_el)
{
	PhaseCurrent_s current;
	float i_alpha, i_beta;
	float ct,st;
	DQCurrent_s ret;
	
	// read current phase currents
	current = getPhaseCurrents();
//	printf("current.a = %f,current.b = %f,current.c = %f\r\n",current.a,current.b,current.c);
	
	// calculate clarke transform
	if(!current.c)                                 //Clark变化
	{
		// if only two measured currents
		i_alpha = current.a;  
		i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
	}
	else
	{
		// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
		float mid = (1.f/3) * (current.a + current.b + current.c);
		float a = current.a - mid;
		float b = current.b - mid;
		i_alpha = a;
		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	}
	
	// calculate park transform                    Park变化获取IQ和ID
	ct = arm_cos_f32(angle_el);
	st = arm_sin_f32(angle_el);
	ret.d = i_alpha * ct + i_beta * st;      
	ret.q = i_beta * ct - i_alpha * st;
	return ret;
}
