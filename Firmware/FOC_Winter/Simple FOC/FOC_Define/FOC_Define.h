#ifndef __FOC_Define_H__
#define __FOC_Define_H__

#include "math.h"
#include "FOC_Motor.h"
#include "AS5600.h"
#include "Low_pass_filtering.h"

//数学函数定义
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _sqrt(a) (_sqrtApprox(a))
#define Set_if(a) ( (a) != (NOT_SET) )


//数学参数
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f

#define NOT_SET -12345

typedef struct    //通过三相电流经过Clark变换和Park变换得到的IQ和ID
{
	float d;
	float q;
} DQCurrent_s;

typedef struct  //通过三相电流经过Clark变换和Park变换得到的VQ和VD
{
	float d;
	float q;
} DQVoltage_s;

typedef struct    //通过电流环采样出来的三相电流
{
	float a;
	float b;
	float c;
} PhaseCurrent_s;


float _sin(float a);
float _cos(float a);
float _electricalAngle(float shaft_angle, int pole_pairs);
float _normalizeAngle(float angle);
float _sqrtApprox(float number);

#endif
