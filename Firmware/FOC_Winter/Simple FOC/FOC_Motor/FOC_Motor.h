#ifndef __FOC_Motor_H__
#define __FOC_Motor_H__

#include "stdio.h"
#include "PID_Motor.h"
#include "Base_Motor_Control.h"
#include "FOC_Define.h"
#include "arm_math.h"
#include "delay_JR.h"
#include "tim.h"
#include "AS5600.h"
#include "Low_pass_filtering.h"


typedef enum
{
    CW      = 1,  		//顺时针
    CCW     = -1, 		//逆时针
    UNKNOWN = 0   		//还未确定
} Direction;			//方向指定

typedef enum
{
	Type_torque,       		 //力矩控制
	Type_velocity,	   		 //速度控制
	Type_angle,		   		 //角度控制
	Type_velocity_openloop,  //速度开环控制
	Type_angle_openloop      //位置开环控制
} MotionControlType;

extern MotionControlType Controller;
extern long sensor_direction;            //旋转方向

void Motor_Init(void);
void Motor_FOC_Init(float zero_electric_offset,Direction _sensor_direction);
void LoopFOC(void);
void Move(float New_Target);

#endif

