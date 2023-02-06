#ifndef __PID_Motor_H__
#define __PID_Motor_H__

#include "main.h"
#include "FOC_Define.h"

typedef struct 
{
    float P;         
    float I;                     
    float D;                    
    float output_ramp;              //最大输出变化幅度
    float limit;                    //最大输出值
    float error_prev;               //上一次跟踪误差值
    float output_prev;              //上一次PID输出值
    float integral_prev;            //上一次积分量值
    unsigned long timestamp_prev;   //上一次执行时候的时间戳
} PID_Controller;

extern PID_Controller  PID_Current_q,PID_Current_d,PID_Velocity,PID_Angle;
void PID_init(void);
float PID_Operator(PID_Controller* PID,float error);

#endif
