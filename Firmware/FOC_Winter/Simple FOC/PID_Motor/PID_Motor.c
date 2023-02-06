#include "PID_Motor.h"


PID_Controller  PID_Current_q,PID_Current_d,PID_Velocity,PID_Angle;

void PID_init(void)    //Motor_init()函数已经对limit初始化，此处无需处理
{
	PID_Velocity.P=0.5;  //0.5
	PID_Velocity.I=0;    //0.5
	PID_Velocity.D=0;
	PID_Velocity.output_ramp=0;   
	//PID_Velocity.limit=0;               
	PID_Velocity.error_prev=0;
	PID_Velocity.output_prev=0;
	PID_Velocity.integral_prev=0;
	PID_Velocity.timestamp_prev=0;
	
	PID_Angle.P=20;
	PID_Angle.I=0;
	PID_Angle.D=0;
	PID_Angle.output_ramp=0;
	//PID_Angle.limit=0;
	PID_Angle.error_prev=0;
	PID_Angle.output_prev=0;
	PID_Angle.integral_prev=0;
	PID_Angle.timestamp_prev=0;
	
	PID_Current_q.P=0.5;                //航模电机，速度闭环，不能大于1，否则容易失控
	PID_Current_q.I=0;                  //电流环I参数不太好调试，只用P参数也可以
	PID_Current_q.D=0;
	PID_Current_q.output_ramp=0;
	//PID_Current_q.limit=0;
	PID_Current_q.error_prev=0;
	PID_Current_q.output_prev=0;
	PID_Current_q.integral_prev=0;
	PID_Current_q.timestamp_prev=0;
	
	PID_Current_d.P=0.5;  //0.5
	PID_Current_d.I=0;
	PID_Current_d.D=0;
	PID_Current_d.output_ramp=0;
	//PID_Current_d.limit=0;
	PID_Current_d.error_prev=0;
	PID_Current_d.output_prev=0;
	PID_Current_d.integral_prev=0;
	PID_Current_d.timestamp_prev=0;
}

float PID_Operator(PID_Controller* PID,float error)
{
    unsigned long Now_us;
	float Sample_time;
	float Proportional,Integral,Derivative,Output;
	float Output_rate;

	Now_us = SysTick->VAL;
	if(Now_us < PID->timestamp_prev)
	{
		Sample_time = (float)(PID->timestamp_prev - Now_us) / 168 * 1e-6f;
	}
	else
	{
		Sample_time = (float)(0xFFFFFF - Now_us + PID->timestamp_prev) / 168 * 1e-6f;            //简单来说就是看看过了多久
	}
	PID->timestamp_prev = Now_us;
	if(Sample_time == 0 || Sample_time > 0.5f) Sample_time = 1e-3f;                      //解决奇怪溢出

	Proportional = PID->P * error;                                   					//比例
	Integral = PID->I + PID->I * (Sample_time * 0.5f * (error + PID->error_prev));		//积分
	Integral = _constrain(Integral, -PID->limit, PID->limit);							//限幅
	Derivative = PID->D * (error - PID->error_prev) / Sample_time;                      //微分

	Output = Proportional + Integral + Derivative;
	Output = _constrain(Output, -PID->limit, PID->limit);

	if(PID->output_ramp > 0)
	{
		Output_rate = (Output - PID->output_prev) / Sample_time;
		if(Output_rate > PID->output_ramp)
		{
			Output_rate = PID->output_prev + PID->output_ramp * Sample_time;
		}
		else if(Output_rate < -PID->output_ramp)
		{
			Output_rate = PID->output_prev - PID->output_ramp * Sample_time;
		}
	}

	PID->integral_prev = Integral;
	PID->output_prev = Output;
	PID->error_prev = error;

	return Output;
}

