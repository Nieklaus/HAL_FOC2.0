#include "Low_pass_filtering.h"

LowPassFilter LPF_current_q,LPF_current_d,LPF_velocity;

void LPF_init(void)
{
	LPF_current_q.Tf=0.05;    
	LPF_current_q.y_prev=0;
	LPF_current_q.timestamp_prev=0;  //SysTick->VAL;
	
	LPF_current_d.Tf=0.05;
	LPF_current_d.y_prev=0;
	LPF_current_d.timestamp_prev=0;
	
	LPF_velocity.Tf=0.0001;   //Tf设置小一点，配合爬升斜率设置PID_velocity.output_ramp，速度切换更平稳；如果没有爬升模式的斜率限制，Tf太小电机容易抖动。
	LPF_velocity.y_prev=0;
	LPF_velocity.timestamp_prev=0;
}

float LPF_Operator(LowPassFilter* LPF,float X)                    //这里的x为一阶低通滤波公式中的x
{
	unsigned long Now_us;
	float Dt,alpha,Y;

	Now_us = SysTick->VAL;//递减
	if(Now_us < LPF->timestamp_prev) //timestamp_prev 为上次的计算时的滴答定时器的值
	{
		Dt = (float)(LPF->timestamp_prev - Now_us) / 168 * 1e-6f; //计算上次滤波时和现在此次计算的时间差值来决定时间常数滤波程度
	}
	else
	{
		Dt = (float)(0xFFFFFF - Now_us + LPF->timestamp_prev) / 168 * 1e-6f;//由于是递减的滴答定时器所以时间越久0xFFFFFF - now_us的数值越大
	}
	LPF->timestamp_prev = Now_us; //保存滴答定时器的值
	if(Dt > 0.3f)   //时间过长，大概是程序刚启动初始化，直接返回
	{
		LPF->y_prev = X;         
		return X;
	}
	alpha = LPF->Tf/(LPF->Tf + Dt);      //滤波系数 时间过久则要求不要突变平稳一些

	Y = alpha * LPF->y_prev + (1.0f - alpha) * X;       //这个公式看起来自变量和上次变量反过来了但是在simple foc官网中的计算推导的LPF公式确实是反过来的
	LPF->y_prev = Y;									//如果实在介意可以通过改变PID的参数让公式达到应该的效果(即高时间常数的状态)

	return Y;
}
