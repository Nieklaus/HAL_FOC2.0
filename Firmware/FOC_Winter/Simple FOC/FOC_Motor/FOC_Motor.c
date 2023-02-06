#include "FOC_Motor.h"
#include "InlineCurrentSense.h"
#include "CurrentSense.h"

MotionControlType Controller;

DQCurrent_s Current;              //经Clark和park转换后的电流
DQVoltage_s Voltage;              //经Clark和park转换后的电压

float zero_electric_angle;        //零点角度
long sensor_direction;            //旋转方向
float shaft_velocity;              //轴速度
float shaft_angle;                //轴角度  也是当前电机角度
float electrical_angle;

float shaft_velocity_sp;          //Temp
float shaft_angle_sp;
float current_sp;

unsigned long open_loop_timestamp;

// 轴角度计算
float shaftAngle(void)
{
  return sensor_direction*AS5600_Get_Angle();
}
// 轴速度计算
float shaftVelocity(void)
{
  return sensor_direction * LPF_Operator(&LPF_velocity,Get_Velocity());
}

float electricalAngle(void)
{
  return _normalizeAngle(shaft_angle * Motor_init.pole_pairs - zero_electric_angle);
}

void setPhaseVoltage(float Uq,float Ud,float angle)
{
    float Uout;
	uint32_t sector;
	float T0,T1,T2;
	float Ta,Tb,Tc;
	uint32_t TAA,TBB,TCC;
	
	_constrain(Uq,-Motor_init.voltage_limit,Motor_init.voltage_limit);
	_constrain(Ud,-Motor_init.voltage_limit,Motor_init.voltage_limit);

    if(Ud) 
	{
		Uout = _sqrt(Ud*Ud + Uq*Uq) / Motor_init.voltage_power_supply;
		angle = _normalizeAngle(angle + atan2(Uq, Ud));
	}
	
	else
	{
		Uout = Uq / Motor_init.voltage_power_supply;
		angle = _normalizeAngle(angle + _PI_2);        //_PI_2应该是让初始电流矢量垂直于转子磁极
	}
//	if(Uout> 0.577f)Uout= 0.577f;
//	if(Uout<-0.577f)Uout=-0.577f;
	
	sector = (angle / _PI_3) + 1;                      //确定每一个扇形
	T1 = _SQRT3 * arm_sin_f32(sector*_PI_3 - angle) * Uout;   //_PI_3确定改扇形区域
	T2 = _SQRT3 * arm_sin_f32(angle - (sector-1.0)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;
	
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	TAA = Ta * (htim1.Init.Period + 1);
	TBB = Tb * (htim1.Init.Period + 1);
	TCC = Tc * (htim1.Init.Period + 1);
	if(TAA > htim1.Init.Period-50)
	{
		TAA = htim1.Init.Period-51;
	}
	if(TBB > htim1.Init.Period-50)
	{
		TBB = htim1.Init.Period-51;
	}
	if(TCC > htim1.Init.Period-50)
	{
		TCC = htim1.Init.Period-51;
	}
	htim1.Instance->CCR1 = TAA;
	htim1.Instance->CCR2 = TBB;
	htim1.Instance->CCR3 = TCC;
	
//	htim1.Instance->CCR1 = Ta * (htim1.Init.Period + 1);
//	htim1.Instance->CCR2 = Tb * (htim1.Init.Period + 1);
//	htim1.Instance->CCR3 = Tc * (htim1.Init.Period + 1);
	
}


uint8_t Align_Sensor(void)
{
    int16_t i;
    float angle;
    float Mid_angle,End_angle;
    float Moved;

    printf("MOT: Align sensor.\r\n");

    if(sensor_direction == UNKNOWN)  //没有设置，需要检测
	{
        for(i=0; i<=500; i++)                                            
		{
			angle = _3PI_2 + _2PI * i / 500.0f;                           //_3PI_2是为了保证电机能够转回0度
			setPhaseVoltage(Motor_init.voltage_sensor_align, 0, angle);  //类似开环转动
			delay_ms(2);
		}
		Mid_angle = AS5600_Get_Angle();

		for(i=500; i>=0; i--) 
		{
			angle = _3PI_2 + _2PI * i / 500.0f ;
			setPhaseVoltage(Motor_init.voltage_sensor_align, 0, angle);
			delay_ms(2);
		}
		End_angle = AS5600_Get_Angle();

		setPhaseVoltage(0, 0, 0);
		delay_ms(200);

		printf("Mid_angle=%.4f\r\n",Mid_angle);
		printf("End_angle=%.4f\r\n",End_angle);

		Moved = fabs(Mid_angle - End_angle);
		if((Mid_angle == End_angle)||(Moved < 0.01f))  //相等或者几乎没有动
		{
			printf("MOT: Failed to notice movement loop222.\r\n");
			M1_Disable;    							//电机检测不正常，关闭驱动
			return 0;
		}
		else if(Mid_angle < End_angle)
		{
			printf("MOT: sensor_direction==CCW\r\n");
			sensor_direction=CCW;
		}
		else
		{
			printf("MOT: sensor_direction==CW\r\n");
			sensor_direction=CW;
		}

		printf("MOT: PP check: \r\n");    //计算Pole_Pairs
		if( fabs(Moved * Motor_init.pole_pairs - _2PI) > 0.5f )  // 0.5 is arbitrary number it can be lower or higher!
		{
			printf("fail - estimated pp:\r\n");
//			Motor_init.pole_pairs=_2PI/Moved+0.5f;     //浮点数转整形，四舍五入
			printf("pole_pairs = %d\r\n",Motor_init.pole_pairs);
		}
		else 
			printf("PP Check OK!\r\n");
    }
	else
		printf("MOT: Skip dir calib.\r\n");

	if(zero_electric_angle == 0)  //没有设置，需要检测
	{
		setPhaseVoltage(Motor_init.voltage_sensor_align, 0,  _3PI_2);  //计算零点偏移角度
		delay_ms(700);
		zero_electric_angle = _normalizeAngle(_electricalAngle(sensor_direction * AS5600_Get_Angle(), Motor_init.pole_pairs));
		delay_ms(20);
		printf("MOT: Zero elec. angle:\r\n");
		printf("%.4f\r\n",zero_electric_angle);
		setPhaseVoltage(0, 0, 0);
		delay_ms(200);
	}
	else
		printf("MOT: Skip offset calib.\r\n");

	return 1;
}

void Motor_Init(void)
{
    printf("Motor Start Init\r\n");

    if(Motor_init.voltage_sensor_align > Motor_init.voltage_limit)    //防止校准电流过大
    {
        Motor_init.voltage_sensor_align = Motor_init.voltage_limit;
    }

    PID_Current_q.limit = Motor_init.voltage_limit;         //Iq和Id最终会反馈成电压后输出至SVPWM，因此使用电压限制他。
	PID_Current_d.limit = Motor_init.voltage_limit;
    PID_Velocity.limit = Motor_init.current_limit;          //通过设定的最大电流来给PID的反馈速度进行一个的限制
	PID_Angle.limit = Motor_init.velocity_limit;

    M1_Enable;
	
	printf("MOT: Enable driver.\r\n");
}

void Motor_FOC_Init(float zero_electric_offset,Direction _sensor_direction)
{
    if(zero_electric_offset != 0)
    {
        zero_electric_angle = zero_electric_offset;
    }
    if(_sensor_direction != 0)
    {
        sensor_direction = _sensor_direction;
    }

	Align_Sensor();
	//轴角度更新
	angle_prev = AS5600_Get_Angle();
	delay_ms(50);
	shaft_velocity = shaftVelocity();
	delay_ms(5);
	shaft_angle = shaftAngle();
	if(Controller == Type_angle)
	{
		// target=shaft_angle;
	}          //角度模式，以当前的角度为目标角度，进入主循环后电机静止
	delay_ms(200);
}

void LoopFOC(void)
{
	if( Controller==Type_angle_openloop || Controller==Type_velocity_openloop ) return;
	
	shaft_angle = shaftAngle();
	electrical_angle = electricalAngle();

	Current = getFOCCurrents(electrical_angle);
	Current.q = LPF_Operator(&LPF_current_q,Current.q);
	Current.d = LPF_Operator(&LPF_current_d,Current.d);

	Voltage.q = PID_Operator(&PID_Current_q,(current_sp - Current.q));
	Voltage.d = PID_Operator(&PID_Current_d,-Current.d);

	setPhaseVoltage(Voltage.q, Voltage.d, electrical_angle);
}

float Velocity_Openloop(float Target_velocity)
{
	unsigned long Now_us;
	float Sample_time,Uq;

	Now_us = SysTick->VAL;
	if(Now_us < open_loop_timestamp)
	{
		Sample_time = (float)(open_loop_timestamp - Now_us) / 168 * 1e-6f;
	}
	else
	{
		Sample_time = (float)(0xFFFFFF - Now_us + open_loop_timestamp) / 168 * 1e-6f;            //简单来说就是看看过了多久
	}
	open_loop_timestamp = Now_us;
	if(Sample_time == 0 || Sample_time > 0.5f) Sample_time = 1e-3f;                      //解决奇怪溢出

	shaft_angle = _normalizeAngle(shaft_angle + Target_velocity * Sample_time);

	Uq = Motor_init.voltage_limit;

	setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, Motor_init.pole_pairs));

	return Uq;
}

//float Angle_Openloop(float Target_angle)
//{

//}

void Move(float New_Target)
{
	shaft_velocity = shaftVelocity();
	
	switch(Controller)
	{
		case Type_torque:
		current_sp = New_Target;
			break;
		case Type_velocity:
		shaft_velocity_sp = New_Target;
		current_sp = PID_Operator(&PID_Velocity,(shaft_velocity_sp - shaft_velocity));          //计算扭矩通过扭矩控制速度
			break;
		case Type_angle:
		shaft_angle_sp = New_Target;
		shaft_velocity_sp = PID_Operator(&PID_Angle,(shaft_angle_sp - shaft_angle));
		current_sp = PID_Operator(&PID_Velocity,(shaft_velocity_sp - shaft_velocity));
			break;
		case Type_velocity_openloop:
		shaft_velocity_sp = New_Target;
		Voltage.q = Velocity_Openloop(shaft_velocity_sp); 	// returns the voltage that is set to the motor
		Voltage.d = 0;
			break;
		case Type_angle_openloop:
		shaft_angle_sp = New_Target;
//		Voltage.q = Angle_Openloop(shaft_angle_sp); 		// returns the voltage that is set to the motor
		Voltage.d = 0;
			break;
	}
}
