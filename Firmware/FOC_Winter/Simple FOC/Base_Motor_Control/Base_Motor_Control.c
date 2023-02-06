#include "Base_Motor_Control.h"

Motor_Base Motor_init;

void Base_parameter_init(void)
{
	Motor_init.voltage_power_supply=12;   //V 电源电压
	Motor_init.pole_pairs=7;              //电机极对数，按照实际设置，虽然可以上电检测但有失败的概率
	Motor_init.voltage_sensor_align=3;    //V alignSensor() use it，大功率电机设置的值小一点比如0.5-1，小电机设置的大一点比如2-3
	Motor_init.voltage_limit=10;           //V，主要为限制电机最大电流，最大值需小于12/1.732=6.9
	Motor_init.velocity_limit=20;         //rad/s 角度模式时限制最大转速，力矩模式和速度模式不起作用
	Motor_init.current_limit=50;          //A，foc_current和dc_current模式限制电流，不能为0。速度模式和位置模式起作用
}
