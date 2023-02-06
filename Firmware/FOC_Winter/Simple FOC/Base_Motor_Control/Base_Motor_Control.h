#ifndef __Base_Motor_Control_H__
#define __Base_Motor_Control_H__

#include "main.h"

#define M1_Enable HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
#define M1_Disable HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);

typedef struct
{
	float voltage_power_supply;
	uint8_t pole_pairs;
	float voltage_sensor_align;
	float voltage_limit;
	float velocity_limit;
	float current_limit;
} Motor_Base;

extern Motor_Base Motor_init;

void Base_parameter_init(void);

#endif
