#include "GUI.h"

extern float Target,ADC_Reg_Value,shaft_angle;

void GUI_display(void)
{
//		LCD_ShowChinese(0,0,"中园子",RED,WHITE,32,0);
	LCD_ShowString(0,40,"Mode:",RED,LGRAY,16,0);
	switch(Controller)
	{
		case Type_torque:LCD_ShowString(50,40,"Type_torque             ",RED,LGRAY,16,0);
			break;
		case Type_velocity:LCD_ShowString(50,40,"Type_velocity           ",RED,LGRAY,16,0);
			break;
		case Type_angle:LCD_ShowString(50,40,"Type_angle           ",RED,LGRAY,16,0);
			break;
		case Type_velocity_openloop:LCD_ShowString(50,40,"Type_velocity_openloop",RED,LGRAY,16,0);
			break;
		case Type_angle_openloop:LCD_ShowString(50,40,"Type_angle_openloop",RED,LGRAY,16,0);
			break;
	}
	LCD_ShowString(0,70,"Target:",RED,LGRAY,16,0);
	LCD_ShowFloatNum1(60,70,Target,5,RED,LGRAY,16);
	LCD_ShowFloatNum1(60,100,ADC_Reg_Value,5,RED,LGRAY,16);
	LCD_ShowFloatNum1(60,130,shaft_angle,5,RED,LGRAY,16);
//		LCD_ShowString(0,70,"Increaseing Nun:",RED,WHITE,16,0);
//		LCD_ShowFloatNum1(128,70,t,4,RED,WHITE,16);
//		t+=0.11f;
//		for(j=0;j<3;j++)
//		{
//			for(k=0;k<6;k++)
//			{
////				LCD_ShowPicture(40*k,120+j*40,40,40,gImage_1);
//			}
//		}

}
