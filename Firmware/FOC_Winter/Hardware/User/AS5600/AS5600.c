#include "AS5600.h"

#define abs(x) ((x)>0?(x):-(x))

float angle_prev;
static float angle_data_prev; //上次位置
static float full_rotation_offset; //转过的整圈数
unsigned long velocity_calc_timestamp; //计算速度时间采样

#if AS5600_AGREEMENT_MODE == 0


static int IIC_Write(uint8_t Addr,uint8_t *pData,uint32_t count)
{
	int status;
	int IIC_Time_Out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
	
	status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, Addr, pData, count, IIC_Time_Out);
	return status;
}

static int IIC_Read(uint8_t Addr,uint8_t *pData,uint32_t count)
{
	int status;
	int IIC_Time_Out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
	
	status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, (Addr | 1), pData, count, IIC_Time_Out);
	return status;
}

uint16_t AS5600_Get_Raw_Angle(void)      //直接读取寄存器的未经处理
{
	uint16_t Raw_angle;
	uint8_t Buffer[2] = {0};
	uint8_t raw_angle_register = AS5600_RAW_ANGLE_HIGH_REGISTER;
	uint8_t raw_angle_low_register = AS5600_RAW_ANGLE_LOW_REGISTER;
	
	IIC_Write(AS5600_ADDR, &raw_angle_register, 1);
	IIC_Read(AS5600_ADDR, &Buffer[0], 1);
	IIC_Write(AS5600_ADDR, &raw_angle_low_register, 1);
	IIC_Read(AS5600_ADDR, &Buffer[1], 1);
	Raw_angle = ((uint16_t)Buffer[0] << 8) | (uint16_t)Buffer[1];
	return Raw_angle;
}

float AS5600_Get_Angle(void)
{
	float Angle_data = AS5600_Get_Raw_Angle();
	float Angle_Error = Angle_data - angle_data_prev;

	if(abs(Angle_Error) > (0.8f * AS5600_RESOLUTION))
	{
		full_rotation_offset += (Angle_Error > 0 ? -_2PI : _2PI);
	}
	
	angle_data_prev = Angle_data;
	
	return (full_rotation_offset + (Angle_data) / (float)(AS5600_RESOLUTION) * _2PI);
}

float Get_Velocity(void)
{
	unsigned long Now_us;
	float Sample_time,Angle_now,Velocity;
	
	Now_us = SysTick->VAL;
	if(Now_us < velocity_calc_timestamp)
	{
		Sample_time = (float)(velocity_calc_timestamp - Now_us) / 168 * 1e-6f;
	}
	else
		Sample_time = (float)(0xFFFFFF - Now_us + velocity_calc_timestamp) / 168 * 1e-6f;            //简单来说就是看看过了多久
	if(Sample_time == 0 || Sample_time > 0.5f) Sample_time = 1e-3f;                                    //解决奇怪溢出
	
//	printf("Smtime = %f\r\n",Sample_time);
	Angle_now = AS5600_Get_Angle();
	
	Velocity = (Angle_now - angle_prev) / Sample_time;
	
	angle_prev = Angle_now;
	velocity_calc_timestamp = Now_us;
	
	return Velocity;
}

void AS5600_lnit(void)
{
	full_rotation_offset = 0;
	angle_data_prev = AS5600_Get_Raw_Angle();
}

#elif AS5600_AGREEMENT_MODE == 1

void IIC_Start(void)
{
	SDA_OUT();    
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);	  	  
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	delay_us(4);
 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
	delay_us(4);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
}	  

void IIC_Stop(void)
{
	SDA_OUT();
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
 	delay_us(4);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
	delay_us(4);
}


u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);delay_us(1);	   
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);delay_us(1);	 
	while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);	   
	return 0;  
} 

void IIC_Ack(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	SDA_OUT();
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
}
    
void IIC_NAck(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
	SDA_OUT();
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	delay_us(2);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
}					 				     


void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
		txd<<=1; 	  
		delay_us(2);   
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
		delay_us(2); 
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);	
		delay_us(2);
    }	 
} 	    


u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET); 
        delay_us(2);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
        receive<<=1;
        if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7))receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();   
    return receive;
}
 

u8 AS5600_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=-1;		  	    																 
  IIC_Start();  
	IIC_Send_Byte((0X36<<1)|0x00);	  
	IIC_Wait_Ack(); 
  IIC_Send_Byte(ReadAddr);   
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte((0X36<<1)|0x01);       		   
	IIC_Wait_Ack();	 
  temp=IIC_Read_Byte(0);		   
  IIC_Stop();    
	return temp;
}
 

void AS5600_WriteOneByte(u16 WriteAddr,u8 WriteData)
{				  	  	    																 
  IIC_Start();  
	IIC_Send_Byte((0X36<<1)|0x00);	 
	IIC_Wait_Ack(); 
  IIC_Send_Byte(WriteAddr); 
	IIC_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(WriteData);      	   
	IIC_Wait_Ack();	 	   
  IIC_Stop();	    
	delay_ms(10);
}
 

u16 AS5600_ReadTwoByte(u16 ReadAddr_hi,u16 ReadAddr_lo)
{
	u16 TwoByte_Data=-1;
	u8 hi_Data=0,lo_Data=0;
	hi_Data=AS5600_ReadOneByte(ReadAddr_hi);
	lo_Data=AS5600_ReadOneByte(ReadAddr_lo);
	TwoByte_Data = (hi_Data<<8)|lo_Data;
	return TwoByte_Data;

}

void AS5600_lnit(void)
{
	full_rotation_offset = 0;
	angle_data_prev = AS5600_ReadTwoByte(_raw_ang_hi, _raw_ang_lo);
}

float AS5600_Get_Angle(void)
{
	float Angle_data = AS5600_ReadTwoByte(_raw_ang_hi, _raw_ang_lo);
	float Angle_Error = Angle_data - angle_data_prev;

	if(abs(Angle_Error) > (0.8f * AS5600_RESOLUTION))
	{
		full_rotation_offset += (Angle_Error > 0 ? -_2PI : _2PI);
	}
	
	angle_data_prev = Angle_data;
	
	return (full_rotation_offset + (Angle_data) / (float)(AS5600_RESOLUTION) * _2PI);
}

float Get_Velocity(void)
{
	unsigned long Now_us;
	float Sample_time,Angle_now,Velocity;
	
	Now_us = SysTick->VAL;
	if(Now_us < velocity_calc_timestamp)
	{
		Sample_time = (float)(velocity_calc_timestamp - Now_us) / 168 * 1e-6f;
	}
	else
		Sample_time = (float)(0xFFFFFF - Now_us + velocity_calc_timestamp) / 168 * 1e-6f;            //简单来说就是看看过了多久
	if(Sample_time == 0 || Sample_time > 0.5f) Sample_time = 1e-3f;                                    //解决奇怪溢出
	
//	printf("Smtime = %f\r\n",Sample_time);
	Angle_now = AS5600_Get_Angle();
	
	Velocity = (Angle_now - angle_prev) / Sample_time;
	
	angle_prev = Angle_now;
	velocity_calc_timestamp = Now_us;
	
	return Velocity;
}

#endif

