#ifndef __AS5600_H
#define __AS5600_H

#include "main.h"
#include "delay_JR.h"
#include "stdio.h"

#define AS5600_RESOLUTION 4096 //12bit Resolution 
#define _2PI 6.28318530718f

#define AS5600_AGREEMENT_MODE 1
/*
		0硬件IIC
		1软件IIC
*/

extern float angle_prev;

#if AS5600_AGREEMENT_MODE == 0

#include "i2c.h"

#define AS5600_I2C_HANDLE hi2c1

#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)

#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1



#define AS5600_RAW_ANGLE_HIGH_REGISTER  0x0C
#define AS5600_RAW_ANGLE_LOW_REGISTER  0x0D



void AS5600_lnit(void);
float AS5600_Get_Angle(void);
uint16_t AS5600_Get_Raw_Angle(void);
float Get_Velocity(void);

#elif AS5600_AGREEMENT_MODE == 1

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 
#define GPIOE_ODR_Addr    (GPIOE_BASE+20) //0x40021014 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOG_ODR_Addr    (GPIOG_BASE+20) //0x40021814   
#define GPIOH_ODR_Addr    (GPIOH_BASE+20) //0x40021C14    
#define GPIOI_ODR_Addr    (GPIOI_BASE+20) //0x40022014 
#define GPIOJ_ODR_ADDr    (GPIOJ_BASE+20) //0x40022414
#define GPIOK_ODR_ADDr    (GPIOK_BASE+20) //0x40022814

#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 
#define GPIOE_IDR_Addr    (GPIOE_BASE+16) //0x40021010 
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define GPIOG_IDR_Addr    (GPIOG_BASE+16) //0x40021810 
#define GPIOH_IDR_Addr    (GPIOH_BASE+16) //0x40021C10 
#define GPIOI_IDR_Addr    (GPIOI_BASE+16) //0x40022010 
#define GPIOJ_IDR_Addr    (GPIOJ_BASE+16) //0x40022410 
#define GPIOK_IDR_Addr    (GPIOK_BASE+16) //0x40022810 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PH5����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PH5���ģʽ
 
//IO��������	 
#define IIC_SCL    PBout(6) //SCL���
#define IIC_SDA    PBout(7) //SDA���	 
#define READ_SDA   PBin(7)  //SDA����
 
#define	_raw_ang_hi 0x0c
#define	_raw_ang_lo 0x0d

u16 AS5600_ReadTwoByte(u16 ReadAddr_hi,u16 ReadAddr_lo);
void AS5600_lnit(void);
float AS5600_Get_Angle(void);
float Get_Velocity(void);

#endif

#endif
