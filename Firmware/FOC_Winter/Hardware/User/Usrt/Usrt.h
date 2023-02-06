#ifndef __Usrt_H__
#define __Usrt_H__

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "stdlib.h"
#include "stdio.h"
#include "FOC_Motor.h"
#include "PID_Motor.h"

extern DMA_HandleTypeDef hdma_usart1_rx;

#define BUFFER_SIZE 1000
extern volatile uint32_t rx1_len;        //接收一帧数据的长度
extern volatile uint8_t rec1_end_flag;   //一帧数据接收完成标志
extern uint8_t rx1_buffer[BUFFER_SIZE];  //接收数据缓存数组

void Usart1_Handle(void);
void DMA_Usart1_Send(uint8_t *buf,uint8_t len);//串口发送自己
void Usart1_IDLE(void);

#endif 

