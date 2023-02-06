#ifndef __delay_JR__H
#define __delay_JR__H

#include "main.h"
#include "core_cm4.h"


void delay_init(uint8_t SYSCLK);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void ll(void);
#endif

