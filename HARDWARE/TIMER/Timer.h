#ifndef __TIMER_H__
#define __TIMER_H__
#include "stm32f10x.h"

extern uint8_t CurMode;

void TIM5_Config(unsigned short int Period,unsigned short int Prescaler);
void TIM5_IRQHandler(void);
#endif


