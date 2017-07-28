#ifndef __UART6050_H
#define __UART6050_H

#include "string.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"

void Uart2_NVIC(void) ;   //��ʼ�� ����USART2
void Read_MPUData(void);    //��ȡMPU��ֵ
void NVIC_Configuration(void);//uart2�жϳ�ʼ��
void UART6050_Init(void);//���������ǳ�ʼ

extern float pitch_zero, roll_zero;
extern float a[3],w[3],angle[3],T;

#endif	   

